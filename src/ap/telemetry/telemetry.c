/*
 * telemetry.c
 *
 * Orion BMS and Bamocar CAN decoder with a display-facing signal snapshot.
 * Persistent CAN logging is owned by the external TMA-1/ESP logger.
 */

#include "telemetry.h"

#include "can.h"

/*
 * Bamocar 응답 프레임 식별자.
 *
 * 원본(APPS 보드)은 bamocar_can.h 에서 가져다 썼지만, 이 보드는 인버터에
 * 명령을 보내지 않고 버스에 흐르는 응답만 듣는다. 명령 계층 전체를 끌어올
 * 이유가 없어 필요한 두 값만 여기 둔다. APPS 보드의 bamocar_can.h 와 값이
 * 갈라지면 안 되므로 바꿀 일이 생기면 양쪽을 같이 고칠 것.
 */
#define BAMOCAR_CAN_DEFAULT_TX_ID             0x181U
#define BAMOCAR_REG_STATUS                    0x40U

#ifdef _USE_HW_CLI
#include "cli.h"
#endif

#define TELEMETRY_BAMOCAR_REG_ERROR_WARNING   0x8FU
#define TELEMETRY_BAMOCAR_REG_IGBT_TEMP       0x4AU
#define TELEMETRY_BAMOCAR_REG_DC_BUS          0xEBU

/* BAMOCAR 700 V model: manual page 49 says 1 V ~= 31.5 Num for DC bus. */
#define TELEMETRY_BAMOCAR_DC_BUS_NUM_PER_10V  315L

/*
 * Verify these register IDs and scales against the actual NDrive setup.
 * Speed, torque and motor-temperature scaling are installation-dependent.
 */
#define TELEMETRY_BAMOCAR_REG_N_ACTUAL        0x30U
#define TELEMETRY_BAMOCAR_REG_TORQUE_ACTUAL   0xA8U
#define TELEMETRY_BAMOCAR_REG_MOTOR_TEMP      0x49U
#define TELEMETRY_BAMOCAR_N_MAX_RPM           6500L

/*
 * Orion BMS 2 CAN frames are user-configured. This decoder expects:
 *
 *   CAN ID 0x6B0, standard, 8 bytes, big-endian
 *     [0..1] Pack voltage     u16, 0.1 V/bit
 *     [2..3] Pack current     i16, 0.1 A/bit (+ = discharge)
 *     [4..5] Pack SOC         u16, 0.5 %/bit
 *     [6..7] High temperature i16, 1 degC/bit
 */
#define TELEMETRY_ORION_CAN_ID                0x6B0U
#define TELEMETRY_ORION_VOLT_MV_PER_BIT       100L
#define TELEMETRY_ORION_CURR_MA_PER_BIT       100L
#define TELEMETRY_ORION_SOC_PERMILLE_PER_BIT  5L
#define TELEMETRY_ORION_TEMP_DECIC_PER_BIT    10L

/* Orion's optional battery-cell broadcast sends one cell per frame. The BMS
 * profile must enable this broadcast on the vehicle CAN interface and use the
 * same standard identifier. Instant voltage is big-endian and scaled at
 * 0.1 mV/bit. */
#define TELEMETRY_ORION_CELL_CAN_ID            0x36U
#define TELEMETRY_ORION_CELL_VOLT_MAX_DECIMV   50000U
#define TELEMETRY_ORION_CELL_TIMEOUT_MS         3000U

typedef struct
{
  uint16_t raw;
  int16_t  temp_decic;
} telemetry_temp_point_t;

/*
 * 소스별 수신 진단.
 *
 * 값이 안 뜰 때 원인이 셋 중 어디인지 — 버스가 죽었는지, 그 노드만 조용한지,
 * 프레임은 오는데 디코딩이 안 맞는지 — 를 구분하려면 ID 단위로 세어야 한다.
 * 특히 Bamocar 는 누군가 0x201 로 반복 전송을 요청해 줘야 0x181 을 뿌리기
 * 시작한다. 이 보드는 요청을 보내지 않으므로, 0x181 이 0 프레임이면 우리
 * 잘못이 아니라 APPS 보드가 요청을 안 걸었다는 뜻이다.
 */
typedef enum
{
  TELEMETRY_SRC_ORION = 0,
  TELEMETRY_SRC_ORION_CELL,
  TELEMETRY_SRC_BAMOCAR,
  TELEMETRY_SRC_MAX
} telemetry_src_t;

typedef struct
{
  uint32_t frame_cnt;
  uint32_t last_ms;
} telemetry_src_stat_t;

typedef struct
{
  bool                        initialized;
  telemetry_signal_snapshot_t signal;
  bool                        cell_scan_active;
  bool                        cell_scan_contiguous;
  uint8_t                     cell_scan_last_id;
  uint16_t                    cell_scan_count;
  uint16_t                    cell_scan_min_decimv;
  bool                        cell_expected_last_id_valid;
  uint8_t                     cell_expected_last_id;
  uint32_t                    cell_min_last_update_ms;

  telemetry_src_stat_t        src[TELEMETRY_SRC_MAX];
  uint32_t                    rx_isr_cnt;
} telemetry_t;

static telemetry_t telemetry;

static const telemetry_temp_point_t bamocar_igbt_temp_table[] =
{
  {16308U, -300}, {16387U, -250}, {16487U, -200}, {16609U, -150},
  {16757U, -100}, {16938U,  -50}, {17151U,    0}, {17400U,   50},
  {17688U,  100}, {18017U,  150}, {18387U,  200}, {18797U,  250},
  {19247U,  300}, {19733U,  350}, {20250U,  400}, {20793U,  450},
  {21357U,  500}, {21933U,  550}, {22515U,  600}, {23097U,  650},
  {23671U,  700}, {24232U,  750}, {24775U,  800}, {25296U,  850},
  {25792U,  900}, {26261U,  950}, {26702U, 1000}, {27114U, 1050},
  {27497U, 1100}, {27851U, 1150}, {28179U, 1200}, {28480U, 1250},
};

#ifdef _USE_HW_CLI
static void cliTelemetry(cli_args_t *args);
#endif

/*
 * 디코딩은 수신 인터럽트 안에서 돈다(telemetryCanRxHandler 주석 참고).
 * 그래서 telemetry.signal 은 ISR 과 메인 루프가 같이 만지는 상태다.
 * 스냅샷을 통째로 복사하거나 valid_mask 를 read-modify-write 하는 동안에는
 * 인터럽트를 잠깐 막는다. 복사량이 60 바이트 남짓이라 지연은 무시할 만하다.
 */
static uint32_t telemetryEnterCritical(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();

  return primask;
}

static void telemetryExitCritical(uint32_t primask)
{
  __set_PRIMASK(primask);
}

static void telemetryMarkSource(telemetry_src_t src, uint32_t timestamp)
{
  telemetry.src[src].frame_cnt++;
  telemetry.src[src].last_ms = timestamp;
}

static uint16_t telemetryReadU16Le(const uint8_t *p_data)
{
  return (uint16_t)p_data[0] | ((uint16_t)p_data[1] << 8);
}

static uint32_t telemetryReadU32Le(const uint8_t *p_data)
{
  return (uint32_t)p_data[0] |
         ((uint32_t)p_data[1] << 8) |
         ((uint32_t)p_data[2] << 16) |
         ((uint32_t)p_data[3] << 24);
}

static uint16_t telemetryReadU16Be(const uint8_t *p_data)
{
  return ((uint16_t)p_data[0] << 8) | (uint16_t)p_data[1];
}

static int16_t telemetryBamocarTempRawToDeciC(uint16_t raw)
{
  const uint32_t count =
      sizeof(bamocar_igbt_temp_table) / sizeof(bamocar_igbt_temp_table[0]);

  if (raw <= bamocar_igbt_temp_table[0].raw)
  {
    return bamocar_igbt_temp_table[0].temp_decic;
  }
  if (raw >= bamocar_igbt_temp_table[count - 1U].raw)
  {
    return bamocar_igbt_temp_table[count - 1U].temp_decic;
  }

  for (uint32_t i = 1U; i < count; i++)
  {
    if (raw <= bamocar_igbt_temp_table[i].raw)
    {
      int32_t raw0 = bamocar_igbt_temp_table[i - 1U].raw;
      int32_t raw1 = bamocar_igbt_temp_table[i].raw;
      int32_t temp0 = bamocar_igbt_temp_table[i - 1U].temp_decic;
      int32_t temp1 = bamocar_igbt_temp_table[i].temp_decic;

      return (int16_t)(temp0 +
          (((int32_t)raw - raw0) * (temp1 - temp0)) / (raw1 - raw0));
    }
  }

  return 0;
}

static bool telemetryDecodeOrion(const can_msg_t *p_msg)
{
  uint16_t raw;

  if ((p_msg->id_type != CAN_STD) ||
      (p_msg->id != TELEMETRY_ORION_CAN_ID) ||
      (p_msg->length < 8U))
  {
    return false;
  }

  raw = telemetryReadU16Be(&p_msg->data[0]);
  telemetry.signal.bms_pack_mv =
      (int32_t)raw * TELEMETRY_ORION_VOLT_MV_PER_BIT;
  telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BMS_PACK_MV;

  raw = telemetryReadU16Be(&p_msg->data[2]);
  telemetry.signal.bms_pack_ma =
      (int32_t)(int16_t)raw * TELEMETRY_ORION_CURR_MA_PER_BIT;
  telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BMS_PACK_MA;

  raw = telemetryReadU16Be(&p_msg->data[4]);
  telemetry.signal.bms_soc_permille =
      (uint16_t)((int32_t)raw * TELEMETRY_ORION_SOC_PERMILLE_PER_BIT);
  telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BMS_SOC_PERMILLE;

  raw = telemetryReadU16Be(&p_msg->data[6]);
  telemetry.signal.bms_max_temp_decic =
      (int16_t)((int32_t)(int16_t)raw * TELEMETRY_ORION_TEMP_DECIC_PER_BIT);
  telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BMS_MAX_TEMP_DECIC;

  telemetryMarkSource(TELEMETRY_SRC_ORION, p_msg->timestamp);
  telemetry.signal.last_update_ms = p_msg->timestamp;
  return true;
}

/*
 * 스캔이 완성되기를 기다리지 않고 곧바로 내리는 예외.
 *
 * 완전한 스캔만 발표하는 규칙은 "일부만 받은 스캔의 최소값은 실제 최소보다
 * 높을 수 있다", 즉 실제보다 안전해 보인다는 이유에서 나왔다. 그 논리는 값이
 * 올라갈 때만 성립한다. 지금 표시 중인 값보다 낮은 셀을 실제로 봤다면 그것은
 * 스캔이 온전한지와 무관하게 참이고, 늦출수록 위험한 쪽이다. 그래서 내려가는
 * 방향으로는 즉시 반영하고, 올라가는 방향은 종전대로 완전한 스캔만 허용한다.
 *
 * 이미 유효한 값이 있을 때만 적용한다. 첫 값까지 이렇게 잡으면 "본 것 중
 * 최소"를 팩 최소로 발표하게 되어, 막으려던 그 과대평가를 그대로 하게 된다.
 */
static void telemetryCellMinLowerIfSeen(uint16_t voltage_decimv,
                                        uint32_t timestamp)
{
  uint16_t mv;

  if ((telemetry.signal.valid_mask & TELEMETRY_SIGNAL_BMS_MIN_CELL_MV) == 0U)
  {
    return;
  }

  mv = (uint16_t)((voltage_decimv + 5U) / 10U);

  if (mv < telemetry.signal.bms_min_cell_mv)
  {
    telemetry.signal.bms_min_cell_mv = mv;
    telemetry.cell_min_last_update_ms = timestamp;
  }
}

static bool telemetryDecodeOrionCell(const can_msg_t *p_msg)
{
  uint32_t checksum;
  uint16_t voltage_decimv;
  uint8_t cell_id;

  if ((p_msg->id_type != CAN_STD) ||
      (p_msg->id != TELEMETRY_ORION_CELL_CAN_ID) ||
      (p_msg->length != 8U))
  {
    return false;
  }

  checksum = p_msg->id + p_msg->length;
  for (uint8_t i = 0U; i < 7U; i++)
  {
    checksum += p_msg->data[i];
  }
  if ((uint8_t)checksum != p_msg->data[7])
  {
    return false;
  }

  cell_id = p_msg->data[0];
  voltage_decimv = telemetryReadU16Be(&p_msg->data[1]);
  if ((voltage_decimv == 0U) ||
      (voltage_decimv > TELEMETRY_ORION_CELL_VOLT_MAX_DECIMV))
  {
    return false;
  }

  telemetryMarkSource(TELEMETRY_SRC_ORION_CELL, p_msg->timestamp);
  telemetryCellMinLowerIfSeen(voltage_decimv, p_msg->timestamp);

  /* A new cell-zero frame closes the preceding scan. The first contiguous
   * scan learns the pack's last cell ID; later scans are published only when
   * they reach that same ID. A longer scan raises the learned boundary and is
   * itself used only for relearning, which prevents a dropped tail from making
   * the displayed minimum look safer than it really is. */
  if (cell_id == 0U)
  {
    const bool scan_contiguous =
        telemetry.cell_scan_active &&
        telemetry.cell_scan_contiguous &&
        (telemetry.cell_scan_count > 1U) &&
        (telemetry.cell_scan_count ==
         ((uint16_t)telemetry.cell_scan_last_id + 1U));

    if (scan_contiguous &&
        ((!telemetry.cell_expected_last_id_valid) ||
         (telemetry.cell_scan_last_id > telemetry.cell_expected_last_id)))
    {
      telemetry.cell_expected_last_id = telemetry.cell_scan_last_id;
      telemetry.cell_expected_last_id_valid = true;
    }
    else if (scan_contiguous &&
             (telemetry.cell_scan_last_id ==
              telemetry.cell_expected_last_id))
    {
      telemetry.signal.bms_min_cell_mv =
          (uint16_t)((telemetry.cell_scan_min_decimv + 5U) / 10U);
      telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BMS_MIN_CELL_MV;
      telemetry.cell_min_last_update_ms = p_msg->timestamp;
    }

    telemetry.cell_scan_active = true;
    telemetry.cell_scan_contiguous = true;
    telemetry.cell_scan_last_id = 0U;
    telemetry.cell_scan_count = 1U;
    telemetry.cell_scan_min_decimv = voltage_decimv;
    telemetry.signal.last_update_ms = p_msg->timestamp;
    return true;
  }

  if (telemetry.cell_scan_active != true)
  {
    return true;
  }

  if (cell_id != (uint8_t)(telemetry.cell_scan_last_id + 1U))
  {
    telemetry.cell_scan_contiguous = false;
  }
  telemetry.cell_scan_last_id = cell_id;
  telemetry.cell_scan_count++;
  if (voltage_decimv < telemetry.cell_scan_min_decimv)
  {
    telemetry.cell_scan_min_decimv = voltage_decimv;
  }
  telemetry.signal.last_update_ms = p_msg->timestamp;
  return true;
}

static bool telemetryDecodeBamocar(const can_msg_t *p_msg)
{
  uint8_t reg_id;
  uint16_t raw16;
  bool updated = false;

  if ((p_msg->id_type != CAN_STD) ||
      (p_msg->id != BAMOCAR_CAN_DEFAULT_TX_ID) ||
      (p_msg->length < 3U))
  {
    return false;
  }

  /*
   * 아는 레지스터든 아니든, 0x181 이 왔다는 것 자체가 "인버터가 방송 중"이라는
   * 정보다. 진단은 updated 와 무관하게 센다.
   */
  telemetryMarkSource(TELEMETRY_SRC_BAMOCAR, p_msg->timestamp);

  reg_id = p_msg->data[0];

  switch (reg_id)
  {
    case BAMOCAR_REG_STATUS:
      raw16 = telemetryReadU16Le(&p_msg->data[1]);
      telemetry.signal.bamocar_status = raw16;
      telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BAMOCAR_STATUS;
      updated = true;
      break;

    case TELEMETRY_BAMOCAR_REG_ERROR_WARNING:
      if (p_msg->length >= 5U)
      {
        telemetry.signal.bamocar_error_warning =
            telemetryReadU32Le(&p_msg->data[1]);
        telemetry.signal.valid_mask |=
            TELEMETRY_SIGNAL_BAMOCAR_ERROR_WARNING;
        updated = true;
      }
      break;

    case TELEMETRY_BAMOCAR_REG_DC_BUS:
      raw16 = telemetryReadU16Le(&p_msg->data[1]);
      telemetry.signal.bamocar_dc_bus_mv =
          (int32_t)(((int32_t)raw16 * 10000L) /
                    TELEMETRY_BAMOCAR_DC_BUS_NUM_PER_10V);
      telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BAMOCAR_DC_BUS_MV;
      updated = true;
      break;

    case TELEMETRY_BAMOCAR_REG_IGBT_TEMP:
      raw16 = telemetryReadU16Le(&p_msg->data[1]);
      telemetry.signal.bamocar_igbt_temp_decic =
          telemetryBamocarTempRawToDeciC(raw16);
      telemetry.signal.valid_mask |=
          TELEMETRY_SIGNAL_BAMOCAR_IGBT_TEMP_DECIC;
      updated = true;
      break;

    case TELEMETRY_BAMOCAR_REG_N_ACTUAL:
      raw16 = telemetryReadU16Le(&p_msg->data[1]);
      telemetry.signal.bamocar_rpm =
          ((int32_t)(int16_t)raw16 * TELEMETRY_BAMOCAR_N_MAX_RPM) / 32767L;
      telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BAMOCAR_RPM;
      updated = true;
      break;

    case TELEMETRY_BAMOCAR_REG_TORQUE_ACTUAL:
      raw16 = telemetryReadU16Le(&p_msg->data[1]);
      telemetry.signal.bamocar_torque_raw = (int32_t)(int16_t)raw16;
      telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BAMOCAR_TORQUE_RAW;
      updated = true;
      break;

    case TELEMETRY_BAMOCAR_REG_MOTOR_TEMP:
      raw16 = telemetryReadU16Le(&p_msg->data[1]);
      /*
       * The motor uses a KTY81/210 sensor, not the IGBT NTC. Keep the raw Num
       * until the NDrive motor-temperature conversion is confirmed.
       */
      telemetry.signal.bamocar_motor_temp_decic = (int16_t)raw16;
      telemetry.signal.valid_mask |=
          TELEMETRY_SIGNAL_BAMOCAR_MOTOR_TEMP_DECIC;
      updated = true;
      break;

    default:
      break;
  }

  if (updated == true)
  {
    telemetry.signal.last_update_ms = p_msg->timestamp;
  }

  return updated;
}

static void telemetryDecodeCan(const can_msg_t *p_msg)
{
  if (p_msg == NULL)
  {
    return;
  }

  (void)telemetryDecodeOrion(p_msg);
  (void)telemetryDecodeOrionCell(p_msg);
  (void)telemetryDecodeBamocar(p_msg);
}

/*
 * 수신 인터럽트에서 곧바로 디코딩한다.
 *
 * 메인 루프는 TouchGFX 가 한 프레임을 그리는 동안 수십 ms 씩 막힌다. 그동안에도
 * 차량 CAN 은 계속 프레임을 올리는데 수신 큐는 32 칸뿐이고, RAM 이 5 KB 남짓
 * 남아 있어 크게 늘릴 수도 없다. 넘친 프레임은 조용히 버려졌다. 셀 브로드캐스트
 * 처럼 한 덩어리로 몰려오는 신호는 그 유실 때문에 스캔이 늘 깨졌다.
 *
 * 그래서 값 반영은 큐를 거치지 않고 여기서 끝낸다. 큐가 넘치든 말든 숫자는
 * 이미 반영된 뒤다. 큐는 CLI(can read_test)가 원본 프레임을 들여다보는 용도로만
 * 남기므로 false 를 돌려준다. true 를 주면 드라이버가 방금 넣은 것을 도로 빼낸다.
 *
 * 디코딩은 분기 몇 개와 최대 32 회짜리 온도 테이블 탐색이 전부다. 500 kbps 에서
 * 프레임 간격이 아무리 짧아도 200 us 는 되므로 ISR 안에서 돌기에 충분히 짧다.
 */
static bool telemetryCanRxHandler(uint8_t ch, CanEvent_t evt, can_msg_t *p_msg)
{
  if ((evt != CAN_EVT_MSG) || (p_msg == NULL))
  {
    return false;
  }

  if ((telemetry.initialized != true) || (ch != telemetry.signal.can_ch))
  {
    return false;
  }

  telemetry.rx_isr_cnt++;
  telemetryDecodeCan(p_msg);

  return false;
}

bool telemetryConfigCanFilter(uint8_t ch)
{
  bool ret = true;

  /*
   * canOpen() 은 뱅크 0(표준)과 1(확장)에 전체 통과 필터를 깔고 나온다.
   * 그대로 두면 버스에 흐르는 모든 프레임이 인터럽트를 걸고 큐를 먹는다.
   * 우리가 실제로 읽는 것은 아래 셋뿐이므로 그 둘을 덮어쓰고 하나를 더 건다.
   */
  if (canConfigFilter(ch, 0, CAN_STD, CAN_ID_MASK,
                      TELEMETRY_ORION_CAN_ID, 0x7FFU) != true)
  {
    ret = false;
  }

  if (canConfigFilter(ch, 1, CAN_STD, CAN_ID_MASK,
                      TELEMETRY_ORION_CELL_CAN_ID, 0x7FFU) != true)
  {
    ret = false;
  }

  if (canConfigFilter(ch, 2, CAN_STD, CAN_ID_MASK,
                      BAMOCAR_CAN_DEFAULT_TX_ID, 0x7FFU) != true)
  {
    ret = false;
  }

  return ret;
}

bool telemetryInit(void)
{
  static bool cli_registered = false;

  /*
   * 상태를 지우는 동안 ISR 이 끼어들지 않게 먼저 떼어 낸다. 초기화는 부팅 때
   * 한 번이지만 telemetryUpdate() 에도 재초기화 경로가 있다.
   */
  canDetachRxInterrupt(_DEF_CAN1);

  memset(&telemetry, 0, sizeof(telemetry));
  telemetry.initialized = true;
  telemetry.signal.can_ch = _DEF_CAN1;

  canAttachRxInterrupt(telemetry.signal.can_ch, telemetryCanRxHandler);

#ifdef _USE_HW_CLI
  /* 재초기화 때마다 등록하면 명령 목록(16 칸)이 같은 이름으로 찬다. */
  if (cli_registered != true)
  {
    cli_registered = cliAdd("telemetry", cliTelemetry);
  }
#endif

  return true;
}

void telemetryUpdate(void)
{
  can_msg_t msg;
  uint32_t  available;
  uint32_t  primask;
  uint8_t   can_ch;

  if (telemetry.initialized != true)
  {
    (void)telemetryInit();
  }

  can_ch = telemetry.signal.can_ch;

  if (canIsOpen(can_ch) == true)
  {
    /*
     * 값은 수신 인터럽트에서 이미 반영됐다. 여기서는 큐를 비워 주기만 한다.
     * 비우지 않으면 큐가 가득 찬 채로 남아, CLI 로 프레임을 들여다볼 때
     * 한참 지난 것부터 나온다. CLI 루프가 도는 동안에는 이 함수가 불리지
     * 않으므로 그때 들어온 프레임은 그대로 남아 있다.
     */
    available = canMsgAvailable(can_ch);

    while (available > 0U)
    {
      (void)canMsgRead(can_ch, &msg);
      available--;
    }
  }

  /* valid_mask 는 ISR 도 만진다. 지우는 동안 갱신이 끼어들면 안 된다. */
  primask = telemetryEnterCritical();

  if (((telemetry.signal.valid_mask &
        TELEMETRY_SIGNAL_BMS_MIN_CELL_MV) != 0U) &&
      ((millis() - telemetry.cell_min_last_update_ms) >=
       TELEMETRY_ORION_CELL_TIMEOUT_MS))
  {
    telemetry.signal.valid_mask &= ~TELEMETRY_SIGNAL_BMS_MIN_CELL_MV;
  }

  telemetryExitCritical(primask);
}

bool telemetryGetSignalSnapshot(telemetry_signal_snapshot_t *p_snapshot)
{
  uint32_t primask;

  if (p_snapshot == NULL)
  {
    return false;
  }

  /*
   * ISR 이 중간에 끼어들면 서로 다른 시점의 값이 한 스냅샷에 섞인다.
   * 60 바이트 남짓 복사라 인터럽트를 막아 두는 시간은 무시할 만하다.
   */
  primask = telemetryEnterCritical();
  *p_snapshot = telemetry.signal;
  telemetryExitCritical(primask);

  return p_snapshot->valid_mask != 0U;
}

#ifdef _USE_HW_CLI
static void cliTelemetry(cli_args_t *args)
{
  bool handled = false;

  if ((args->argc == 1) && args->isStr(0, "info"))
  {
    static const char *src_name[TELEMETRY_SRC_MAX] =
    {
      "orion", "cell", "bamocar"
    };
    static const uint32_t src_id[TELEMETRY_SRC_MAX] =
    {
      TELEMETRY_ORION_CAN_ID,
      TELEMETRY_ORION_CELL_CAN_ID,
      BAMOCAR_CAN_DEFAULT_TX_ID,
    };
    uint32_t now_ms = millis();

    cliPrintf("decoder    : running (in rx isr)\n");
    cliPrintf("can_ch     : %d\n", telemetry.signal.can_ch);
    cliPrintf("rx_frames  : %lu\n",
              (unsigned long)canGetRxCount(telemetry.signal.can_ch));
    cliPrintf("decoded    : %lu\n", (unsigned long)telemetry.rx_isr_cnt);
    cliPrintf("valid_mask : 0x%08lX\n",
              (unsigned long)telemetry.signal.valid_mask);
    cliPrintf("storage    : external (TMA-1/ESP)\n");

    /*
     * 소스별로 나눠 세면 "값이 안 뜬다"의 원인이 바로 갈린다. 셋 다 0 이면
     * 버스나 우리 쪽 문제, 하나만 0 이면 그 노드가 조용한 것이다.
     */
    for (uint32_t i = 0U; i < (uint32_t)TELEMETRY_SRC_MAX; i++)
    {
      cliPrintf("%-8s0x%03X : %lu frames",
                src_name[i],
                (unsigned int)src_id[i],
                (unsigned long)telemetry.src[i].frame_cnt);

      if (telemetry.src[i].frame_cnt > 0U)
      {
        cliPrintf(", %lu ms ago\n",
                  (unsigned long)(now_ms - telemetry.src[i].last_ms));
      }
      else
      {
        cliPrintf(", never\n");
      }
    }

    if (telemetry.src[TELEMETRY_SRC_BAMOCAR].frame_cnt == 0U)
    {
      cliPrintf("hint       : no 0x181 yet. this board only listens;\n");
      cliPrintf("             the inverter stays silent until someone\n");
      cliPrintf("             sends the 0x201 repetition request.\n");
    }

    handled = true;
  }

  if ((args->argc == 1) && args->isStr(0, "signal"))
  {
    telemetry_signal_snapshot_t signal;
    bool valid = telemetryGetSignalSnapshot(&signal);

    cliPrintf("valid      : %d\n", valid);
    cliPrintf("valid_mask : 0x%08lX\n", (unsigned long)signal.valid_mask);
    cliPrintf("time_ms    : %lu\n", (unsigned long)signal.last_update_ms);
    cliPrintf("can_ch     : %d\n", signal.can_ch);
    cliPrintf("bms volt   : %ld mV\n", (long)signal.bms_pack_mv);
    cliPrintf("bms curr   : %ld mA\n", (long)signal.bms_pack_ma);
    cliPrintf("bms soc    : %u.%u %%\n",
              (unsigned int)(signal.bms_soc_permille / 10U),
              (unsigned int)(signal.bms_soc_permille % 10U));
    cliPrintf("bms temp   : %d.%d C\n",
              (int)(signal.bms_max_temp_decic / 10),
              (int)abs(signal.bms_max_temp_decic % 10));
    cliPrintf("bms min V  : %u.%03u V\n",
              (unsigned int)(signal.bms_min_cell_mv / 1000U),
              (unsigned int)(signal.bms_min_cell_mv % 1000U));
    cliPrintf("bam rpm    : %ld rpm\n", (long)signal.bamocar_rpm);
    cliPrintf("bam torque : %ld raw\n", (long)signal.bamocar_torque_raw);
    cliPrintf("bam dc bus : %ld mV\n", (long)signal.bamocar_dc_bus_mv);
    cliPrintf("bam m temp : %d.%d C\n",
              (int)(signal.bamocar_motor_temp_decic / 10),
              (int)abs(signal.bamocar_motor_temp_decic % 10));
    cliPrintf("bam i temp : %d.%d C\n",
              (int)(signal.bamocar_igbt_temp_decic / 10),
              (int)abs(signal.bamocar_igbt_temp_decic % 10));
    cliPrintf("bam status : 0x%08lX\n",
              (unsigned long)signal.bamocar_status);
    cliPrintf("bam error  : 0x%08lX\n",
              (unsigned long)signal.bamocar_error_warning);
    handled = true;
  }

  /*
   * 'can filter all' 로 열어 둔 필터를 디코더가 쓰는 세 ID 로 되돌린다.
   * 재부팅해도 같은 상태가 되지만, 버스를 들여다보다 바로 정상 동작으로
   * 돌아오고 싶을 때 전원을 껐다 켤 이유는 없다.
   */
  if ((args->argc == 1) && args->isStr(0, "filter"))
  {
    bool ok = telemetryConfigCanFilter(telemetry.signal.can_ch);

    cliPrintf("filter restore : %s\n", ok ? "ok" : "fail");
    cliPrintf("  bank 0 : 0x%03X orion\n",
              (unsigned int)TELEMETRY_ORION_CAN_ID);
    cliPrintf("  bank 1 : 0x%03X orion cell\n",
              (unsigned int)TELEMETRY_ORION_CELL_CAN_ID);
    cliPrintf("  bank 2 : 0x%03X bamocar\n",
              (unsigned int)BAMOCAR_CAN_DEFAULT_TX_ID);
    handled = true;
  }

  if (handled != true)
  {
    cliPrintf("telemetry info\n");
    cliPrintf("telemetry signal\n");
    cliPrintf("telemetry filter\n");
  }
}
#endif
