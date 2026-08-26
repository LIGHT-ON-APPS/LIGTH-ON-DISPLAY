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
 * 레지스터 번호와 이름은 NDrive 설치본의 settings/regid.txt 를 정본으로 삼는다.
 *
 * 0xA8 을 '실토크' 로 알고 쓰던 것을 바로잡았다. regid.txt 상 0xA8 은
 * REGID_SPEED_ACTUAL_FILT, 즉 드라이브가 필터를 먹인 실회전수다. 실토크는
 * 0xA0(REGID_TORQUE_OUT) 이고 아무도 요청하지 않으므로 지금은 받지 않는다.
 */
#define TELEMETRY_BAMOCAR_REG_N_ACTUAL        0x30U
#define TELEMETRY_BAMOCAR_REG_N_ACTUAL_FILT   0xA8U
#define TELEMETRY_BAMOCAR_REG_NMAX            0xCEU
#define TELEMETRY_BAMOCAR_REG_TORQUE_OUT      0xA0U
#define TELEMETRY_BAMOCAR_REG_MOTOR_TEMP      0x49U

/*
 * 회전수 풀스케일.
 *
 * 0x30/0xA8 은 ±32767 로 정규화돼 오고, 그 32767 에 대응하는 값은 모터
 * 사양이 아니라 드라이브에 설정된 NMAX 다 — NDrive 의 settings/units.txt 가
 * "speedRPMmax updates from 0xCE" 라고 못박고 있다. 그래서 0xCE 를 받으면
 * 그 값을 쓰고, 못 받았을 때만 아래 기본값으로 환산한다.
 *
 * 기본값 6500 은 EMRAX 228 의 limiting speed(데이터시트 v1.6)다. 드라이브의
 * NMAX 가 이와 다르게 잡혀 있으면 0xCE 가 오기 전까지 표시 회전수는 그 비율
 * 만큼 어긋난다. 0xCE 는 아무도 요청하지 않으므로, 정확히 하려면 APPS 의
 * 요청 목록에 0xCE 를 한 줄 넣으면 된다(한 번만 와도 그 뒤로 계속 맞는다).
 *
 * 참고로 이전 계기판 펌웨어는 여기에 6000 을 썼다. 어느 쪽도 드라이브에서
 * 확인한 값이 아니라 6500 을 그대로 둔다 — 둘 중 무엇이 맞든 표시 회전수만
 * 8 % 어긋나고, 0xCE 를 한 번 받으면 그 순간 정확해진다.
 */
#define TELEMETRY_BAMOCAR_N_MAX_RPM_DEFAULT   6500L

/*
 * BMS 요약 프레임.
 *
 * 이 차의 BMS 가 실제로 무엇을 어디에 싣는지는 F411 이전에 쓰던 계기판
 * 펌웨어(KSAE_L432KC_Display_ver2.6 의 main.c)가 정본이다. 거기서 쓰던
 * 두 ID 를 그대로 가져왔다. 처음 이 파일에 적혀 있던 0x6B0 은 버스에
 * 존재하지 않는다 — 2026-08-26 에 'telemetry scan' 으로 확인했다.
 *
 *   CAN ID 0x446, standard, 8 bytes, big-endian
 *     [0..1] Power            u16, 눈금 미확인
 *     [2..3] Min cell voltage u16, 0.1 mV/bit
 *     [4]    High temperature i8,  1 degC/bit
 *     [5]    Low temperature  i8,  1 degC/bit
 *     [6]    Pack SOC         u8,  0.5 %/bit
 *
 *   CAN ID 0x447, standard, big-endian
 *     [0..1] Pack voltage     u16, 0.1 V/bit
 *     [2..3] Pack current     i16, 0.1 A/bit
 *
 * 실측 대조 (LV 만 켠 상태, 0x446 = 00 00 9B BA 18 16 C7 4A):
 * 최소 셀 3.9866 V 는 같은 시각 0x036 셀 브로드캐스트가 준 값과 일치했고,
 * 온도 24/22 도와 SOC 99.5 % 도 그대로 말이 된다.
 *
 * 전류의 부호는 아직 못 봤다(LV 만이라 0 이었다). 화면은 전류를 쓰지 않으니
 * 지금 틀려도 표시에는 영향이 없지만, 나중에 쓸 일이 생기면 충전/방전 방향을
 * 먼저 확인할 것.
 */
#define TELEMETRY_ORION_CAN_ID                0x446U
#define TELEMETRY_ORION_PACK_CAN_ID           0x447U
#define TELEMETRY_ORION_VOLT_MV_PER_BIT       100L
#define TELEMETRY_ORION_CURR_MA_PER_BIT       100L
#define TELEMETRY_ORION_SOC_PERMILLE_PER_BIT  5L
#define TELEMETRY_ORION_TEMP_DECIC_PER_BIT    10L
#define TELEMETRY_ORION_CELL_MV_PER_10_BIT    10U

/* Orion's optional battery-cell broadcast sends one cell per frame. The BMS
 * profile must enable this broadcast on the vehicle CAN interface and use the
 * same standard identifier. Instant voltage is big-endian and scaled at
 * 0.1 mV/bit. */
#define TELEMETRY_ORION_CELL_CAN_ID            0x36U
#define TELEMETRY_ORION_CELL_VOLT_MAX_DECIMV   50000U
#define TELEMETRY_ORION_CELL_TIMEOUT_MS         3000U

/*
 * 소스가 조용해지면 그 소스가 만든 신호를 무효로 돌린다.
 *
 * 이게 없으면 valid_mask 는 한 번 선 비트가 영원히 남는다. 화면 전체의
 * 신선도(telemetry_fresh)는 어느 ID 든 하나만 살아 있으면 유지되므로,
 * 셀 브로드캐스트가 초당 250 프레임씩 오는 동안 인버터가 조용해져도
 * 모터/IGBT 온도는 마지막 값을 계속 붙들고 있게 된다. 실제로는 값을
 * 모르는 상태인데 화면은 정상값을 보여주는 셈이라, 온도처럼 오르는 쪽이
 * 위험한 신호에서는 그대로 두면 안 된다.
 *
 * 가장 느린 것이 0x446/0x447(20 Hz 안팎)이라 1 초면 여유가 충분하다.
 */
#define TELEMETRY_SRC_TIMEOUT_MS                1000U

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
  TELEMETRY_SRC_ORION_PACK,
  TELEMETRY_SRC_ORION_CELL,
  TELEMETRY_SRC_BAMOCAR,
  TELEMETRY_SRC_MAX
} telemetry_src_t;

typedef struct
{
  uint32_t frame_cnt;
  uint32_t last_ms;
} telemetry_src_stat_t;

/*
 * Bamocar 응답을 레지스터 단위로 세기 위한 슬롯 수.
 *
 * 아래 bamocar_track_reg[] 의 항목 수 + 1(목록에 없는 레지스터를 모으는 칸).
 * 표와 개수가 어긋나면 telemetryBamocarTrackReg() 가 남는 칸을 못 쓰거나
 * 배열 밖을 짚으므로, 표를 고칠 때 이 값을 같이 고칠 것.
 */
#define TELEMETRY_BAMOCAR_TRACK_MAX           10U

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

  /* 0x181 응답의 레지스터별 수신 횟수. bamocar_track_reg[] 와 같은 순서다. */
  uint16_t                    bamocar_reg_cnt[TELEMETRY_BAMOCAR_TRACK_MAX];
} telemetry_t;

static telemetry_t telemetry;

#define TELEMETRY_ARRAY_COUNT(a)              (sizeof(a) / sizeof((a)[0]))

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

/*
 * 모터 온도 대조표 (KTY81/210).
 *
 * EMRAX 228 의 고정자 권선 센서는 KTY 81/210 이고(228 데이터시트 v1.6,
 * "Temperature sensor on the stator windings"), Bamocar 는 0x49 에 그 입력의
 * 원시 Num 을 실어 보낸다. 아래 값은 UniTek 이 NDrive 와 함께 배포하는
 * "BAMOCAR - Temperature and VdcBus Tables" 4/4 쪽의 KTY81-2xx 열을 그대로
 * 옮긴 것이다. 같은 표의 Tigbt NTC-infin 열이 위 IGBT 표와 한 값도 틀리지 않고
 * 맞으므로, 두 표는 같은 출처의 같은 눈금이다.
 *
 * 위로 갈수록 5 도당 Num 간격이 좁아진다(25 도 부근 285, 150 도 부근 68).
 * 센서 자체의 특성이라 여기서 할 수 있는 일은 없고, 고온에서 표시가 한두 도씩
 * 튀는 것은 정상이다.
 */
static const telemetry_temp_point_t bamocar_motor_temp_table[] =
{
  { 7414U,  -350}, { 7687U,  -300}, { 7962U,  -250}, { 8240U,  -200},
  { 8520U,  -150}, { 8802U,  -100}, { 9085U,   -50}, { 9369U,     0},
  { 9654U,    50}, { 9939U,   100}, {10225U,   150}, {10510U,   200},
  {10795U,   250}, {11080U,   300}, {11364U,   350}, {11646U,   400},
  {11927U,   450}, {12207U,   500}, {12485U,   550}, {12762U,   600},
  {13036U,   650}, {13308U,   700}, {13578U,   750}, {13846U,   800},
  {14111U,   850}, {14373U,   900}, {14633U,   950}, {14890U,  1000},
  {15144U,  1050}, {15391U,  1100}, {15628U,  1150}, {15852U,  1200},
  {16061U,  1250}, {16251U,  1300}, {16421U,  1350}, {16569U,  1400},
  {16692U,  1450}, {16789U,  1500}, {16857U,  1550},
};

/*
 * 수신 횟수를 따로 세는 Bamocar 레지스터.
 *
 * 0x181 프레임 총수만으로는 "무엇이 오고 무엇이 안 오는지" 가 갈리지 않는다.
 * 요청 목록을 들고 있는 것은 APPS 이고 이 보드는 응답을 엿듣기만 하므로,
 * 화면에서 한 칸만 비어 있을 때 그 레지스터가 버스에 아예 없는 것인지
 * 우리 디코딩이 틀린 것인지 구분할 방법이 필요하다. 앞의 것부터 화면이
 * 쓰는 순서로 두고, 목록에 없는 레지스터는 마지막 칸에 모아 센다.
 */
static const uint8_t bamocar_track_reg[TELEMETRY_BAMOCAR_TRACK_MAX - 1U] =
{
  TELEMETRY_BAMOCAR_REG_N_ACTUAL,
  TELEMETRY_BAMOCAR_REG_MOTOR_TEMP,
  TELEMETRY_BAMOCAR_REG_IGBT_TEMP,
  TELEMETRY_BAMOCAR_REG_DC_BUS,
  BAMOCAR_REG_STATUS,
  TELEMETRY_BAMOCAR_REG_ERROR_WARNING,
  TELEMETRY_BAMOCAR_REG_N_ACTUAL_FILT,
  TELEMETRY_BAMOCAR_REG_TORQUE_OUT,
  TELEMETRY_BAMOCAR_REG_NMAX,
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

static int16_t telemetryTempRawToDeciC(uint16_t raw,
                                       const telemetry_temp_point_t *p_table,
                                       uint32_t count)
{

  if (raw <= p_table[0].raw)
  {
    return p_table[0].temp_decic;
  }
  if (raw >= p_table[count - 1U].raw)
  {
    return p_table[count - 1U].temp_decic;
  }

  for (uint32_t i = 1U; i < count; i++)
  {
    if (raw <= p_table[i].raw)
    {
      int32_t raw0 = p_table[i - 1U].raw;
      int32_t raw1 = p_table[i].raw;
      int32_t temp0 = p_table[i - 1U].temp_decic;
      int32_t temp1 = p_table[i].temp_decic;

      return (int16_t)(temp0 +
          (((int32_t)raw - raw0) * (temp1 - temp0)) / (raw1 - raw0));
    }
  }

  return 0;
}

/*
 * 정규화된 회전수(±32767)를 rpm 으로. 분모는 0xCE 를 받았으면 그 값이다.
 *
 * 부호는 버리고 크기만 쓴다. 이 차는 전진할 때 raw 가 음수로 오는데(이전
 * 계기판 펌웨어가 화면에 그릴 때 `-hv_rpm` 으로 뒤집어 쓰고 있었다),
 * 화면은 회전수를 0 아래로 클램프하므로 그대로 두면 주행 중에도 0 이
 * 찍힌다. 계기판에 필요한 것은 방향이 아니라 얼마나 도는지이고, 회생
 * 제동 중에도 바퀴는 같은 방향으로 돌기 때문에 절댓값이 맞다. 방향까지
 * 봐야 할 일이 생기면 여기가 아니라 별도 신호로 빼는 게 낫다.
 */
static int32_t telemetryBamocarSpeedToRpm(uint16_t raw)
{
  int32_t nmax = (telemetry.signal.bamocar_nmax_rpm != 0U)
                     ? (int32_t)telemetry.signal.bamocar_nmax_rpm
                     : TELEMETRY_BAMOCAR_N_MAX_RPM_DEFAULT;
  int32_t rpm = ((int32_t)(int16_t)raw * nmax) / 32767L;

  return (rpm < 0) ? -rpm : rpm;
}

/* 0x446 — 전력, 최소 셀 전압, 최고/최저 온도, SOC. */
static bool telemetryDecodeOrion(const can_msg_t *p_msg)
{
  if ((p_msg->id_type != CAN_STD) ||
      (p_msg->id != TELEMETRY_ORION_CAN_ID) ||
      (p_msg->length < 7U))
  {
    return false;
  }

  telemetry.signal.bms_power_raw = telemetryReadU16Be(&p_msg->data[0]);
  telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BMS_POWER;

  /*
   * 이 프레임도 최소 셀 전압을 싣지만 화면에는 쓰지 않는다. 0x036 셀
   * 브로드캐스트 쪽 스캔이 이미 그 값을 만들고 있고, 거기에는 표시 중인
   * 값보다 낮은 셀을 보면 스캔 완성을 기다리지 않고 즉시 내리는 안전
   * 규칙이 붙어 있다. 두 곳에서 같은 필드를 쓰면 그 규칙과 타임아웃이
   * 서로 엉킨다. 교차 확인이 필요하면 CLI 로 두 값을 나란히 본다.
   */

  telemetry.signal.bms_max_temp_decic =
      (int16_t)((int32_t)(int8_t)p_msg->data[4] *
                TELEMETRY_ORION_TEMP_DECIC_PER_BIT);
  telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BMS_MAX_TEMP_DECIC;

  telemetry.signal.bms_min_temp_decic =
      (int16_t)((int32_t)(int8_t)p_msg->data[5] *
                TELEMETRY_ORION_TEMP_DECIC_PER_BIT);
  telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BMS_MIN_TEMP_DECIC;

  telemetry.signal.bms_soc_permille =
      (uint16_t)((uint32_t)p_msg->data[6] *
                 (uint32_t)TELEMETRY_ORION_SOC_PERMILLE_PER_BIT);
  telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BMS_SOC_PERMILLE;

  telemetryMarkSource(TELEMETRY_SRC_ORION, p_msg->timestamp);
  telemetry.signal.last_update_ms = p_msg->timestamp;
  return true;
}

/* 0x447 — 팩 전압과 전류. */
static bool telemetryDecodeOrionPack(const can_msg_t *p_msg)
{
  uint16_t raw;

  if ((p_msg->id_type != CAN_STD) ||
      (p_msg->id != TELEMETRY_ORION_PACK_CAN_ID) ||
      (p_msg->length < 4U))
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

  telemetryMarkSource(TELEMETRY_SRC_ORION_PACK, p_msg->timestamp);
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

/*
 * 레지스터별 수신 횟수를 올린다. 표에 없는 레지스터는 마지막 칸으로 간다.
 *
 * 수신 인터럽트 안에서 도는 코드다. 9 칸 선형 탐색은 500 kbps 의 프레임
 * 간격(최소 200 us)에 비하면 없는 것과 같다. 카운터는 넘쳐도 진단이
 * "오고 있다/안 온다" 를 가리는 데는 지장이 없으므로 포화시키지 않는다.
 */
static void telemetryBamocarTrackReg(uint8_t reg_id)
{
  uint32_t slot = TELEMETRY_BAMOCAR_TRACK_MAX - 1U;

  for (uint32_t i = 0U; i < (TELEMETRY_BAMOCAR_TRACK_MAX - 1U); i++)
  {
    if (bamocar_track_reg[i] == reg_id)
    {
      slot = i;
      break;
    }
  }

  telemetry.bamocar_reg_cnt[slot]++;
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
  telemetryBamocarTrackReg(reg_id);

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
          telemetryTempRawToDeciC(raw16, bamocar_igbt_temp_table,
                                  TELEMETRY_ARRAY_COUNT(bamocar_igbt_temp_table));
      telemetry.signal.valid_mask |=
          TELEMETRY_SIGNAL_BAMOCAR_IGBT_TEMP_DECIC;
      updated = true;
      break;

    case TELEMETRY_BAMOCAR_REG_N_ACTUAL:
      raw16 = telemetryReadU16Le(&p_msg->data[1]);
      telemetry.signal.bamocar_rpm = telemetryBamocarSpeedToRpm(raw16);
      telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BAMOCAR_RPM;
      updated = true;
      break;

    case TELEMETRY_BAMOCAR_REG_TORQUE_OUT:
      /* 실토크. 32767 이 무엇에 대응하는지(장치 전류 기준)를 아직 확인하지
       * 못해 환산하지 않고 원시값으로 둔다. 계기판에는 쓰지 않는다. */
      raw16 = telemetryReadU16Le(&p_msg->data[1]);
      telemetry.signal.bamocar_torque_raw = (int32_t)(int16_t)raw16;
      telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BAMOCAR_TORQUE_RAW;
      updated = true;
      break;

    case TELEMETRY_BAMOCAR_REG_NMAX:
      /* 부호 없는 16bit(regid.txt). 한 번만 와도 그 뒤 환산이 계속 맞는다. */
      telemetry.signal.bamocar_nmax_rpm = telemetryReadU16Le(&p_msg->data[1]);
      updated = true;
      break;

    case TELEMETRY_BAMOCAR_REG_N_ACTUAL_FILT:
      raw16 = telemetryReadU16Le(&p_msg->data[1]);
      telemetry.signal.bamocar_speed_filt_rpm =
          telemetryBamocarSpeedToRpm(raw16);
      telemetry.signal.valid_mask |= TELEMETRY_SIGNAL_BAMOCAR_SPEED_FILT_RPM;
      updated = true;
      break;

    case TELEMETRY_BAMOCAR_REG_MOTOR_TEMP:
      raw16 = telemetryReadU16Le(&p_msg->data[1]);
      /*
       * 모터는 IGBT 의 NTC 가 아니라 KTY81/210 이라 표가 다르다.
       * regid.txt 상 0x49(REGID_T_MOTOR)는 부호 없는 16bit 이므로
       * int16 으로 접지 않는다 — 접으면 32767 을 넘는 Num 이 음수가 된다.
       */
      telemetry.signal.bamocar_motor_temp_decic =
          telemetryTempRawToDeciC(raw16, bamocar_motor_temp_table,
                                  TELEMETRY_ARRAY_COUNT(bamocar_motor_temp_table));
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
  (void)telemetryDecodeOrionPack(p_msg);
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
   * 우리가 실제로 읽는 것은 아래 넷뿐이므로 그 둘을 덮어쓰고 둘을 더 건다.
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

  if (canConfigFilter(ch, 3, CAN_STD, CAN_ID_MASK,
                      TELEMETRY_ORION_PACK_CAN_ID, 0x7FFU) != true)
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

/*
 * 소스별로 그 소스만이 세우는 신호 비트. 순서는 telemetry_src_t 와 같다.
 *
 * 셀 브로드캐스트(0x036)는 0 이다 — 최소 셀 전압에는 스캔 완성 규칙에 묶인
 * 자체 타임아웃이 이미 있고, 그 둘을 겹쳐 걸면 서로 지운다.
 */
static const uint32_t telemetry_src_signal_mask[TELEMETRY_SRC_MAX] =
{
  TELEMETRY_SIGNAL_BMS_SOC_PERMILLE |
      TELEMETRY_SIGNAL_BMS_MAX_TEMP_DECIC |
      TELEMETRY_SIGNAL_BMS_MIN_TEMP_DECIC |
      TELEMETRY_SIGNAL_BMS_POWER,

  TELEMETRY_SIGNAL_BMS_PACK_MV |
      TELEMETRY_SIGNAL_BMS_PACK_MA,

  0U,

  TELEMETRY_SIGNAL_BAMOCAR_RPM |
      TELEMETRY_SIGNAL_BAMOCAR_SPEED_FILT_RPM |
      TELEMETRY_SIGNAL_BAMOCAR_DC_BUS_MV |
      TELEMETRY_SIGNAL_BAMOCAR_MOTOR_TEMP_DECIC |
      TELEMETRY_SIGNAL_BAMOCAR_IGBT_TEMP_DECIC |
      TELEMETRY_SIGNAL_BAMOCAR_STATUS |
      TELEMETRY_SIGNAL_BAMOCAR_ERROR_WARNING |
      TELEMETRY_SIGNAL_BAMOCAR_TORQUE_RAW,
};

/* 조용해진 소스의 신호를 무효로 돌린다. 호출자가 임계구역을 잡는다. */
static void telemetryExpireStaleSources(uint32_t now_ms)
{
  for (uint32_t i = 0U; i < (uint32_t)TELEMETRY_SRC_MAX; i++)
  {
    if ((telemetry_src_signal_mask[i] == 0U) ||
        (telemetry.src[i].frame_cnt == 0U))
    {
      continue;
    }

    if ((now_ms - telemetry.src[i].last_ms) >= TELEMETRY_SRC_TIMEOUT_MS)
    {
      telemetry.signal.valid_mask &= ~telemetry_src_signal_mask[i];
    }
  }
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

  telemetryExpireStaleSources(millis());

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
      "bms", "pack", "cell", "bamocar"
    };
    static const uint32_t src_id[TELEMETRY_SRC_MAX] =
    {
      TELEMETRY_ORION_CAN_ID,
      TELEMETRY_ORION_PACK_CAN_ID,
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

    /* 셀 스캔이 학습한 팩의 셀 수. 팩 전압을 셀 평균으로 나눠 맞춰 볼 때
     * 쓴다 — 어느 바이트가 팩 전압인지 역산하는 데 이만한 단서가 없다. */
    if (telemetry.cell_expected_last_id_valid == true)
    {
      cliPrintf("cells      : %u\n",
                (unsigned int)telemetry.cell_expected_last_id + 1U);
    }
    else
    {
      cliPrintf("cells      : -- (scan not complete yet)\n");
    }

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
        /*
         * now_ms 는 출력 전에 한 번 찍는데, 이 목록을 UART 로 밀어내는
         * 동안에도 프레임은 계속 들어와 last_ms 를 미래로 밀어 놓는다.
         * 그대로 빼면 언더플로우해서 4294967xxx ms 로 보인다. 방금 온
         * 것이니 0 으로 적는다.
         */
        uint32_t age_ms = now_ms - telemetry.src[i].last_ms;

        if ((int32_t)age_ms < 0)
        {
          age_ms = 0U;
        }

        cliPrintf(", %lu ms ago\n", (unsigned long)age_ms);
      }
      else
      {
        cliPrintf(", never\n");
      }
    }

    /*
     * 0x181 이 오는데 화면 한 칸만 비어 있는 경우를 가른다. 아래에서 그
     * 레지스터가 0 이면 인버터가 그 값을 아예 안 보내는 것이고(= 요청
     * 목록을 들고 있는 APPS 쪽 문제), 0 이 아닌데 화면이 비어 있으면
     * 디코딩이나 화면 쪽 문제다.
     */
    if (telemetry.src[TELEMETRY_SRC_BAMOCAR].frame_cnt > 0U)
    {
      static const char *reg_name[TELEMETRY_BAMOCAR_TRACK_MAX] =
      {
        "n_act", "t_motor", "t_igbt", "v_dcbus", "status",
        "err/warn", "n_filt", "torque", "nmax", "other",
      };

      for (uint32_t i = 0U; i < TELEMETRY_BAMOCAR_TRACK_MAX; i++)
      {
        if (i < (TELEMETRY_BAMOCAR_TRACK_MAX - 1U))
        {
          cliPrintf("  0x%02X %-8s: %u\n",
                    (unsigned int)bamocar_track_reg[i],
                    reg_name[i],
                    (unsigned int)telemetry.bamocar_reg_cnt[i]);
        }
        else
        {
          cliPrintf("  ---- %-8s: %u\n",
                    reg_name[i],
                    (unsigned int)telemetry.bamocar_reg_cnt[i]);
        }
      }
    }

    if (telemetry.src[TELEMETRY_SRC_BAMOCAR].frame_cnt == 0U)
    {
      cliPrintf("hint       : no 0x181 yet. this board only listens;\n");
      cliPrintf("             the inverter stays silent until someone\n");
      cliPrintf("             sends the 0x201 repetition request.\n");
    }

    /*
     * 셀 브로드캐스트는 오는데 요약 프레임이 안 오는 경우. BMS 도 버스도
     * 살아 있으니 남는 것은 그 메시지가 프로파일에서 빠졌을 가능성이다.
     */
    if ((telemetry.src[TELEMETRY_SRC_ORION_CELL].frame_cnt > 0U) &&
        ((telemetry.src[TELEMETRY_SRC_ORION].frame_cnt == 0U) ||
         (telemetry.src[TELEMETRY_SRC_ORION_PACK].frame_cnt == 0U)))
    {
      cliPrintf("hint       : 0x036 arrives but 0x446/0x447 does not.\n");
      cliPrintf("             those are configured broadcasts in the\n");
      cliPrintf("             BMS profile - check they are enabled.\n");
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
    cliPrintf("bms temp   : %d.%d C (min %d.%d C)\n",
              (int)(signal.bms_max_temp_decic / 10),
              (int)abs(signal.bms_max_temp_decic % 10),
              (int)(signal.bms_min_temp_decic / 10),
              (int)abs(signal.bms_min_temp_decic % 10));
    cliPrintf("bms power  : %u raw\n", (unsigned int)signal.bms_power_raw);
    cliPrintf("bms min V  : %u.%03u V\n",
              (unsigned int)(signal.bms_min_cell_mv / 1000U),
              (unsigned int)(signal.bms_min_cell_mv % 1000U));
    cliPrintf("bam rpm    : %ld rpm\n", (long)signal.bamocar_rpm);
    cliPrintf("bam torque : %ld raw\n", (long)signal.bamocar_torque_raw);
    /* 0xA8 은 APPS 요청 목록에서 뺐다(0x30 과 같은 회전수라 주기 슬롯만
     * 먹었다). 디코더는 남겨 두되, 안 온 것을 0 rpm 으로 보여 주면 정지한
     * 것처럼 읽히므로 수신 여부를 같이 찍는다. */
    if ((signal.valid_mask & TELEMETRY_SIGNAL_BAMOCAR_SPEED_FILT_RPM) != 0U)
    {
      cliPrintf("bam n filt : %ld rpm\n",
                (long)signal.bamocar_speed_filt_rpm);
    }
    else
    {
      cliPrintf("bam n filt : -- (0xA8 미요청)\n");
    }
    cliPrintf("bam nmax   : %u rpm%s\n",
              (unsigned int)signal.bamocar_nmax_rpm,
              (signal.bamocar_nmax_rpm != 0U) ? "" : " (미수신, 기본 6500 적용)");
    cliPrintf("bam dc bus : %ld mV\n", (long)signal.bamocar_dc_bus_mv);
    cliPrintf("bam m temp : %d.%d C\n",
              (int)(signal.bamocar_motor_temp_decic / 10),
              (int)abs(signal.bamocar_motor_temp_decic % 10));
    cliPrintf("bam i temp : %d.%d C\n",
              (int)(signal.bamocar_igbt_temp_decic / 10),
              (int)abs(signal.bamocar_igbt_temp_decic % 10));
    cliPrintf("bam status : 0x%08lX\n",
              (unsigned long)signal.bamocar_status);
    /*
     * 0x8F(REGID_LOGIC_ERR_BITMAP)는 regid.txt 상 isHighLow=1 이다. 32 비트
     * 하나가 아니라 16 비트 두 벌이고, 아래쪽이 에러, 위쪽이 경고다. 통으로
     * 찍으면 둘이 구분되지 않아 나눠 적는다.
     */
    cliPrintf("bam error  : 0x%08lX (err 0x%04X, warn 0x%04X)\n",
              (unsigned long)signal.bamocar_error_warning,
              (unsigned int)(signal.bamocar_error_warning & 0xFFFFU),
              (unsigned int)((signal.bamocar_error_warning >> 16) & 0xFFFFU));
    handled = true;
  }

  /*
   * 버스에 실제로 흐르는 ID 를 모아 센다.
   *
   * 'can read_test' 는 프레임을 오는 족족 UART 로 찍는데, 셀 브로드캐스트만
   * 초당 200 프레임을 넘겨 115200 bps 로는 감당이 안 된다. 화면이 0x36 으로
   * 도배되고 정작 찾는 프레임은 밀린다. 그래서 여기서는 찍지 않고 세기만
   * 하다가 끝에 한 번 요약한다.
   *
   * 마지막 프레임의 바이트를 같이 남긴다. ID 만 알아서는 디코더를 못 짜고,
   * 어느 바이트에 무엇이 실려 오는지 봐야 하기 때문이다.
   */
  if ((args->argc == 1) && args->isStr(0, "scan"))
  {
    enum { SCAN_ID_MAX = 24, SCAN_MS = 3000U };

    struct
    {
      uint32_t id;
      uint32_t cnt;
      uint8_t  ext;
      uint8_t  length;
      uint8_t  data[8];
    } seen[SCAN_ID_MAX];

    uint32_t  seen_cnt = 0U;
    uint32_t  missed_id = 0U;
    uint32_t  total = 0U;
    uint32_t  start_ms;
    can_msg_t msg;
    uint8_t   ch = telemetry.signal.can_ch;

    if (canIsOpen(ch) != true)
    {
      cliPrintf("can ch %d is not open\n", ch);
      return;
    }

    /* 전체통과로 열어야 디코더가 안 보는 ID 까지 잡힌다. */
    (void)canConfigFilter(ch, 0, CAN_STD, CAN_ID_MASK, 0x000, 0x000);
    (void)canConfigFilter(ch, 1, CAN_EXT, CAN_ID_MASK, 0x000, 0x000);

    /* 필터를 넓히기 전에 쌓여 있던 것은 표본에서 뺀다. */
    while (canMsgAvailable(ch) > 0U)
    {
      (void)canMsgRead(ch, &msg);
    }

    cliPrintf("scanning %lu ms ...\n", (unsigned long)SCAN_MS);

    start_ms = millis();
    while ((millis() - start_ms) < SCAN_MS)
    {
      uint32_t i;

      if (canMsgAvailable(ch) == 0U)
      {
        continue;
      }

      (void)canMsgRead(ch, &msg);
      total++;

      for (i = 0U; i < seen_cnt; i++)
      {
        if ((seen[i].id == msg.id) &&
            (seen[i].ext == (uint8_t)(msg.id_type == CAN_EXT)))
        {
          break;
        }
      }

      if (i == seen_cnt)
      {
        if (seen_cnt >= (uint32_t)SCAN_ID_MAX)
        {
          missed_id++;
          continue;
        }
        seen[i].id  = msg.id;
        seen[i].ext = (uint8_t)(msg.id_type == CAN_EXT);
        seen[i].cnt = 0U;
        seen_cnt++;
      }

      seen[i].cnt++;
      seen[i].length = (msg.length > 8U) ? 8U : msg.length;
      for (uint8_t b = 0U; b < seen[i].length; b++)
      {
        seen[i].data[b] = msg.data[b];
      }
    }

    /* 디코더가 보는 세 ID 로 되돌린다. 안 되돌리면 부팅 전까지 열린 채다. */
    (void)telemetryConfigCanFilter(ch);

    cliPrintf("%lu frames, %lu ids\n",
              (unsigned long)total, (unsigned long)seen_cnt);

    for (uint32_t i = 0U; i < seen_cnt; i++)
    {
      cliPrintf("  0x%03X %s L%d x%5lu :",
                (unsigned int)seen[i].id,
                (seen[i].ext != 0U) ? "ext" : "std",
                (int)seen[i].length,
                (unsigned long)seen[i].cnt);

      for (uint8_t b = 0U; b < seen[i].length; b++)
      {
        cliPrintf(" %02X", (unsigned int)seen[i].data[b]);
      }
      cliPrintf("\n");
    }

    if (missed_id > 0U)
    {
      cliPrintf("  (%lu frames of further ids not listed)\n",
                (unsigned long)missed_id);
    }

    cliPrintf("filter restored to the decoded ids\n");
    handled = true;
  }

  /*
   * ID 하나를 지목해 바이트가 바뀔 때만 찍는다.
   *
   * 어느 바이트에 무엇이 실려 오는지 알아내는 용도다. 매 프레임을 다 찍으면
   * 같은 줄이 초당 수십 개씩 흘러가 눈으로 못 쫓는데, 바뀐 것만 남기면
   * 움직이는 바이트(전압/전류/SOC)와 고정 바이트가 바로 갈린다. 바뀐
   * 바이트 앞에는 '*' 를 붙인다.
   *
   * 아무 키나 누르면 끝나고 필터는 되돌아간다.
   */
  if ((args->argc == 2) && args->isStr(0, "watch"))
  {
    uint32_t    want_id = (uint32_t)args->getData(1);
    CanIdType_t id_type = (want_id > 0x7FFU) ? CAN_EXT : CAN_STD;
    uint32_t    id_mask = (id_type == CAN_EXT) ? 0x1FFFFFFFU : 0x7FFU;
    uint8_t     ch = telemetry.signal.can_ch;
    can_msg_t   msg;
    uint8_t     prev[8];
    uint8_t     prev_len = 0U;
    bool        have_prev = false;
    uint32_t    seen = 0U;

    if (canIsOpen(ch) != true)
    {
      cliPrintf("can ch %d is not open\n", ch);
      return;
    }

    /*
     * 필터 뱅크를 모두 이 ID 로 덮는다. 뱅크를 끄는 길이 없으므로, 남겨 두면
     * 디코더가 쓰던 ID 가 같이 들어와 화면이 섞인다.
     */
    for (uint8_t bank = 0U; bank < 4U; bank++)
    {
      (void)canConfigFilter(ch, bank, id_type, CAN_ID_MASK, want_id, id_mask);
    }

    while (canMsgAvailable(ch) > 0U)
    {
      (void)canMsgRead(ch, &msg);
    }

    cliPrintf("watching 0x%03X (%s). press any key to stop.\n",
              (unsigned int)want_id, (id_type == CAN_EXT) ? "ext" : "std");

    while (cliKeepLoop())
    {
      uint8_t len;
      bool    changed;

      if (canMsgAvailable(ch) == 0U)
      {
        continue;
      }

      (void)canMsgRead(ch, &msg);

      if (msg.id != want_id)
      {
        continue;
      }

      seen++;
      len = (msg.length > 8U) ? 8U : msg.length;
      changed = (have_prev != true) || (prev_len != len);

      for (uint8_t b = 0U; (b < len) && (changed != true); b++)
      {
        if (prev[b] != msg.data[b])
        {
          changed = true;
        }
      }

      if (changed != true)
      {
        continue;
      }

      cliPrintf("%5lu L%d :", (unsigned long)seen, (int)len);

      for (uint8_t b = 0U; b < len; b++)
      {
        bool diff = (have_prev == true) && (prev_len == len) &&
                    (prev[b] != msg.data[b]);

        cliPrintf("%c%02X", diff ? '*' : ' ', (unsigned int)msg.data[b]);
        prev[b] = msg.data[b];
      }
      cliPrintf("\n");

      prev_len = len;
      have_prev = true;
    }

    (void)telemetryConfigCanFilter(ch);
    cliPrintf("\n%lu frames seen. filter restored.\n", (unsigned long)seen);
    handled = true;
  }

  /*
   * 'can filter all' 로 열어 둔 필터를 디코더가 쓰는 ID 들로 되돌린다.
   * 재부팅해도 같은 상태가 되지만, 버스를 들여다보다 바로 정상 동작으로
   * 돌아오고 싶을 때 전원을 껐다 켤 이유는 없다.
   */
  if ((args->argc == 1) && args->isStr(0, "filter"))
  {
    bool ok = telemetryConfigCanFilter(telemetry.signal.can_ch);

    cliPrintf("filter restore : %s\n", ok ? "ok" : "fail");
    cliPrintf("  bank 0 : 0x%03X bms summary\n",
              (unsigned int)TELEMETRY_ORION_CAN_ID);
    cliPrintf("  bank 1 : 0x%03X bms cell\n",
              (unsigned int)TELEMETRY_ORION_CELL_CAN_ID);
    cliPrintf("  bank 2 : 0x%03X bamocar\n",
              (unsigned int)BAMOCAR_CAN_DEFAULT_TX_ID);
    cliPrintf("  bank 3 : 0x%03X bms pack v/a\n",
              (unsigned int)TELEMETRY_ORION_PACK_CAN_ID);
    handled = true;
  }

  if (handled != true)
  {
    cliPrintf("telemetry info\n");
    cliPrintf("telemetry signal\n");
    cliPrintf("telemetry scan\n");
    cliPrintf("telemetry watch <id>\n");
    cliPrintf("telemetry filter\n");
  }
}
#endif
