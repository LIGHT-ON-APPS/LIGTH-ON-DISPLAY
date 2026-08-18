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

  telemetry.signal.last_update_ms = p_msg->timestamp;
  return true;
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

bool telemetryInit(void)
{
  memset(&telemetry, 0, sizeof(telemetry));
  telemetry.initialized = true;
  telemetry.signal.can_ch = _DEF_CAN1;

#ifdef _USE_HW_CLI
  (void)cliAdd("telemetry", cliTelemetry);
#endif

  return true;
}

void telemetryUpdate(void)
{
  can_msg_t msg;
  uint32_t available;
  uint8_t can_ch;

  if (telemetry.initialized != true)
  {
    (void)telemetryInit();
  }

  can_ch = telemetry.signal.can_ch;
  if (canIsOpen(can_ch) != true)
  {
    if ((telemetry.signal.valid_mask &
         TELEMETRY_SIGNAL_BMS_MIN_CELL_MV) != 0U)
    {
      if ((millis() - telemetry.cell_min_last_update_ms) >=
          TELEMETRY_ORION_CELL_TIMEOUT_MS)
      {
        telemetry.signal.valid_mask &=
            ~TELEMETRY_SIGNAL_BMS_MIN_CELL_MV;
      }
    }
    return;
  }

  available = canMsgAvailable(can_ch);

  while (available > 0U)
  {
    if (canMsgRead(can_ch, &msg) == true)
    {
      telemetryDecodeCan(&msg);
    }

    available--;
  }

  if (((telemetry.signal.valid_mask &
        TELEMETRY_SIGNAL_BMS_MIN_CELL_MV) != 0U) &&
      ((millis() - telemetry.cell_min_last_update_ms) >=
       TELEMETRY_ORION_CELL_TIMEOUT_MS))
  {
    telemetry.signal.valid_mask &= ~TELEMETRY_SIGNAL_BMS_MIN_CELL_MV;
  }
}

bool telemetryGetSignalSnapshot(telemetry_signal_snapshot_t *p_snapshot)
{
  if (p_snapshot == NULL)
  {
    return false;
  }

  *p_snapshot = telemetry.signal;
  return p_snapshot->valid_mask != 0U;
}

#ifdef _USE_HW_CLI
static void cliTelemetry(cli_args_t *args)
{
  bool handled = false;

  if ((args->argc == 1) && args->isStr(0, "info"))
  {
    cliPrintf("decoder    : running\n");
    cliPrintf("can_ch     : %d\n", telemetry.signal.can_ch);
    cliPrintf("rx_frames  : %lu\n",
              (unsigned long)canGetRxCount(telemetry.signal.can_ch));
    cliPrintf("valid_mask : 0x%08lX\n",
              (unsigned long)telemetry.signal.valid_mask);
    cliPrintf("storage    : external (TMA-1/ESP)\n");
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

  if (handled != true)
  {
    cliPrintf("telemetry info\n");
    cliPrintf("telemetry signal\n");
  }
}
#endif
