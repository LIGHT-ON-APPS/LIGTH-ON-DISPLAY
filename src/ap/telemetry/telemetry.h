/*
 * telemetry.h
 *
 * Orion/Bamocar CAN decoder and vehicle-signal snapshot service.
 */

#ifndef SRC_AP_SW_TELEMETRY_TELEMETRY_H_
#define SRC_AP_SW_TELEMETRY_TELEMETRY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"

#define TELEMETRY_SIGNAL_BMS_PACK_MV              (1UL << 0)
#define TELEMETRY_SIGNAL_BMS_PACK_MA              (1UL << 1)
#define TELEMETRY_SIGNAL_BMS_SOC_PERMILLE         (1UL << 2)
#define TELEMETRY_SIGNAL_BMS_MAX_TEMP_DECIC       (1UL << 3)
#define TELEMETRY_SIGNAL_BMS_STATUS               (1UL << 4)
#define TELEMETRY_SIGNAL_BAMOCAR_RPM              (1UL << 5)
#define TELEMETRY_SIGNAL_BAMOCAR_SPEED_FILT_RPM   (1UL << 6)
#define TELEMETRY_SIGNAL_BAMOCAR_DC_BUS_MV        (1UL << 7)
#define TELEMETRY_SIGNAL_BAMOCAR_MOTOR_TEMP_DECIC (1UL << 8)
#define TELEMETRY_SIGNAL_BAMOCAR_IGBT_TEMP_DECIC  (1UL << 9)
#define TELEMETRY_SIGNAL_BAMOCAR_STATUS           (1UL << 10)
#define TELEMETRY_SIGNAL_BAMOCAR_ERROR_WARNING    (1UL << 11)
#define TELEMETRY_SIGNAL_BAMOCAR_TORQUE_RAW       (1UL << 12)
/* 13~15 는 DISPLAY_PROTOCOL_TLM_* 에 대응이 없는 자리다. 화면이 읽지 않는
 * 진단용 신호를 여기에 둔다 — valid_mask 는 변환 없이 그대로 넘어가므로
 * 화면이 쓰는 비트와 겹치면 안 된다. */
#define TELEMETRY_SIGNAL_BMS_MIN_TEMP_DECIC       (1UL << 13)
#define TELEMETRY_SIGNAL_BMS_POWER                (1UL << 14)
#define TELEMETRY_SIGNAL_BMS_MIN_CELL_MV           (1UL << 16)

typedef struct
{
  uint32_t valid_mask;
  uint32_t last_update_ms;
  uint8_t  can_ch;

  int32_t  bms_pack_mv;
  int32_t  bms_pack_ma;
  uint16_t bms_soc_permille;
  int16_t  bms_max_temp_decic;
  /* 0x446 이 최고와 함께 실어 보내는 최저 온도. 화면에는 안 쓰고 CLI 에서
   * 팩 안의 온도 편차를 볼 때만 쓴다. */
  int16_t  bms_min_temp_decic;
  /* 0x446 [0..1]. 눈금이 확인되지 않아 원시값 그대로 둔다. */
  uint16_t bms_power_raw;
  uint16_t bms_min_cell_mv;
  uint32_t bms_status;

  int32_t  bamocar_rpm;
  /* 0xA8(REGID_SPEED_ACTUAL_FILT). 0x30 과 같은 회전수인데 드라이브가
   * 필터를 먹인 값이라 덜 떤다. 눈금은 0x30 과 동일하다. */
  int32_t  bamocar_speed_filt_rpm;
  /* 0xCE(REGID_SPEED_RPMMAX_INT). 회전수 정규화의 분모다. 아직 못 받았으면
   * 0 이고, 그때는 기본값으로 환산한다. */
  uint16_t bamocar_nmax_rpm;
  /* 0xA0(REGID_TORQUE_OUT). 눈금이 확인되지 않아 원시값 그대로 둔다 —
   * 계기판에는 띄우지 않고 CLI 진단에만 쓴다. */
  int32_t  bamocar_torque_raw;
  int32_t  bamocar_dc_bus_mv;
  int16_t  bamocar_motor_temp_decic;
  int16_t  bamocar_igbt_temp_decic;
  uint32_t bamocar_status;
  uint32_t bamocar_error_warning;
} telemetry_signal_snapshot_t;

bool telemetryInit(void);
void telemetryUpdate(void);
bool telemetryGetSignalSnapshot(telemetry_signal_snapshot_t *p_snapshot);

/*
 * 디코더가 실제로 읽는 ID 만 통과하도록 수신 필터를 좁힌다.
 *
 * canOpen() 은 뱅크 0~1 에 전체 통과 필터를 깔고 나오므로, 반드시 채널을 연
 * 뒤에 불러야 한다. 순서를 뒤집으면 이 설정이 도로 덮인다.
 */
bool telemetryConfigCanFilter(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* SRC_AP_SW_TELEMETRY_TELEMETRY_H_ */
