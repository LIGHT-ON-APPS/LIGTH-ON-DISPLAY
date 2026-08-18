/*
 * display_link.c
 *
 * telemetry 디코더가 CAN 에서 뽑아낸 차량 신호를 화면용 스냅샷으로 옮긴다.
 */

#include "display_link.h"
#include "telemetry/telemetry.h"

#include <string.h>

#ifdef _USE_HW_CLI
#include "cli.h"
#endif


typedef struct
{
  bool     initialized;
  display_link_snapshot_t snapshot;

  /* 직전에 반영한 telemetry 갱신 시각. 값이 그대로면 화면을 건드리지 않는다. */
  uint32_t signal_update_ms;
  bool     signal_update_valid;

  /* 링크 살아 있음 판정은 프레임 수 증가로만 한다. 어떤 ID 든 우리 필터를
   * 통과해 들어왔다면 버스는 살아 있는 것이다. */
  uint32_t last_rx_frame_count;
} display_link_t;

static display_link_t display_link;


#ifdef _USE_HW_CLI
static void displayLinkCli(cli_args_t *args);
#endif


/*
 * telemetry 신호를 화면이 읽는 형태로 옮긴다.
 *
 * TELEMETRY_SIGNAL_* 과 DISPLAY_PROTOCOL_TLM_* 은 비트 배치가 같게 유지되고
 * 있으므로(양쪽 헤더 주석 참고) valid_mask 는 변환 없이 그대로 넘긴다.
 * 단위만 화면 쪽 표기(0.1 V, 0.1 A)로 낮춘다.
 */
static void displayLinkMapSignal(const telemetry_signal_snapshot_t *p_signal)
{
  display_protocol_telemetry_t *p_tlm = &display_link.snapshot.telemetry;

  p_tlm->valid_mask          = p_signal->valid_mask;
  p_tlm->source_time_ms      = p_signal->last_update_ms;
  p_tlm->motor_rpm           = p_signal->bamocar_rpm;
  p_tlm->cell_min_mv         = p_signal->bms_min_cell_mv;
  p_tlm->pack_dv             = (uint16_t)(p_signal->bms_pack_mv / 100L);
  p_tlm->pack_da             = (int16_t)(p_signal->bms_pack_ma / 100L);
  p_tlm->soc_per_mille       = p_signal->bms_soc_permille;
  p_tlm->cell_max_temp_decic = p_signal->bms_max_temp_decic;
  p_tlm->igbt_temp_decic     = p_signal->bamocar_igbt_temp_decic;
  p_tlm->motor_temp_decic    = p_signal->bamocar_motor_temp_decic;
  p_tlm->sequence++;
}

/*
 * 화면 상단의 상태 배지가 읽는 필드.
 *
 * 페달 값은 CAN 에 없으므로 0 으로 두고, 화면이 "값이 유효하다"고 볼 수 있도록
 * VALID 플래그만 세운다. fault 에는 Bamocar 의 에러/경고 워드를 그대로 싣는다.
 * 인버터 상태 워드(status)는 정상 주행 중에도 0 이 아니므로 여기에 넣으면
 * 화면이 늘 고장으로 보인다. 넣지 않는다.
 */
static void displayLinkMapStatus(const telemetry_signal_snapshot_t *p_signal)
{
  display_protocol_status_t *p_status = &display_link.snapshot.status;
  uint32_t fault = 0U;

  if ((p_signal->valid_mask & TELEMETRY_SIGNAL_BAMOCAR_ERROR_WARNING) != 0U)
  {
    fault = p_signal->bamocar_error_warning;
  }

  p_status->flags           = DISPLAY_PROTOCOL_FLAG_VALID |
                              DISPLAY_PROTOCOL_FLAG_CONFIGURED;
  p_status->source_time_ms  = p_signal->last_update_ms;
  p_status->status          = 0U;
  p_status->fault           = fault;
  p_status->apps1_raw       = 0U;
  p_status->apps2_raw       = 0U;
  p_status->apps1_per_mille = 0U;
  p_status->apps2_per_mille = 0U;
  p_status->pedal_per_mille = 0U;
  p_status->command         = 0;
}

bool displayLinkInit(void)
{
  memset(&display_link, 0, sizeof(display_link));

  (void)telemetryInit();

  /* CAN 은 hw 계층에서 이미 열렸을 수 있다. 닫혀 있을 때만 우리가 연다. */
  if (canIsOpen(DISPLAY_LINK_CAN_CH) != true)
  {
    (void)canOpen(DISPLAY_LINK_CAN_CH,
                  CAN_NORMAL,
                  CAN_CLASSIC,
                  DISPLAY_LINK_CAN_BAUD,
                  DISPLAY_LINK_CAN_BAUD);
  }

  display_link.last_rx_frame_count = canGetRxCount(DISPLAY_LINK_CAN_CH);
  display_link.initialized = true;

#ifdef _USE_HW_CLI
  (void)cliAdd("display", displayLinkCli);
#endif

  return true;
}

void displayLinkUpdate(void)
{
  telemetry_signal_snapshot_t signal;
  uint32_t now_ms;
  uint32_t rx_frame_count;

  if (display_link.initialized != true)
  {
    (void)displayLinkInit();
  }

  /* CAN 수신 큐를 비우고 디코딩한다. */
  telemetryUpdate();

  now_ms = millis();
  rx_frame_count = canGetRxCount(DISPLAY_LINK_CAN_CH);

  if (rx_frame_count != display_link.last_rx_frame_count)
  {
    display_link.last_rx_frame_count = rx_frame_count;
    display_link.snapshot.rx_frame_count = rx_frame_count;
    display_link.snapshot.last_valid_ms = now_ms;
    display_link.snapshot.has_data = true;

    if (display_link.snapshot.connected != true)
    {
      display_link.snapshot.connected = true;
      display_link.snapshot.valid_packet_count++;
    }
  }

  if (telemetryGetSignalSnapshot(&signal) == true)
  {
    const bool signal_changed =
        (display_link.signal_update_valid != true) ||
        (signal.last_update_ms != display_link.signal_update_ms);

    if (signal_changed == true)
    {
      const uint32_t prev_fault = display_link.snapshot.status.fault;

      display_link.signal_update_ms = signal.last_update_ms;
      display_link.signal_update_valid = true;

      displayLinkMapSignal(&signal);
      displayLinkMapStatus(&signal);

      display_link.snapshot.telemetry_last_valid_ms = now_ms;
      display_link.snapshot.telemetry_packet_count++;
      display_link.snapshot.telemetry_fresh = true;
      display_link.snapshot.has_telemetry = true;

      /* 상태 배지는 고장 워드가 바뀔 때만 다시 그리게 한다. 매 프레임마다
       * 올리면 화면이 아무 변화 없이도 계속 갱신된다. */
      if (display_link.snapshot.status.fault != prev_fault)
      {
        display_link.snapshot.valid_packet_count++;
      }
    }
  }

  if ((display_link.snapshot.connected == true) &&
      ((now_ms - display_link.snapshot.last_valid_ms) >=
       DISPLAY_LINK_TIMEOUT_MS))
  {
    display_link.snapshot.connected = false;
    display_link.snapshot.valid_packet_count++;
  }

  if ((display_link.snapshot.telemetry_fresh == true) &&
      ((now_ms - display_link.snapshot.telemetry_last_valid_ms) >=
       DISPLAY_LINK_TELEMETRY_TIMEOUT_MS))
  {
    display_link.snapshot.telemetry_fresh = false;
  }
}

bool displayLinkGetSnapshot(display_link_snapshot_t *p_snapshot)
{
  if (p_snapshot == NULL)
  {
    return false;
  }

  *p_snapshot = display_link.snapshot;

  return display_link.snapshot.has_data;
}


#ifdef _USE_HW_CLI
static void displayLinkCli(cli_args_t *args)
{
  bool handled = false;

  if ((args->argc == 1) && args->isStr(0, "info"))
  {
    const display_link_snapshot_t *p_snap = &display_link.snapshot;

    cliPrintf("can ch     : %d\n", DISPLAY_LINK_CAN_CH);
    cliPrintf("can open   : %d\n", canIsOpen(DISPLAY_LINK_CAN_CH));
    cliPrintf("connected  : %d\n", p_snap->connected);
    cliPrintf("rx frames  : %lu\n", (unsigned long)p_snap->rx_frame_count);
    cliPrintf("tlm frames : %lu\n",
              (unsigned long)p_snap->telemetry_packet_count);
    cliPrintf("tlm fresh  : %d\n", p_snap->telemetry_fresh);
    cliPrintf("valid mask : 0x%08lX\n",
              (unsigned long)p_snap->telemetry.valid_mask);
    cliPrintf("fault      : 0x%08lX\n",
              (unsigned long)p_snap->status.fault);
    handled = true;
  }

  if ((args->argc == 1) && args->isStr(0, "value"))
  {
    while (cliKeepLoop())
    {
      const display_protocol_telemetry_t *p_tlm =
          &display_link.snapshot.telemetry;

      cliPrintf("rpm %ld  soc %u.%u%%  cell %u.%03uV  pack %u.%uV  "
                "mot %d.%dC  igbt %d.%dC\n",
                (long)p_tlm->motor_rpm,
                (unsigned int)(p_tlm->soc_per_mille / 10U),
                (unsigned int)(p_tlm->soc_per_mille % 10U),
                (unsigned int)(p_tlm->cell_min_mv / 1000U),
                (unsigned int)(p_tlm->cell_min_mv % 1000U),
                (unsigned int)(p_tlm->pack_dv / 10U),
                (unsigned int)(p_tlm->pack_dv % 10U),
                (int)(p_tlm->motor_temp_decic / 10),
                (int)(p_tlm->motor_temp_decic % 10),
                (int)(p_tlm->igbt_temp_decic / 10),
                (int)(p_tlm->igbt_temp_decic % 10));
      delay(200);
    }
    handled = true;
  }

  if (handled != true)
  {
    cliPrintf("display info\n");
    cliPrintf("display value\n");
  }
}
#endif
