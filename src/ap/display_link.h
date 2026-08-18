/*
 * display_link.h
 *
 * 차량 CAN 버스에서 계기판이 쓸 신호를 모아 스냅샷으로 제공한다.
 *
 * 예전 구조는 APPS 보드(L432)가 UART 로 display_protocol 패킷을 보내고
 * 디스플레이(F411)가 그것을 파싱하는 것이었다. 지금은 디스플레이 자신이
 * 차량 CAN 에 직접 붙으므로 중간 홉이 없다. Orion BMS 와 Bamocar 인버터
 * 프레임은 telemetry 디코더가 읽고, 이 모듈은 그 결과를 화면이 기대하는
 * 형태(display_protocol_telemetry_t)로 옮기고 신선도를 판정한다.
 *
 * APPS 페달 상태는 CAN 에 실려 오지 않는다. 그래서 status 는 링크 유무와
 * 인버터 경고만 채우고, 페달 관련 필드는 0 으로 둔다. APPS 보드가 나중에
 * 상태 프레임을 브로드캐스트하게 되면 여기에 디코더를 하나 더 붙이면 된다.
 */

#ifndef SRC_AP_DISPLAY_LINK_H_
#define SRC_AP_DISPLAY_LINK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"
#include "display_protocol.h"


#define DISPLAY_LINK_CAN_CH                _DEF_CAN1
#define DISPLAY_LINK_CAN_BAUD              CAN_500K

/*
 * 링크가 끊겼다고 보기까지의 시간.
 *
 * 차량 CAN 에는 Orion 과 Bamocar 가 최소 10 Hz 로 프레임을 올린다. 300 ms 동안
 * 아무 프레임도 없으면 버스가 죽었거나 우리 노드가 버스에서 떨어진 것이다.
 */
#define DISPLAY_LINK_TIMEOUT_MS            300U

/*
 * 텔레메트리 값 자체의 신선도. 프레임은 오는데 특정 신호만 갱신이 끊기는
 * 경우가 있어 링크 판정과 따로 둔다. 이 시간이 지나면 화면은 마지막 숫자
 * 대신 자리표시자를 띄운다.
 */
#define DISPLAY_LINK_TELEMETRY_TIMEOUT_MS  1000U


typedef struct
{
  display_protocol_status_t status;
  uint32_t last_valid_ms;
  uint32_t valid_packet_count;
  uint32_t rx_frame_count;
  bool     connected;
  bool     has_data;

  display_protocol_telemetry_t telemetry;
  uint32_t telemetry_last_valid_ms;
  uint32_t telemetry_packet_count;
  bool     telemetry_fresh;
  bool     has_telemetry;
} display_link_snapshot_t;


bool displayLinkInit(void);
void displayLinkUpdate(void);
bool displayLinkGetSnapshot(display_link_snapshot_t *p_snapshot);


#ifdef __cplusplus
}
#endif

#endif /* SRC_AP_DISPLAY_LINK_H_ */
