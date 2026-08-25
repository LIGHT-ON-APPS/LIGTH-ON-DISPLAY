/*
 * ap.c
 *
 * 애플리케이션 진입점.
 *
 * 이 보드가 하는 일은 셋뿐이다. 차량 CAN 에서 신호를 받고(display_link),
 * 그 값으로 계기판을 그리고(TouchGFX), 정비용 CLI 를 열어 둔다.
 */

#include "ap.h"
#include "display_link.h"

#ifdef _USE_HW_TOUCHGFX
#include "app_touchgfx.h"
#endif
#ifdef _USE_HW_CLI
#include "cli.h"
#endif


/*
 * CAN 링크를 먼저 세우고 화면을 올린다.
 *
 * 순서가 중요하다. TouchGFX 는 초기화하면서 첫 화면을 바로 그리는데, 그때
 * display_link 가 아직 없으면 스냅샷을 못 읽어 "링크 없음"으로 한 번 그렸다가
 * 곧바로 다시 그리게 된다. CLI 는 화면과 무관하므로 가장 먼저 연다.
 */
void apInit(void)
{
#ifdef _USE_HW_CLI
  (void)cliOpen(_DEF_UART1, 57600);
#endif

  (void)displayLinkInit();

#ifdef _USE_HW_TOUCHGFX
  MX_TouchGFX_Init();
#endif
}

/*
 * CAN 수신을 먼저 처리하고, 남는 시간을 화면에 준다.
 *
 * MX_TouchGFX_Process() 는 소프트웨어 VSYNC 주기가 아직 안 됐으면 곧바로
 * 돌아오므로 이 루프는 막히지 않는다. CLI 는 가장 나중에 둔다.
 */
void apMain(void)
{
  while (1)
  {
    /*
     * CAN 에러 상태를 살피고 필요하면 회복을 건다.
     *
     * bus-off 자체는 canOpen() 이 AutoBusOff 를 켜 두었으므로 하드웨어가
     * 풀지만, error-passive 로 주저앉은 채 버티는 경우는 여기서
     * canRecovery() 를 걸어 줘야 정상 수신으로 돌아온다. 이 호출이 빠져
     * 있던 동안에는 회복 로직이 코드에만 있고 아무도 돌리지 않았다.
     */
    (void)canUpdate();

    displayLinkUpdate();

#ifdef _USE_HW_TOUCHGFX
    MX_TouchGFX_Process();
#endif

#ifdef _USE_HW_CLI
    cliMain();
#endif
  }
}
