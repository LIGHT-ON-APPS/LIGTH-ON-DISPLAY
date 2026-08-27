/*
 * ili9488.c
 *
 *  Created on: Jun 25, 2026
 *      Author: young
 */

#include "spi.h"
#include "gpio.h"
#include "lcd/ili9488.h"
#include "lcd/ili9488_reg.h"

#ifdef _USE_HW_ILI9488
#ifdef _USE_HW_SPI
#ifdef _USE_HW_GPIO

#define ILI9488_SPI_CH                       _DEF_SPI1   // SPI1 (PA1 SCK, PA7 MOSI)
#define ILI9488_CS_CH    					 _DEF_GPIO4  // PB7  (D4)
#define ILI9488_RST_CH   					 _DEF_GPIO3  // PB6  (D5)
#define ILI9488_DC_CH    					 _DEF_GPIO2  // PB5  (D11)

#define ILI9488_TX_CHUNK_PIXELS              128U
#define ILI9488_TX_CHUNK_BYTES               (ILI9488_TX_CHUNK_PIXELS * 3U)
#define ILI9488_SPI_TIMEOUT_MS                1000U

/*
 * 전원/리셋 타이밍.
 *
 * SETTLE 은 패널 전원이 올라올 때까지 기다리는 시간이고, LOW 와 WAIT 는
 * 데이터시트가 요구하는 리셋 펄스 폭(10us 이상)과 리셋 해제 후 대기(120ms)에
 * 여유를 얹은 값이다. 부팅 때 한 번만 도는 구간이라 넉넉히 잡아도 손해가 없다.
 */
#define ILI9488_POWER_SETTLE_MS               300U
#define ILI9488_RESET_LOW_MS                  50U
#define ILI9488_RESET_WAIT_MS                 150U

/*
 * 소프트 리셋 후 대기.
 *
 * 데이터시트는 SWRESET 다음 명령까지 5ms, 그중에서도 SLPOUT 을 보내려면 120ms 를
 * 기다리라고 한다. 컨트롤러가 내부 전압을 다시 세우는 동안이라 그 전에 들어온
 * SLPOUT 은 그냥 버려진다. 버려져도 SPI 쓰기 자체는 성공하므로 뒤따르는 레지스터
 * 설정과 DISPON 은 그대로 들어가고, 결과는 '백라이트만 켜지고 화면은 검은' 상태다.
 */
#define ILI9488_SWRESET_WAIT_MS               150U

/*
 * 리셋+초기화를 몇 번 돌 것인가.
 *
 * 1 이면 예전 동작(한 번만). 2 면 첫 회차가 날아가도 두 번째가 받아 준다. 한 회차가
 * 약 610ms 이므로 늘릴수록 부팅이 그만큼 늦어진다. 되돌리려면 이 값만 1 로 바꾸면
 * 된다 — 나머지 코드는 손댈 것이 없다.
 */
#define ILI9488_INIT_ATTEMPTS                 2U

/*
 * SPI 분주.
 *
 * 프레임은 픽셀당 3바이트라 전송량이 많아 20MHz(분주 4)로 밀지만, 그 값은
 * 데이터시트 쓰기 사이클 하한(50ns)에 정확히 걸려 마진이 없다. 부팅 초기화는
 * 총 수십 바이트뿐이라 속도가 아무 의미가 없으므로, 이 구간만 5MHz 로 내려
 * 배선/노이즈 여유를 벌고 끝나면 프레임 속도로 되돌린다.
 */
#define ILI9488_SPI_DIV_INIT                  16U
#define ILI9488_SPI_DIV_FRAME                 4U

/*
 * 비동기 전송용 변환 버퍼.
 *
 * ILI9488 은 SPI 에서 16bpp 를 받지 못하므로(데이터시트상 16-bit/pixel 은 병렬
 * 인터페이스 전용) RGB565 프레임버퍼를 픽셀당 3바이트 RGB666 으로 풀어서 보내야
 * 한다. 그 변환 결과를 통째로 담아 DMA 에 넘기기 위한 버퍼다.
 *
 * 크기는 TouchGFX 부분 프레임버퍼의 블록 크기에 맞춘다. 블록 하나가 한 번의
 * DMA 로 나가야, 전송이 도는 동안 CPU 가 다음 블록을 그릴 수 있다.
 *
 * TouchGFXGeneratedHAL.cpp 의 ManyBlockAllocator 는 블록을 4800 바이트로 잡고
 * 픽셀당 2바이트(RGB565)이므로 한 블록은 2400 픽셀이다. 여기를 그보다 크게
 * 잡아 봐야 쓰이지 않고 RAM 만 먹는다. 반대로 작게 잡으면 블록이 한 번에 못
 * 나가 전송이 실패하므로, 블록 크기를 바꾸면 이 값도 같이 바꿔야 한다.
 */
#define ILI9488_TX_BUF_PIXELS                2400U
#define ILI9488_TX_BUF_BYTES                 (ILI9488_TX_BUF_PIXELS * 3U)

#define MADCTL_MY                            ILI9488_MADCTL_MY
#define MADCTL_MX                            ILI9488_MADCTL_MX
#define MADCTL_MV                            ILI9488_MADCTL_MV
#define MADCTL_ML                            ILI9488_MADCTL_ML
#define MADCTL_RGB                           0x00U
#define MADCTL_BGR                           ILI9488_MADCTL_BGR
#define MADCTL_MH                            ILI9488_MADCTL_MH


static uint8_t spi_ch = ILI9488_SPI_CH;

static void (*frameCallBack)(void) = NULL;
volatile static bool is_write_frame = false;
static uint8_t tx_buf[ILI9488_TX_BUF_BYTES];


static bool writecommand(uint8_t c);
static bool writedata(uint8_t d);
static bool writedata_buf(const uint8_t *p_data, uint32_t length);
static void ili9488GpioInit(void);
static bool ili9488InitRegs(void);


static void TransferDoneISR(void)
{
  if (is_write_frame == true)
  {
    is_write_frame = false;
    gpioPinWrite(ILI9488_CS_CH, _DEF_HIGH);

    if (frameCallBack != NULL)
    {
      frameCallBack();
    }
  }
}

bool ili9488Init(void)
{
  return ili9488Reset();
}

bool ili9488InitDriver(lcd_driver_t *p_driver)
{
  if (p_driver == NULL)
  {
    return false;
  }

  p_driver->init        = ili9488Init;
  p_driver->reset       = ili9488Reset;
  p_driver->setWindow   = ili9488SetWindow;
  p_driver->getWidth    = ili9488GetWidth;
  p_driver->getHeight   = ili9488GetHeight;
  p_driver->setCallBack = ili9488SetCallBack;
  p_driver->sendBuffer  = ili9488SendBuffer;
  p_driver->isTxBusy    = ili9488IsTxBusy;

  return true;
}

bool ili9488Reset(void)
{
  bool ret;

  ili9488GpioInit();

  is_write_frame = false;

  gpioPinWrite(ILI9488_DC_CH,  _DEF_HIGH);
  gpioPinWrite(ILI9488_CS_CH,  _DEF_HIGH);

  /* 전원이 올라오는 동안은 리셋을 걸어 둔다. 아래 대기 구간 설명 참조. */
  gpioPinWrite(ILI9488_RST_CH, _DEF_LOW);

  if (spiBegin(spi_ch) != true)
  {
    return false;
  }

  spiSetBitWidth(spi_ch, 8);
  spiSetDataMode(spi_ch, SPI_MODE0);
  spiSetClockDivider(spi_ch, ILI9488_SPI_DIV_INIT);
  spiAttachTxInterrupt(spi_ch, TransferDoneISR);

  /*
   * 리셋을 건 채로 패널 전원이 올라오기를 기다린다.
   *
   * 디스플레이 보드가 LCD 모듈보다 먼저 살아나는 배선이라, 부팅하자마자 RST 를
   * 토글하면 컨트롤러는 아직 전원이 서지 않은 채로 그 펄스를 흘려보낸다. 그
   * 뒤에 오는 레지스터 설정도 함께 사라지므로 화면은 끝까지 검은 채로 남는다.
   * 백라이트는 컨트롤러를 거치지 않고 켜지기 때문에, 밖에서 보면 '불은 들어오는데
   * 화면만 안 나오는' 모습이 된다.
   *
   * RST 를 HIGH 로 둔 채 기다리지 않는 이유가 여기 있다. 리셋이 풀린 상태로 전원이
   * 천천히 올라오면 컨트롤러는 불안정한 전압에서 제 나름의 파워온 리셋을 진행하다가
   * 어중간한 상태로 자리잡는다. 전원이 서는 동안 리셋을 붙잡고 있다가 다 올라온 뒤에
   * 풀어 주는 것이 정석이고, gpio.c 의 RST 초기값을 LOW 로 둔 것도 같은 이유다
   * (gpioInit 이 lcdInit 보다 먼저 돌기 때문에 거기서부터 잡아 두어야 한다).
   */
  delay(ILI9488_POWER_SETTLE_MS);

  /*
   * 리셋과 초기화를 두 번 돈다.
   *
   * SPI 가 MOSI 단방향이라 초기화가 실제로 먹었는지 읽어서 확인할 방법이 없다.
   * 확인이 안 되면 "실패했을 때만 다시" 라는 조건을 세울 수가 없으므로, 첫 회차의
   * 성패와 무관하게 무조건 한 번 더 돈다. 첫 회차가 통째로 날아가더라도 두 번째는
   * 전원 인가 후 900ms 가 지난 시점이라 거의 확실히 살아 있는 컨트롤러로 들어간다.
   *
   * 전원 대기는 위에서 한 번만 한다. 두 번째 회차 시점에는 전원이 이미 안정되어
   * 있어 또 기다릴 이유가 없다. 대신 회차마다 하드웨어 리셋 펄스부터 다시 때린다.
   * 레지스터 설정만 되풀이하면 컨트롤러가 아예 죽어 있던 경우를 못 잡는다.
   *
   * 백라이트는 lcd.c 가 lcdInit() 성공 뒤에 켜므로 이 구간 내내 꺼져 있다. 몇 번을
   * 돌든 화면에 깜빡임으로 보이지는 않는다.
   *
   * 근본 해결이 아니라 확률을 낮추는 대책이라는 점은 분명히 해 둔다. 제대로 고치려면
   * MISO 를 연결해 RDDPM(0x0A) 으로 슬립 해제 여부를 읽고 그때만 재시도해야 한다.
   */
  for (uint32_t i = 0U; i < ILI9488_INIT_ATTEMPTS; i++)
  {
    /*
     * 첫 회차는 위 대기 구간 내내 RST 가 LOW 였으므로 펄스 폭이 이미 300ms 확보돼
     * 있다. 다시 내릴 것 없이 바로 풀면 된다. 두 번째 회차부터는 직전 회차가 RST 를
     * HIGH 로 올려 두었으니 새로 펄스를 만들어야 한다.
     */
    if (i > 0U)
    {
      gpioPinWrite(ILI9488_RST_CH, _DEF_LOW);
      delay(ILI9488_RESET_LOW_MS);
    }

    gpioPinWrite(ILI9488_RST_CH, _DEF_HIGH);
    delay(ILI9488_RESET_WAIT_MS);

    /*
     * 가로 방향에서 180도 돌린 화면.
     *
     * mode 1 과 3 은 둘 다 MV(행/열 교환)로 가로를 만들고, 3 은 거기에 MX 와 MY
     * 미러를 더해 좌우와 상하를 같이 뒤집는다. 즉 컨트롤러가 프레임버퍼를 읽는
     * 순서만 달라질 뿐이라 회전 비용이 따로 들지 않는다. TouchGFX 쪽 좌표계는
     * 그대로 480x320 이므로 화면 구성 코드는 손댈 것이 없다.
     */
    ret = (ili9488InitRegs() == true) && (ili9488SetRotation(3) == true);
  }

  /* 중간에 실패해도 분주는 되돌린다. 초기화 구간용 저속이 그대로 남으면
   * 이후 프레임 전송이 이유 없이 4배 느려진다. */
  spiSetClockDivider(spi_ch, ILI9488_SPI_DIV_FRAME);

  return ret;
}

uint16_t ili9488GetWidth(void)
{
  return HW_LCD_WIDTH;
}

uint16_t ili9488GetHeight(void)
{
  return HW_LCD_HEIGHT;
}

static bool writecommand(uint8_t c)
{
  bool ret;

  spiSetBitWidth(spi_ch, 8);

  gpioPinWrite(ILI9488_DC_CH, _DEF_LOW);
  gpioPinWrite(ILI9488_CS_CH, _DEF_LOW);

  ret = spiWrite(spi_ch, &c, 1, ILI9488_SPI_TIMEOUT_MS);

  gpioPinWrite(ILI9488_CS_CH, _DEF_HIGH);

  return ret;
}

static bool writedata(uint8_t d)
{
  bool ret;

  spiSetBitWidth(spi_ch, 8);

  gpioPinWrite(ILI9488_DC_CH, _DEF_HIGH);
  gpioPinWrite(ILI9488_CS_CH, _DEF_LOW);

  ret = spiWrite(spi_ch, &d, 1, ILI9488_SPI_TIMEOUT_MS);

  gpioPinWrite(ILI9488_CS_CH, _DEF_HIGH);

  return ret;
}

static bool writedata_buf(const uint8_t *p_data, uint32_t length)
{
  bool ret;

  if ((p_data == NULL) || (length == 0U))
  {
    return false;
  }

  spiSetBitWidth(spi_ch, 8);

  gpioPinWrite(ILI9488_DC_CH, _DEF_HIGH);
  gpioPinWrite(ILI9488_CS_CH, _DEF_LOW);

  ret = spiWrite(spi_ch, p_data, length, ILI9488_SPI_TIMEOUT_MS);

  gpioPinWrite(ILI9488_CS_CH, _DEF_HIGH);

  return ret;
}

static void ili9488GpioInit(void)
{
  gpioPinMode(ILI9488_CS_CH, _DEF_OUTPUT);
  gpioPinMode(ILI9488_DC_CH, _DEF_OUTPUT);
  gpioPinMode(ILI9488_RST_CH, _DEF_OUTPUT);
}

static bool ili9488InitRegs(void)
{
  static const uint8_t pgamma[] =
  {
    0x00, 0x03, 0x09, 0x08, 0x16,
    0x0A, 0x3F, 0x78, 0x4C, 0x09,
    0x0A, 0x08, 0x16, 0x1A, 0x0F
  };
  static const uint8_t ngamma[] =
  {
    0x00, 0x16, 0x19, 0x03, 0x0F,
    0x05, 0x32, 0x45, 0x46, 0x04,
    0x0E, 0x0D, 0x35, 0x37, 0x0F
  };

  if (writecommand(ILI9488_SWRESET) != true)
  {
    return false;
  }
  delay(ILI9488_SWRESET_WAIT_MS);

  if (writecommand(ILI9488_SLPOUT) != true)
  {
    return false;
  }
  delay(120);

  if ((writecommand(ILI9488_PGAMCTRL) != true) ||
      (writedata_buf(pgamma, sizeof(pgamma)) != true))
  {
    return false;
  }

  if ((writecommand(ILI9488_NGAMCTRL) != true) ||
      (writedata_buf(ngamma, sizeof(ngamma)) != true))
  {
    return false;
  }

  if ((writecommand(ILI9488_PWCTR1) != true) ||
      (writedata(0x17) != true) ||
      (writedata(0x15) != true))
  {
    return false;
  }

  if ((writecommand(ILI9488_PWCTR2) != true) ||
      (writedata(0x41) != true))
  {
    return false;
  }

  if ((writecommand(ILI9488_VMCTR1) != true) ||
      (writedata(0x00) != true) ||
      (writedata(0x12) != true) ||
      (writedata(0x80) != true))
  {
    return false;
  }

  if ((writecommand(ILI9488_FRMCTR1) != true) ||
      (writedata(0xA0) != true))
  {
    return false;
  }

  if ((writecommand(ILI9488_DFUNCTR) != true) ||
      (writedata(0x02) != true) ||
      (writedata(0x02) != true) ||
      (writedata(0x3B) != true))
  {
    return false;
  }

  if ((writecommand(ILI9488_ENTMOD) != true) ||
      (writedata(0xC6) != true))
  {
    return false;
  }

  if ((writecommand(ILI9488_ADJCTL3) != true) ||
      (writedata(0xA9) != true) ||
      (writedata(0x51) != true) ||
      (writedata(0x2C) != true) ||
      (writedata(0x82) != true))
  {
    return false;
  }

  if ((writecommand(ILI9488_PIXFMT) != true) ||
      (writedata(ILI9488_PIXFMT_18BIT) != true))
  {
    return false;
  }

  if (writecommand(ILI9488_NORON) != true)
  {
    return false;
  }
  delay(10);

  /*
   * This ILI9488 module's panel polarity is opposite to the controller's
   * reset default. Without inversion the whole TouchGFX UI is displayed as
   * a photographic negative (dark background becomes bright, white text
   * becomes black, and green becomes magenta).
   */
  if (writecommand(ILI9488_INVON) != true)
  {
    return false;
  }
  delay(10);

  if (writecommand(ILI9488_DISPON) != true)
  {
    return false;
  }
  delay(120);

  return true;
}

bool ili9488SetRotation(uint8_t mode)
{
  uint8_t madctl;

  if (writecommand(ILI9488_MADCTL) != true)
  {
    return false;
  }

  switch (mode & 0x03U)
  {
    case 0:
      madctl = MADCTL_MX | MADCTL_BGR;
      break;

    case 1:
      madctl = MADCTL_MV | MADCTL_BGR;
      break;

    case 2:
      madctl = MADCTL_MY | MADCTL_BGR;
      break;

    default:
      madctl = MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR;
      break;
  }

  return writedata(madctl);
}

bool ili9488SetWindow(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
  /* 창 설정은 SPI 로 명령을 직접 밀어 넣는다. 앞 블록의 DMA 가 아직 SPI 를 물고
   * 있으면 그 위에 겹쳐 써서 화면이 어긋나므로 먼저 비워지길 기다린다. */
  {
    uint32_t start_ms = millis();

    while (ili9488IsTxBusy() == true)
    {
      if ((millis() - start_ms) >= ILI9488_SPI_TIMEOUT_MS)
      {
        return false;
      }
    }
  }

  x0 = constrain(x0, 0, HW_LCD_WIDTH - 1);
  x1 = constrain(x1, 0, HW_LCD_WIDTH - 1);
  y0 = constrain(y0, 0, HW_LCD_HEIGHT - 1);
  y1 = constrain(y1, 0, HW_LCD_HEIGHT - 1);

  if (x0 > x1)
  {
    int32_t tmp = x0;
    x0 = x1;
    x1 = tmp;
  }

  if (y0 > y1)
  {
    int32_t tmp = y0;
    y0 = y1;
    y1 = tmp;
  }

  spiSetBitWidth(spi_ch, 8);

  if ((writecommand(ILI9488_CASET) != true) ||
      (writedata((uint8_t)(x0 >> 8)) != true) ||
      (writedata((uint8_t)x0) != true) ||
      (writedata((uint8_t)(x1 >> 8)) != true) ||
      (writedata((uint8_t)x1) != true) ||
      (writecommand(ILI9488_PASET) != true) ||
      (writedata((uint8_t)(y0 >> 8)) != true) ||
      (writedata((uint8_t)y0) != true) ||
      (writedata((uint8_t)(y1 >> 8)) != true) ||
      (writedata((uint8_t)y1) != true) ||
      (writecommand(ILI9488_RAMWR) != true))
  {
    return false;
  }

  return true;
}

/* RGB565 한 픽셀을 ILI9488 이 받는 RGB666 3바이트로 편다. */
static uint32_t ili9488Expand(const uint16_t *p_color,
                              uint8_t *p_out,
                              uint32_t pixels)
{
  uint32_t tx_len = 0;

  for (uint32_t i = 0; i < pixels; i++)
  {
    uint16_t color = *p_color++;
    uint8_t r = (uint8_t)((color >> 11) & 0x1FU);
    uint8_t g = (uint8_t)((color >> 5) & 0x3FU);
    uint8_t b = (uint8_t)(color & 0x1FU);

    p_out[tx_len++] = (uint8_t)((r << 3) | (r >> 2));
    p_out[tx_len++] = (uint8_t)((g << 2) | (g >> 4));
    p_out[tx_len++] = (uint8_t)((b << 3) | (b >> 2));
  }

  return tx_len;
}

bool ili9488IsTxBusy(void)
{
  return (is_write_frame == true) && (spiDmaTxIsDone(spi_ch) != true);
}

/*
 * 한 블록을 화면으로 내보낸다.
 *
 * 블록이 변환 버퍼에 들어가면 DMA 로 넘기고 바로 돌아온다. 전송이 끝나는 시점은
 * SPI TX 완료 인터럽트가 TransferDoneISR 로 알려 준다. 이렇게 해야 DMA 가 SPI 를
 * 물고 있는 동안 CPU 가 다음 블록을 그릴 수 있다. 예전처럼 폴링으로 다 보내고
 * 돌아오면 그리기와 전송이 번갈아 직렬로 돌아서, 프레임 시간이 두 시간의 합이
 * 된다.
 */
bool ili9488SendBuffer(uint8_t *p_data, uint32_t length, uint32_t timeout_ms)
{
  const uint16_t *p_color = (const uint16_t *)p_data;
  uint32_t spi_timeout = timeout_ms;
  uint32_t tx_len;

  if ((p_data == NULL) || (length == 0U))
  {
    return false;
  }

  if (spi_timeout == 0U)
  {
    spi_timeout = ILI9488_SPI_TIMEOUT_MS;
  }

  /* 앞 블록이 아직 나가는 중이면 기다린다. 정상 경로에서는 상위(TouchGFX)가
   * ili9488IsTxBusy() 를 보고 먼저 막아 주므로 여기까지 오지 않는다. */
  if (ili9488IsTxBusy() == true)
  {
    uint32_t start_ms = millis();

    while (ili9488IsTxBusy() == true)
    {
      if ((millis() - start_ms) >= spi_timeout)
      {
        return false;
      }
    }
  }

  gpioPinWrite(ILI9488_DC_CH, _DEF_HIGH);
  gpioPinWrite(ILI9488_CS_CH, _DEF_LOW);
  spiSetBitWidth(spi_ch, 8);

  if (length > ILI9488_TX_BUF_PIXELS)
  {
    /*
     * 변환 버퍼보다 큰 요청. TouchGFX 블록 전송은 여기 걸리지 않고, 화면 전체를
     * 한 번에 칠하는 lcd 자체 API 만 온다. 그쪽은 갱신 속도가 중요하지 않으므로
     * 예전처럼 나눠서 동기 전송한다.
     */
    is_write_frame = true;

    while (length > 0U)
    {
      uint32_t chunk_pixels = (length > ILI9488_TX_CHUNK_PIXELS)
                                  ? ILI9488_TX_CHUNK_PIXELS
                                  : length;

      tx_len = ili9488Expand(p_color, tx_buf, chunk_pixels);
      p_color += chunk_pixels;

      if (spiWrite(spi_ch, tx_buf, tx_len, spi_timeout) != true)
      {
        gpioPinWrite(ILI9488_CS_CH, _DEF_HIGH);
        is_write_frame = false;
        return false;
      }

      length -= chunk_pixels;
    }

    TransferDoneISR();
    return true;
  }

  tx_len = ili9488Expand(p_color, tx_buf, length);

  /* is_write_frame 은 완료 인터럽트가 이 전송을 우리 것으로 알아보는 표시다.
   * DMA 를 걸기 전에 세워야 완료가 빨리 떠도 놓치지 않는다. */
  is_write_frame = true;
  spiDmaTxStart(spi_ch, tx_buf, tx_len);

  if (spiDmaTxIsDone(spi_ch) == true)
  {
    /*
     * DMA 를 걸지 못했다(또는 이미 끝났다). 완료 인터럽트가 오지 않는 경우
     * 상위가 다음 블록을 영영 못 보내고 화면이 멎으므로 여기서 마무리한다.
     * 인터럽트가 이미 처리했다면 is_write_frame 이 내려가 있어 두 번 돌지 않는다.
     */
    TransferDoneISR();
  }

  return true;
}

bool ili9488SetCallBack(void (*p_func)(void))
{
  frameCallBack = p_func;

  return true;
}

#endif
#endif
#endif
