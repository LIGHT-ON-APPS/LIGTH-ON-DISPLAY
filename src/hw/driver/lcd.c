/*
 * lcd.c
 *
 *  Created on: Dec 30, 2025
 *      Author: TEMP
 */


#include "lcd.h"
#include "cli.h"


#ifdef _USE_HW_LCD
#include "gpio.h"
#include "hangul/han.h"

#ifdef _USE_HW_ST7735
#include "lcd/st7735.h"
#endif
#ifdef _USE_HW_ILI9488
#include "lcd/ili9488.h"
#endif

#ifdef _USE_HW_TOUCHGFX
/* TouchGFX 는 C++ 라 여기서 헤더를 읽을 수 없다. HAL 이 extern "C" 로 내보낸
 * 것만 쓴다. 정의는 TouchGFX/target/TouchGFXHAL.cpp 에 있다. */
extern void touchgfxRequestFullRedraw(void);
#endif


#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif




#define MAKECOL(r, g, b) ( ((r)<<11) | ((g)<<5) | (b))




#define LCD_OPT_DEF   			__attribute__((optimize("O2")))
#define _PIN_DEF_BL_CTL       	_DEF_GPIO1
/*
 * 이 스트립 버퍼는 lcd.c 자체 도형/문자 API(lcdDrawFillRect, lcdPrintf 등)가
 * 쓰는 것이고, 화면을 실제로 그리는 TouchGFX 경로는 거치지 않는다. L432KC 는
 * RAM 이 64 KB 뿐이라 여기에 8 줄(7680 바이트)을 묶어 두면 부분 프레임버퍼가
 * 들어갈 자리가 없다. 2 줄이면 CLI 진단 출력에는 충분하다.
 */
#define LCD_STRIP_BUF_LINES   	2U
#define LCD_STRIP_BUF_PIXELS  	(HW_LCD_WIDTH * LCD_STRIP_BUF_LINES)


static lcd_driver_t lcd;


static bool is_init = false;
static volatile bool is_tx_done = true;
static uint8_t backlight_value = 100;


static bool lcd_request_draw = false;

static volatile uint32_t fps_pre_time;
static volatile uint32_t fps_time;
static volatile uint32_t fps_count = 0;

static volatile uint32_t draw_fps = 30;
static volatile uint32_t draw_frame_time = 0;


static uint16_t lcd_strip_buf[LCD_STRIP_BUF_PIXELS];




static volatile bool requested_from_thread = false;
static void (*transfer_done_callback)(void) = NULL;




static void disHanFont(int x, int y, han_font_t *FontPtr, uint16_t textcolor);
static bool lcdClipRect(int16_t *x, int16_t *y, int16_t *w, int16_t *h);
static bool lcdSendColorRepeat(uint16_t color, uint32_t length);


#ifdef _USE_HW_CLI
static void cliLcd(cli_args_t *args);
#endif
// lcd frame buffer의 데이터를 쏜 다음 false로 바꿈
void TransferDoneISR(void)
{
  fps_time = millis() - fps_pre_time;
  fps_pre_time = millis();

  if (fps_time > 0)
  {
    fps_count = 1000 / fps_time;
  }

  lcd_request_draw = false;

  /* 부분 프레임버퍼로 그리는 쪽에 블록 하나가 나갔음을 알린다. 이걸 받아야
   * 다음 블록을 넘겨준다. 인터럽트 문맥이므로 콜백은 짧아야 한다. */
  if (transfer_done_callback != NULL)
  {
    transfer_done_callback();
  }
}

void lcdSetTransferDoneCallBack(void (*p_func)(void))
{
  transfer_done_callback = p_func;
}

bool lcdIsTxBusy(void)
{
  if (lcd.isTxBusy == NULL)
  {
    return false;
  }

  return lcd.isTxBusy();
}


bool lcdInit(void)
{
  bool ret = false;

  backlight_value = 100;
  is_init = false;


#ifdef _USE_HW_ST7735
  ret = st7735InitDriver(&lcd);
  if ((ret == true) && (lcd.init != NULL))
  {
    ret = lcd.init();
  }
#endif
#ifdef _USE_HW_ILI9488
  ret = ili9488InitDriver(&lcd);
  if ((ret == true) && (lcd.init != NULL))
  {
    ret = lcd.init();
  }
#endif

  if ((ret != true) ||
      (lcd.setCallBack == NULL) ||
      (lcd.setCallBack(TransferDoneISR) != true))
  {
    lcdSetBackLight(0);
    return false;
  }


  lcdSetBackLight(100);


  is_init = true;

#ifdef _USE_HW_CLI
  cliAdd("lcd", cliLcd);
#endif

  return true;
}

uint32_t lcdGetDrawTime(void)
{
  return draw_frame_time;
}

bool lcdIsInit(void)
{
  return is_init;
}

/*
 * 패널을 처음부터 다시 세운다.
 *
 * 부팅 때 '백라이트만 켜지고 화면이 안 뜨는' 증상을 눈앞에서 격리하려고 열어 둔
 * 경로다. 이 호출로 화면이 살아나면 원인은 부팅 시점의 전원/리셋 타이밍이고,
 * 그래도 검은 채로 남으면 배선이나 SPI 신호 쪽을 봐야 한다. SPI 가 MOSI 단방향
 * 이라 초기화가 실제로 먹었는지 읽어서 확인할 방법이 없으니, 이렇게 사람이 눈으로
 * 가르는 수밖에 없다.
 *
 * is_init 이 false 여도 돈다. 초기화에 실패한 상태에서 다시 시도할 수 없으면
 * 진단에 쓸 수가 없다.
 */
bool lcdReset(void)
{
  if (lcd.reset == NULL)
  {
    return false;
  }

  /*
   * 돌고 있는 DMA 전송 위로 리셋을 걸면 SPI 가 반쯤 나간 채로 끊긴다. 먼저
   * 비워지길 기다린다. 타임아웃까지 안 끝나면 어차피 정상 상태가 아니므로
   * 그대로 리셋을 진행한다.
   */
  {
    uint32_t start_ms = millis();

    while (lcdIsTxBusy() == true)
    {
      if ((millis() - start_ms) >= 1000U)
      {
        break;
      }
      delay(1);
    }
  }

  if (lcd.reset() != true)
  {
    is_init = false;
    lcdSetBackLight(0);
    return false;
  }

  is_init = true;
  lcdSetBackLight(100);

  return true;
}

uint8_t lcdGetBackLight(void)
{
  return backlight_value;
}

void lcdSetBackLight(uint8_t value)
{
  value = constrain(value, 0, 100);

  if (value != backlight_value)
  {
    backlight_value = value;
  }

  if (backlight_value > 0)
  {
    gpioPinWrite(_PIN_DEF_BL_CTL, _DEF_HIGH);
  }
  else
  {
    gpioPinWrite(_PIN_DEF_BL_CTL, _DEF_LOW);
  }
}

LCD_OPT_DEF uint32_t lcdReadPixel(uint16_t x_pos, uint16_t y_pos)
{
  (void)x_pos;
  (void)y_pos;

  return 0;
}

LCD_OPT_DEF void lcdDrawPixel(uint16_t x_pos, uint16_t y_pos, uint32_t rgb_code)
{
  uint16_t color = (uint16_t)rgb_code;

  if ((x_pos >= LCD_WIDTH) || (y_pos >= LCD_HEIGHT))
  {
    return;
  }

  if ((lcd.setWindow == NULL) || (lcd.sendBuffer == NULL))
  {
    return;
  }

  if (lcd.setWindow(x_pos, y_pos, x_pos, y_pos) != true)
  {
    return;
  }

  (void)lcd.sendBuffer((uint8_t *)&color, 1, 0);
}

LCD_OPT_DEF void lcdClear(uint32_t rgb_code)
{
  lcdClearBuffer(rgb_code);
}

LCD_OPT_DEF void lcdClearBuffer(uint32_t rgb_code)
{
  lcdDrawFillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, (uint16_t)rgb_code);
}

LCD_OPT_DEF void lcdDrawFillCircle(int32_t x0, int32_t y0, int32_t r, uint16_t color)
{
  int32_t  x  = 0;
  int32_t  dx = 1;
  int32_t  dy = r+r;
  int32_t  p  = -(r>>1);


  lcdDrawHLine(x0 - r, y0, dy+1, color);

  while(x<r)
  {

    if(p>=0) {
      dy-=2;
      p-=dy;
      r--;
    }

    dx+=2;
    p+=dx;

    x++;

    lcdDrawHLine(x0 - r, y0 + x, 2 * r+1, color);
    lcdDrawHLine(x0 - r, y0 - x, 2 * r+1, color);
    lcdDrawHLine(x0 - x, y0 + r, 2 * x+1, color);
    lcdDrawHLine(x0 - x, y0 - r, 2 * x+1, color);
  }
}

LCD_OPT_DEF void lcdDrawCircleHelper( int32_t x0, int32_t y0, int32_t r, uint8_t cornername, uint32_t color)
{
  int32_t f     = 1 - r;
  int32_t ddF_x = 1;
  int32_t ddF_y = -2 * r;
  int32_t x     = 0;

  while (x < r)
  {
    if (f >= 0)
    {
      r--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4)
    {
      lcdDrawPixel(x0 + x, y0 + r, color);
      lcdDrawPixel(x0 + r, y0 + x, color);
    }
    if (cornername & 0x2)
    {
      lcdDrawPixel(x0 + x, y0 - r, color);
      lcdDrawPixel(x0 + r, y0 - x, color);
    }
    if (cornername & 0x8)
    {
      lcdDrawPixel(x0 - r, y0 + x, color);
      lcdDrawPixel(x0 - x, y0 + r, color);
    }
    if (cornername & 0x1)
    {
      lcdDrawPixel(x0 - r, y0 - x, color);
      lcdDrawPixel(x0 - x, y0 - r, color);
    }
  }
}

LCD_OPT_DEF void lcdDrawRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint32_t color)
{
  // smarter version
  lcdDrawHLine(x + r    , y        , w - r - r, color); // Top
  lcdDrawHLine(x + r    , y + h - 1, w - r - r, color); // Bottom
  lcdDrawVLine(x        , y + r    , h - r - r, color); // Left
  lcdDrawVLine(x + w - 1, y + r    , h - r - r, color); // Right

  // draw four corners
  lcdDrawCircleHelper(x + r        , y + r        , r, 1, color);
  lcdDrawCircleHelper(x + w - r - 1, y + r        , r, 2, color);
  lcdDrawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4, color);
  lcdDrawCircleHelper(x + r        , y + h - r - 1, r, 8, color);
}

LCD_OPT_DEF void lcdDrawFillCircleHelper(int32_t x0, int32_t y0, int32_t r, uint8_t cornername, int32_t delta, uint32_t color)
{
  int32_t f     = 1 - r;
  int32_t ddF_x = 1;
  int32_t ddF_y = -r - r;
  int32_t y     = 0;

  delta++;

  while (y < r)
  {
    if (f >= 0)
    {
      r--;
      ddF_y += 2;
      f     += ddF_y;
    }

    y++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1)
    {
      lcdDrawHLine(x0 - r, y0 + y, r + r + delta, color);
      lcdDrawHLine(x0 - y, y0 + r, y + y + delta, color);
    }
    if (cornername & 0x2)
    {
      lcdDrawHLine(x0 - r, y0 - y, r + r + delta, color); // 11995, 1090
      lcdDrawHLine(x0 - y, y0 - r, y + y + delta, color);
    }
  }
}

LCD_OPT_DEF void lcdDrawFillRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint32_t color)
{
  // smarter version
  lcdDrawFillRect(x, y + r, w, h - r - r, color);

  // draw four corners
  lcdDrawFillCircleHelper(x + r, y + h - r - 1, r, 1, w - r - r - 1, color);
  lcdDrawFillCircleHelper(x + r, y + r        , r, 2, w - r - r - 1, color);
}

LCD_OPT_DEF void lcdDrawTriangle(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x3, int32_t y3, uint32_t color)
{
  lcdDrawLine(x1, y1, x2, y2, color);
  lcdDrawLine(x1, y1, x3, y3, color);
  lcdDrawLine(x2, y2, x3, y3, color);
}

/*
 * lcdDrawFillTriangle 은 걷어냈다.
 *
 * 이 보드에서 화면을 그리는 것은 TouchGFX 이고, 삼각형 채우기를 호출하는 곳이
 * 하나도 없다. 그런데 구현이 화면 긴 변만큼(480 픽셀 * 4 바이트) 라인 버퍼를
 * 정적으로 물고 있어서, RAM 이 64 KB 뿐인 L432KC 에서는 그 자체가 부담이다.
 * 다시 필요해지면 F411 쪽 lcd.c 에 원본이 남아 있다.
 */

uint32_t lcdGetFps(void)
{
  return fps_count;
}

uint32_t lcdGetFpsTime(void)
{
  return fps_time;
}

bool lcdDrawAvailable(void)
{
  return !lcd_request_draw;
}

bool lcdRequestDraw(void)
{
  if (lcd_request_draw == true)
  {
    return false;
  }

  lcd_request_draw = true;
  TransferDoneISR();

  return true;
}

void lcdUpdateDraw(void)
{
  lcdRequestDraw();
  while(lcdDrawAvailable() != true)
  {
    delay(1);
  }
}

bool lcdSetWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  uint32_t x_end;
  uint32_t y_end;

  if ((lcd.setWindow == NULL) || (w == 0U) || (h == 0U) ||
      (x >= LCD_WIDTH) || (y >= LCD_HEIGHT))
  {
    return false;
  }

  x_end = (uint32_t)x + (uint32_t)w - 1U;
  y_end = (uint32_t)y + (uint32_t)h - 1U;

  if ((x_end >= LCD_WIDTH) || (y_end >= LCD_HEIGHT))
  {
    return false;
  }

  return lcd.setWindow(x, y, (int32_t)x_end, (int32_t)y_end);
}

bool lcdSendBuffer(uint8_t *p_data, uint32_t length, uint32_t timeout_ms)
{
  if ((lcd.sendBuffer == NULL) || (p_data == NULL) || (length == 0U))
  {
    return false;
  }

  return lcd.sendBuffer(p_data, length, timeout_ms);
}

bool lcdDrawRectBuffer(int16_t x, int16_t y, int16_t w, int16_t h, lcd_render_func_t p_func, void *p_arg)
{
  int16_t y_pos;
  int16_t y_end;

  if ((p_func == NULL) || (lcd.setWindow == NULL) || (lcd.sendBuffer == NULL))
  {
    return false;
  }

  if (lcdClipRect(&x, &y, &w, &h) != true)
  {
    return false;
  }

  y_pos = y;
  y_end = y + h;

  while (y_pos < y_end)
  {
    int16_t strip_h = y_end - y_pos;
    uint32_t max_lines = LCD_STRIP_BUF_PIXELS / (uint32_t)w;
    uint32_t tx_len;

    if (max_lines == 0U)
    {
      return false;
    }

    if ((uint32_t)strip_h > max_lines)
    {
      strip_h = (int16_t)max_lines;
    }

    tx_len = (uint32_t)w * (uint32_t)strip_h;

    p_func(x, y_pos, w, strip_h, lcd_strip_buf, p_arg);

    if ((lcd.setWindow(x, y_pos, x + w - 1, y_pos + strip_h - 1) != true) ||
        (lcd.sendBuffer((uint8_t *)lcd_strip_buf, tx_len, 0) != true))
    {
      return false;
    }

    y_pos += strip_h;
  }

  return true;
}

uint16_t *lcdGetFrameBuffer(void)
{
  return NULL;
}

uint16_t *lcdGetCurrentFrameBuffer(void)
{
  return NULL;
}

void lcdDisplayOff(void)
{
}

void lcdDisplayOn(void)
{
  lcdSetBackLight(lcdGetBackLight());
}

int32_t lcdGetWidth(void)
{
  return LCD_WIDTH;
}

int32_t lcdGetHeight(void)
{
  return LCD_HEIGHT;
}


void lcdDrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);

  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 < 0) x1 = 0;
  if (y1 < 0) y1 = 0;


  if (steep)
  {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1)
  {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1)
  {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++)
  {
    if (steep)
    {
      lcdDrawPixel(y0, x0, color);
    } else
    {
      lcdDrawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0)
    {
      y0 += ystep;
      err += dx;
    }
  }
}

void lcdDrawVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
  lcdDrawFillRect(x, y, 1, h, color);
}

void lcdDrawHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
  lcdDrawFillRect(x, y, w, 1, color);
}

void lcdDrawFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  if (lcdClipRect(&x, &y, &w, &h) != true)
  {
    return;
  }

  if ((lcd.setWindow == NULL) || (lcd.sendBuffer == NULL))
  {
    return;
  }

  if (lcd.setWindow(x, y, x + w - 1, y + h - 1) != true)
  {
    return;
  }

  (void)lcdSendColorRepeat(color, (uint32_t)w * (uint32_t)h);
}

void lcdDrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  lcdDrawHLine(x, y, w, color);
  lcdDrawHLine(x, y+h-1, w, color);
  lcdDrawVLine(x, y, h, color);
  lcdDrawVLine(x+w-1, y, h, color);
}

void lcdDrawFillScreen(uint16_t color)
{
  lcdDrawFillRect(0, 0, HW_LCD_WIDTH, HW_LCD_HEIGHT, color);
}

static bool lcdClipRect(int16_t *x, int16_t *y, int16_t *w, int16_t *h)
{
  int32_t x0;
  int32_t y0;
  int32_t x1;
  int32_t y1;

  if ((*w <= 0) || (*h <= 0))
  {
    return false;
  }

  x0 = *x;
  y0 = *y;
  x1 = x0 + *w - 1;
  y1 = y0 + *h - 1;

  if ((x0 >= LCD_WIDTH) || (y0 >= LCD_HEIGHT) || (x1 < 0) || (y1 < 0))
  {
    return false;
  }

  if (x0 < 0)
  {
    x0 = 0;
  }

  if (y0 < 0)
  {
    y0 = 0;
  }

  if (x1 >= LCD_WIDTH)
  {
    x1 = LCD_WIDTH - 1;
  }

  if (y1 >= LCD_HEIGHT)
  {
    y1 = LCD_HEIGHT - 1;
  }

  *x = (int16_t)x0;
  *y = (int16_t)y0;
  *w = (int16_t)(x1 - x0 + 1);
  *h = (int16_t)(y1 - y0 + 1);

  return true;
}

static bool lcdSendColorRepeat(uint16_t color, uint32_t length)
{
  uint32_t init_len;

  if ((length == 0U) || (lcd.sendBuffer == NULL))
  {
    return false;
  }

  init_len = length;
  if (init_len > LCD_STRIP_BUF_PIXELS)
  {
    init_len = LCD_STRIP_BUF_PIXELS;
  }

  for (uint32_t i = 0; i < init_len; i++)
  {
    lcd_strip_buf[i] = color;
  }

  while (length > 0U)
  {
    uint32_t tx_len = length;

    if (tx_len > LCD_STRIP_BUF_PIXELS)
    {
      tx_len = LCD_STRIP_BUF_PIXELS;
    }

    if (lcd.sendBuffer((uint8_t *)lcd_strip_buf, tx_len, 0) != true)
    {
      return false;
    }
    length -= tx_len;
  }

  return true;
}

void lcdPrintf(int x, int y, uint16_t color,  const char *fmt, ...)
{
  va_list arg;
  int32_t len;
  char print_buffer[256];
  int Size_Char = 1;
  int i, x_Pre = x;
  han_font_t FontBuf;
  uint8_t font_width;

  if (fmt == NULL)
  {
    return;
  }

  va_start (arg, fmt);
  len = vsnprintf(print_buffer, sizeof(print_buffer), fmt, arg);
  va_end (arg);

  if (len < 0)
  {
    return;
  }

  if ((uint32_t)len >= sizeof(print_buffer))
  {
    len = sizeof(print_buffer) - 1U;
  }

  for( i=0; i<len; i+=Size_Char )
  {
    hanFontLoad( &print_buffer[i], &FontBuf );

    disHanFont( x, y, &FontBuf, color);

    Size_Char = FontBuf.Size_Char;
    if (Size_Char >= 2)
    {
      font_width = 16;
      x += 2*8;
    }
    else
    {
      font_width = 8;
      x += 1*8;
    }

    if ((x+font_width) > HW_LCD_WIDTH)
    {
      x  = x_Pre;
      y += 16;
    }

    if( FontBuf.Code_Type == PHAN_END_CODE ) break;
  }
}


uint32_t lcdGetStrWidth(const char *fmt, ...)
{
  va_list arg;
  int32_t len;
  char print_buffer[256];
  int Size_Char = 1;
  int i;
  han_font_t FontBuf;
  uint32_t str_len;

  if (fmt == NULL)
  {
    return 0;
  }

  va_start (arg, fmt);
  len = vsnprintf(print_buffer, sizeof(print_buffer), fmt, arg);
  va_end (arg);

  if (len < 0)
  {
    return 0;
  }

  if ((uint32_t)len >= sizeof(print_buffer))
  {
    len = sizeof(print_buffer) - 1U;
  }

  str_len = 0;

  for( i=0; i<len; i+=Size_Char )
  {
    hanFontLoad( &print_buffer[i], &FontBuf );

    Size_Char = FontBuf.Size_Char;

    str_len += (Size_Char * 8);

    if( FontBuf.Code_Type == PHAN_END_CODE ) break;
  }

  return str_len;
}

void disHanFont(int x, int y, han_font_t *FontPtr, uint16_t textcolor)
{
  uint16_t    i, j, Loop;
  uint16_t  FontSize = FontPtr->Size_Char;
  uint16_t index_x;

  if (FontSize > 2)
  {
    FontSize = 2;
  }

  for ( i = 0 ; i < 16 ; i++ )        // 16 Lines per Font/Char
  {
    index_x = 0;
    for ( j = 0 ; j < FontSize ; j++ )      // 16 x 16 (2 Bytes)
    {
      uint8_t font_data;

      font_data = FontPtr->FontBuffer[i*FontSize +j];

      for( Loop=0; Loop<8; Loop++ )
      {
        if( (font_data<<Loop) & (0x80))
        {
          lcdDrawPixel(x + index_x, y + i, textcolor);
        }
        index_x++;
      }
    }
  }
}





#ifdef _USE_HW_CLI
void cliLcd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "test") == true)
  {
    while(cliKeepLoop())
    {
      if (lcdDrawAvailable() == true)
      {
        lcdClearBuffer(black);

        lcdPrintf(25,16*0, green, "[LCD 테스트]");

        lcdPrintf(0,16*1, white, "%d fps", lcdGetFps());
        lcdPrintf(0,16*2, white, "%d ms" , lcdGetFpsTime());
        lcdPrintf(0,16*3, white, "%d ms" , millis());

        lcdDrawFillRect( 0, 70, 10, 10, red);
        lcdDrawFillRect(10, 70, 10, 10, green);
        lcdDrawFillRect(20, 70, 10, 10, blue);

        lcdRequestDraw();
      }
    }

    lcdClearBuffer(black);
    lcdUpdateDraw();

    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "reset") == true)
  {
    bool reset_ok;

    cliPrintf("lcd reset...\n");

    reset_ok = lcdReset();

    cliPrintf("  result    : %s\n", (reset_ok == true) ? "OK" : "FAIL");
    cliPrintf("  is_init   : %s\n", (lcdIsInit() == true) ? "true" : "false");
    cliPrintf("  backlight : %d\n", (int)lcdGetBackLight());

    if (reset_ok == true)
    {
#ifdef _USE_HW_TOUCHGFX
      /* 리셋으로 비워진 GRAM 을 채우려면 화면 전체를 한 번 다시 그려야 한다. */
      touchgfxRequestFullRedraw();
      cliPrintf("  redraw    : requested\n");
#else
      lcdClearBuffer(black);
      lcdUpdateDraw();
#endif
    }

    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("lcd test\n");
    cliPrintf("lcd reset\n");
  }
}
#endif



#endif
