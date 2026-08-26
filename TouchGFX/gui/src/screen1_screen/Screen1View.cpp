#include <gui/screen1_screen/Screen1View.hpp>
#include <touchgfx/Color.hpp>
#include <touchgfx/canvas_widget_renderer/CanvasWidgetRenderer.hpp>
#include <images/BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>

namespace
{
/*
 * 주간 야외 주행용 밝은 테마.
 *
 * ILI9488 은 transmissive TFT 이고 이 보드의 백라이트는 GPIO on/off 라 이미 최대
 * 밝기다. 즉 직사광 아래에서 소프트웨어가 쓸 수 있는 수단은 색 조합뿐이다.
 *
 * 밝은 바탕이 유리한 이유는 대비비가 아니다(반사광은 글자와 배경에 똑같이
 * 더해져서 비율은 거의 그대로다). 실제로 갈리는 것은
 *   - 어두운 화면은 거울처럼 하늘/헬멧을 비춰 그 상이 글자를 덮는다
 *   - 밝은 곳에 순응한 눈은 동공이 수축해 어두운 계조를 읽지 못한다
 *   - 어두운 바탕 위 밝은 글자는 번져서(halation) 획이 뭉개진다
 * 이 세 가지이고, 전부 밝은 바탕에서 사라진다.
 *
 * 정보가 실린 타일을 화면에서 가장 밝게 두고, 바탕은 한 단계 낮춰 타일이
 * 떠 보이게 한다.
 */
/*
 * 회색 축은 중립이 아니라 따뜻한 쪽으로 조금 틀어 둔다.
 *
 * 흔히 쓰는 UI 회색(#111827 / #6B7280 / #D1D5DB 같은 것)은 파랑이 20 남짓 높은
 * 차가운 회색이다. 화면에서 그 자체로 나쁘지는 않은데, 웹에서 워낙 널리 쓰여
 * 계기판에 얹으면 계기판이 아니라 웹 대시보드처럼 읽힌다.
 *
 * 여기서는 R 을 B 보다 두어 계조 높게 잡아 종이나 계측기 눈금판 쪽 회색으로
 * 옮겼다. 차이는 미세하지만 화면 전체에 같은 방향으로 걸리므로 인상이 갈린다.
 */
const uint8_t BG_R = 255,     BG_G = 255,     BG_B = 255;  /* #FFFFFF */
const uint8_t PANEL_R = 255,  PANEL_G = 255,  PANEL_B = 255;  /* = BG */
const uint8_t BORDER_R = 219, BORDER_G = 219, BORDER_B = 215;  /* #DBDBD7 */

/* 밝은 바탕에서는 '꺼짐'도 눈에 보여야 눈금이 성립한다. 배경과 붙어 버리면
 * 시프트 바는 몇 칸이 남았는지, 링은 어디까지 차야 하는지를 알 수 없다.
 * 트랙과 꺼진 세그먼트는 같은 값을 쓴다 — 둘 다 '아직 도달하지 않은 눈금'이라
 * 서로 다른 회색으로 두면 화면에 이유 없는 계조가 하나 늘어난다. */
const uint8_t TRACK_R = 203,  TRACK_G = 203,  TRACK_B = 198;  /* #CBCBC6 */
const uint8_t SEGOFF_R = 203, SEGOFF_G = 203, SEGOFF_B = 198;  /* #CBCBC6 */

/*
 * 글자 색은 두 단계뿐이다. 값은 FG, 그 값이 무엇인지 말해 주는 라벨과 단위는 DIM.
 * 색으로 정보의 종류를 구분하지 않는다 — 색은 상태를 말할 때만 쓴다.
 *
 * 상태색도 순색과 형광을 피한다. 채도를 낮추고 명도로 배경과 벌린 값이라
 * 셋 다 흰 바탕에서 같은 무게로 읽히고, 위에 흰 글자를 얹어도 버틴다.
 */
const uint8_t FG_R = 27,      FG_G = 27,      FG_B = 25;   /* #1B1B19 */
const uint8_t DIM_R = 119,    DIM_G = 118,    DIM_B = 111;  /* #77766F */
const uint8_t OK_R = 23,      OK_G = 121,     OK_B = 90;   /* #17795A */
const uint8_t WARN_R = 194,   WARN_G = 118,   WARN_B = 26;  /* #C2761A */
const uint8_t FAULT_R = 179,  FAULT_G = 63,   FAULT_B = 56;  /* #B33F38 */

/* 경고/고장 칸은 면째 물들이고 글자를 뒤집는다. 밝은 환경에서는 작은 글자의
 * 색보다 큰 면이 훨씬 빨리 읽히고, 햇빛에 화면이 씻겨도 면은 끝까지 남는다. */
const uint8_t ONFILL_R = 255, ONFILL_G = 255, ONFILL_B = 255;

const uint8_t APPS_VALID_FLAG = 1U << 0;

const uint32_t TLM_SOC = 1UL << 2;
const uint32_t TLM_CELL_TEMP = 1UL << 3;
const uint32_t TLM_RPM = 1UL << 5;
const uint32_t TLM_MOTOR_TEMP = 1UL << 8;
const uint32_t TLM_IGBT_TEMP = 1UL << 9;
const uint32_t TLM_CELL_MIN_V = 1UL << 16;

const int32_t RPM_MAX = 6500;
/* 숫자에 색이 붙는 유일한 구간. 그 아래 단계 구분은 시프트 바가 맡는다. */
const int32_t RPM_RED_MIN = 4500;

const int32_t SHIFT_AMBER_PM = 700;
const int32_t SHIFT_RED_PM = 860;
const int32_t SHIFT_BLINK_PM = 960;

const int16_t CELL_WARN_DECIC = 480;
const int16_t CELL_CRIT_DECIC = 550;
const int16_t IGBT_WARN_DECIC = 500;
const int16_t IGBT_CRIT_DECIC = 600;
/* 모터 권선은 인버터보다 높은 온도를 견딘다. --- 팀에서 확정해야 하는 값 ---
 * 모터 사양서의 연속/최대 권선 온도로 반드시 대조할 것. */
const int16_t MOTOR_WARN_DECIC = 800;   /* 80.0 C */
const int16_t MOTOR_CRIT_DECIC = 1000;  /* 100.0 C */
const int32_t SOC_WARN_PM = 300;
const int32_t SOC_CRIT_PM = 150;

/* Shift-light geometry.
 * 이 막대는 시선이 코너에 가 있는 동안 주변시로만 읽는다. 주변시는 해상도가
 * 낮은 대신 면적과 밝기에 반응하므로, 세그먼트를 키울수록 그대로 인식이 빨라진다. */
const int16_t SHIFT_HOUSING_H = 36;
const int16_t SEG_X0 = 10;
const int16_t SEG_Y = 6;
const int16_t SEG_W = 25;
const int16_t SEG_H = 23;
const int16_t SEG_PITCH = 29;

/* Keep the source's full RPM resolution. The previous 10 rpm quantization
 * made otherwise smooth input look artificially stepped. */
const int32_t RPM_DISPLAY_STEP = 1;

/*
 * 링 각도 눈금.
 *
 * 링은 지름 144 px 에 270도라 1도가 채 1 px 이 안 된다. 그런데 setValue 로 각도가
 * 조금이라도 움직이면 그 구간을 감싸는 사각형이 통째로 다시 그려지고, 그 사각형
 * 안에 있는 트랙과 진행 링을 CanvasWidget 으로 다시 계산해야 한다. 눈에 보이지도
 * 않는 변화에 그 비용을 내지 않도록, 링에 넣는 값은 미리 눈금으로 끊는다.
 *
 * 숫자는 그대로 1 % / 1 km/h 로 움직이므로 표시 해상도는 떨어지지 않는다.
 */
const int32_t CELL_MIN_ARC_MIN_MV = 2500;
const int32_t CELL_MIN_ARC_MAX_MV = 4300;
const int32_t CELL_MIN_ARC_STEP_MV = 10;

/* Two compact auxiliary gauges flank the central RPM module. */
const int16_t GAUGE_BOX = 168;
const int16_t GAUGE_Y = 42;
const int16_t GAUGE_LEFT_X = 5;
const int16_t GAUGE_RIGHT_X = 307;
const int16_t GAUGE_CENTER = GAUGE_BOX / 2;
const int16_t GAUGE_RADIUS = 72;
/* 링은 값을 읽는 눈금이지 장식이 아니다. 지름 144 에 10 px 은 테두리처럼 보여
 * 가운데 숫자와 무게를 다툰다. 8 px 이면 눈금 구실을 하면서 숫자를 방해하지 않는다. */
const int16_t GAUGE_LINE_WIDTH = 8;
const int16_t ARC_START = -135;
const int16_t ARC_END = 135;

/* Horizontal battery geometry inside the former left circular gauge area. */
const int16_t BATTERY_BODY_X = 19;
const int16_t BATTERY_BODY_Y = 76;
const int16_t BATTERY_BODY_W = 130;
const int16_t BATTERY_BODY_H = 68;
/*
 * 외곽선은 윤곽만 알려 주면 된다.
 *
 * 두꺼운 선과 큼직한 돌기는 그 자체가 그림이 되어, 잔량을 읽기 전에 '배터리
 * 아이콘'이 먼저 눈에 들어온다. 선을 1 px 로 낮추고 돌기도 줄여 껍데기를
 * 뒤로 물리면, 남는 것은 초록 면이 어디까지 차 있느냐 하나다.
 */
const int16_t BATTERY_BORDER = 1;
/* 외곽선 안쪽으로 3 px 씩 띄운 값. */
const int16_t BATTERY_FILL_X = 23;
const int16_t BATTERY_FILL_Y = 80;
const int16_t BATTERY_FILL_W = 122;
const int16_t BATTERY_FILL_H = 60;
const int16_t BATTERY_TERMINAL_X = 149;
const int16_t BATTERY_TERMINAL_Y = 99;
const int16_t BATTERY_TERMINAL_W = 8;
const int16_t BATTERY_TERMINAL_H = 22;

/*
 * 온도 세 자리의 배치.
 *
 * 한 줄에 이니셜 / 값 / 단위가 나란히 선다. M I C 는 어느 온도인지만 짚어 주면
 * 되는 표시라 한 글자로 충분하고, 그 덕에 값이 한 줄을 통째로 쓴다.
 * 세 자리가 규칙이 같아서 왼쪽 것의 좌표만 두고 나머지는 TILE_PITCH 로 민다.
 */
const int16_t TILE_Y = 236;
const int16_t TILE_W = 146;
const int16_t TILE_H = 76;
const int16_t TILE_X0 = 8;
const int16_t TILE_PITCH = 159;
const int16_t TILE_LABEL_DX = 6;
const int16_t TILE_LABEL_DY = 13;
const int16_t TILE_LABEL_W = 24;
const int16_t TILE_LABEL_H = 40;
const int16_t TILE_VALUE_DX = 30;
const int16_t TILE_VALUE_DY = 13;
const int16_t TILE_VALUE_W = 86;
const int16_t TILE_VALUE_H = 40;
const int16_t TILE_UNIT_DX = 114;
const int16_t TILE_UNIT_DY = 21;
const int16_t TILE_UNIT_W = 32;
const int16_t TILE_UNIT_H = 36;

/* 모서리 조각 한 변. corners.py 가 구운 그림 크기와 반드시 같아야 한다. */
const int16_t CORNER_R = 10;

/* 상태별 조각 묶음. 순서는 tl, tr, bl, br 로 고정한다. */
const uint16_t CORNER_WHITE[4] = {
    BITMAP_CORNER_WHITE_TL_ID, BITMAP_CORNER_WHITE_TR_ID,
    BITMAP_CORNER_WHITE_BL_ID, BITMAP_CORNER_WHITE_BR_ID
};
const uint16_t CORNER_WARN[4] = {
    BITMAP_CORNER_WARN_TL_ID, BITMAP_CORNER_WARN_TR_ID,
    BITMAP_CORNER_WARN_BL_ID, BITMAP_CORNER_WARN_BR_ID
};
const uint16_t CORNER_CRIT[4] = {
    BITMAP_CORNER_CRIT_TL_ID, BITMAP_CORNER_CRIT_TR_ID,
    BITMAP_CORNER_CRIT_BL_ID, BITMAP_CORNER_CRIT_BR_ID
};

touchgfx::colortype rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return touchgfx::Color::getColorFromRGB(r, g, b);
}

/* 네 귀에 조각을 놓는다. 패널과 같은 사각형을 받아 모서리 좌표를 계산한다. */
void placeCorners(touchgfx::Image* corners,
                  int16_t x, int16_t y, int16_t w, int16_t h,
                  const uint16_t* ids)
{
    const int16_t right = static_cast<int16_t>(x + w - CORNER_R);
    const int16_t bottom = static_cast<int16_t>(y + h - CORNER_R);
    const int16_t px[4] = { x, right, x, right };
    const int16_t py[4] = { y, y, bottom, bottom };

    for (uint8_t i = 0U; i < 4U; i++)
    {
        corners[i].setBitmap(touchgfx::Bitmap(ids[i]));
        corners[i].setXY(px[i], py[i]);
    }
}

/* 칸이 물들 때 조각도 같이 갈아 끼운다. */
void swapCorners(touchgfx::Image* corners, const uint16_t* ids)
{
    for (uint8_t i = 0U; i < 4U; i++)
    {
        corners[i].setBitmap(touchgfx::Bitmap(ids[i]));
    }
}

int32_t clampValue(int32_t value, int32_t low, int32_t high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

void placeBox(touchgfx::Box& box,
              int16_t x, int16_t y, int16_t w, int16_t h,
              uint8_t r, uint8_t g, uint8_t b)
{
    box.setPosition(x, y, w, h);
    box.setColor(rgb(r, g, b));
}

/*
 * 값이 놓이는 면.
 *
 * 테두리도 그리지 않고 색도 바탕과 같다. 평상시에는 보이지 않는 자리이며,
 * 온도 칸이 물들 때만 면으로 드러난다.
 */
void placePanel(touchgfx::BoxWithBorder& panel,
                int16_t x, int16_t y, int16_t w, int16_t h)
{
    panel.setPosition(x, y, w, h);
    panel.setColor(rgb(PANEL_R, PANEL_G, PANEL_B));
    panel.setBorderSize(0U);
}

void placeLabel(touchgfx::TextArea& text,
                int16_t x, int16_t y, int16_t w, int16_t h,
                TEXTS id)
{
    text.setPosition(x, y, w, h);
    text.setColor(rgb(DIM_R, DIM_G, DIM_B));
    text.setTypedText(touchgfx::TypedText(id));
}

void placeValue(touchgfx::TextAreaWithOneWildcard& text,
                touchgfx::Unicode::UnicodeChar* buffer,
                int16_t x, int16_t y, int16_t w, int16_t h,
                TEXTS id)
{
    text.setPosition(x, y, w, h);
    text.setColor(rgb(FG_R, FG_G, FG_B));
    text.setTypedText(touchgfx::TypedText(id));
    text.setWildcard(buffer);
}

void placeTrack(touchgfx::Circle& circle,
                touchgfx::PainterRGB565& painter,
                int16_t x, int16_t y)
{
    circle.setPosition(x, y, GAUGE_BOX, GAUGE_BOX);
    circle.setCenter(GAUGE_CENTER, GAUGE_CENTER);
    circle.setRadius(GAUGE_RADIUS);
    circle.setLineWidth(GAUGE_LINE_WIDTH);
    circle.setArc(ARC_START, ARC_END);
    painter.setColor(rgb(TRACK_R, TRACK_G, TRACK_B));
    circle.setPainter(painter);
}

void placeArc(touchgfx::CircleProgress& arc,
              touchgfx::PainterRGB565& painter,
              int16_t x, int16_t y, int32_t rangeMin, int32_t rangeMax)
{
    arc.setXY(x, y);
    arc.setProgressIndicatorPosition(0, 0, GAUGE_BOX, GAUGE_BOX);
    arc.setCenter(GAUGE_CENTER, GAUGE_CENTER);
    arc.setRadius(GAUGE_RADIUS);
    arc.setLineWidth(GAUGE_LINE_WIDTH);
    arc.setStartEndAngle(ARC_START, ARC_END);
    arc.setCapPrecision(180);
    arc.setRange(static_cast<int>(rangeMin), static_cast<int>(rangeMax));
    arc.setPainter(painter);
    arc.setValue(static_cast<int>(rangeMin));
}

/*
 * 온도 한 자리를 배치한다.
 *
 * 이니셜은 값과 같은 40 px 이되 색이 한 단계 흐리다. 크기를 줄이지 않는 것은
 * M I C 가 값과 한 줄에 서 있기 때문이다 — 같은 줄에서 크기까지 달라지면
 * 줄이 울퉁불퉁해 보인다. 위계는 크기가 아니라 색이 만든다.
 */
void placeTempTile(touchgfx::BoxWithBorder& panel,
                   touchgfx::Image* corners,
                   touchgfx::TextArea& label,
                   touchgfx::TextAreaWithOneWildcard& value,
                   touchgfx::Unicode::UnicodeChar* buffer,
                   touchgfx::TextArea& unit,
                   int16_t x,
                   TEXTS labelId)
{
    placePanel(panel, x, TILE_Y, TILE_W, TILE_H);
    /* 평상시 칸은 바탕과 같은 흰 면이라, 선이 없으면 어디까지가 한 칸인지
     * 보이지 않는다. 직선 구간은 여기서, 네 귀는 모서리 그림이 맡는다. */
    panel.setBorderColor(rgb(BORDER_R, BORDER_G, BORDER_B));
    panel.setBorderSize(1U);
    placeCorners(corners, x, TILE_Y, TILE_W, TILE_H, CORNER_WHITE);
    placeLabel(label,
               static_cast<int16_t>(x + TILE_LABEL_DX),
               static_cast<int16_t>(TILE_Y + TILE_LABEL_DY),
               TILE_LABEL_W, TILE_LABEL_H, labelId);
    placeValue(value, buffer,
               static_cast<int16_t>(x + TILE_VALUE_DX),
               static_cast<int16_t>(TILE_Y + TILE_VALUE_DY),
               TILE_VALUE_W, TILE_VALUE_H, T_T_TEMPVALUE);
    /* 단위는 값보다 한 단계 작고(40 -> 30) 한 단계 흐리다. y 를 값보다 8 px
     * 내린 것은 두 폰트 크기 차이만큼 밑선을 맞추기 위함이다. */
    placeLabel(unit,
               static_cast<int16_t>(x + TILE_UNIT_DX),
               static_cast<int16_t>(TILE_Y + TILE_UNIT_DY),
               TILE_UNIT_W, TILE_UNIT_H, T_T_DEGC);
}

/*
 * 0.1 도 단위 값을 정수로 반올림한다.
 *
 * 온도 칸에는 소수점을 띄우지 않는다. 값은 가운데 정렬이라 자리수가 늘면
 * 양쪽으로 번지는데, 세 자리에 소수점까지 붙으면(100.5) 왼쪽으로는 이니셜을,
 * 오른쪽으로는 단위를 밟는다. 100 도짜리 온도에서 0.1 도는 읽는 사람에게
 * 아무 뜻도 없으므로, 자리를 늘리는 대신 반올림해서 버린다.
 */
int32_t roundDeci(int32_t deci)
{
    return (deci >= 0) ? ((deci + 5) / 10) : ((deci - 5) / 10);
}

void formatWhole(touchgfx::Unicode::UnicodeChar* buffer,
                 uint16_t size, int32_t whole)
{
    touchgfx::Unicode::snprintf(buffer, size, "%d", static_cast<int>(whole));
}

void formatPercent(touchgfx::Unicode::UnicodeChar* buffer,
                   uint16_t size, int32_t percent)
{
    touchgfx::Unicode::snprintf(buffer, size, "%d", static_cast<int>(percent));

    uint16_t length = touchgfx::Unicode::strlen(buffer);
    if ((length + 1U) < size)
    {
        buffer[length] = static_cast<touchgfx::Unicode::UnicodeChar>('%');
        buffer[length + 1U] = 0;
    }
}

void formatMilliVolts(touchgfx::Unicode::UnicodeChar* buffer,
                      uint16_t size, uint16_t millivolts)
{
    touchgfx::Unicode::snprintf(buffer, size, "%d.%03d",
                                static_cast<int>(millivolts / 1000U),
                                static_cast<int>(millivolts % 1000U));
}

void formatUnknown(touchgfx::Unicode::UnicodeChar* buffer, uint16_t size)
{
    touchgfx::Unicode::snprintf(buffer, size, "--");
}
}

bool Screen1View::fieldChanged(DrawnField& field,
                               int32_t value,
                               bool valid,
                               touchgfx::colortype color)
{
    /* 값이 없을 때(자리표시자)는 숫자를 비교하지 않는다. '--' 를 띄우는 동안
     * 뒤에서 값이 계속 흔들려도 화면은 그대로여야 한다. */
    const bool same = field.drawn &&
                      (field.valid == valid) &&
                      (field.color == color) &&
                      ((valid != true) || (field.value == value));

    if (same)
    {
        return false;
    }

    field.value = value;
    field.valid = valid;
    field.color = color;
    field.drawn = true;
    return true;
}

Screen1View::Screen1View()
    : appsFaultActive(false),
      linkConnected(false),
      socFillColor(0),
      shiftBlinkActive(false),
      shiftBlinkOn(true),
      blinkTick(0U),
      lastRpm(0),
      lastRpmValid(false),
      rpmDrawn(),
      socDrawn(),
      minCellDrawn(),
      motorDrawn(),
      igbtDrawn(),
      cellDrawn(),
      linkStateDrawn(false),
      drawnConnected(false),
      drawnFaultActive(false)
{
    /* ViewBase 생성자가 잡아 둔 4800 바이트 버퍼를 이걸로 교체한다.
     * ViewBase 가 먼저 생성되므로 여기서 덮어쓰는 순서가 맞다. */
    touchgfx::CanvasWidgetRenderer::setupBuffer(canvasBuffer,
                                                CANVAS_BUFFER_SIZE);

    formatUnknown(rpmBuf, 10U);
    formatUnknown(socBuf, 10U);
    formatUnknown(minCellBuf, 10U);
    formatUnknown(motorBuf, 10U);
    formatUnknown(igbtBuf, 10U);
    formatUnknown(cellBuf, 10U);

    placeBox(screenBg, 0, 0, 480, 320, BG_R, BG_G, BG_B);
    add(screenBg);

    placePanel(shiftHousing, 0, 0, 480, SHIFT_HOUSING_H);
    add(shiftHousing);

    for (uint8_t i = 0U; i < SHIFT_SEGMENTS; i++)
    {
        shiftColor[i] = rgb(SEGOFF_R, SEGOFF_G, SEGOFF_B);
        placeBox(shiftSeg[i],
                 static_cast<int16_t>(SEG_X0 + (i * SEG_PITCH)),
                 SEG_Y, SEG_W, SEG_H,
                 SEGOFF_R, SEGOFF_G, SEGOFF_B);
        add(shiftSeg[i]);
    }

    /* Battery gauge: static case/terminal with a left-to-right SOC fill. */
    placeBox(socBatteryTerminal,
             BATTERY_TERMINAL_X, BATTERY_TERMINAL_Y,
             BATTERY_TERMINAL_W, BATTERY_TERMINAL_H,
             FG_R, FG_G, FG_B);
    add(socBatteryTerminal);

    socBatteryBody.setPosition(BATTERY_BODY_X, BATTERY_BODY_Y,
                               BATTERY_BODY_W, BATTERY_BODY_H);
    /* 케이스 안쪽은 흰색이 아니라 꺼진 눈금 색이다. 흰색으로 두면 잔량이
     * 어디까지 차 있는지는 보여도 '얼마가 비었는지'가 배경과 섞여 안 보인다. */
    socBatteryBody.setColor(rgb(TRACK_R, TRACK_G, TRACK_B));
    socBatteryBody.setBorderColor(rgb(FG_R, FG_G, FG_B));
    socBatteryBody.setBorderSize(BATTERY_BORDER);
    add(socBatteryBody);

    socBatteryFill.setXY(BATTERY_FILL_X, BATTERY_FILL_Y);
    socBatteryFill.setProgressIndicatorPosition(0, 0,
                                                BATTERY_FILL_W,
                                                BATTERY_FILL_H);
    socBatteryFill.setRange(0, 1000);
    socBatteryFill.setDirection(touchgfx::AbstractDirectionProgress::RIGHT);
    socFillColor = rgb(OK_R, OK_G, OK_B);
    socBatteryFill.setColor(socFillColor);
    socBatteryFill.setValue(0);
    add(socBatteryFill);

    placeLabel(socLabel, GAUGE_LEFT_X, 49, GAUGE_BOX, 20, T_T_SOCLABEL);
    add(socLabel);
    /* 숫자는 흰색으로 두고 잔량 상태는 채움 색으로 말한다. 숫자까지 색을 입히면
     * 화면에서 RPM 보다 잔량이 먼저 눈에 들어온다. */
    placeValue(socValue, socBuf, GAUGE_LEFT_X, 142, GAUGE_BOX, 80, T_T_SOC);
    add(socValue);

    /* Minimum cell voltage replaces road speed. */
    placeTrack(minCellTrack, minCellTrackPainter, GAUGE_RIGHT_X, GAUGE_Y);
    add(minCellTrack);
    placeArc(minCellArc, minCellArcPainter, GAUGE_RIGHT_X, GAUGE_Y,
             CELL_MIN_ARC_MIN_MV, CELL_MIN_ARC_MAX_MV);
    minCellArcPainter.setColor(rgb(OK_R, OK_G, OK_B));
    add(minCellArc);

    placeLabel(minCellLabel, GAUGE_RIGHT_X, 80, GAUGE_BOX, 20,
               T_T_MINCELLLABEL);
    add(minCellLabel);
    placeValue(minCellValue, minCellBuf, GAUGE_RIGHT_X, 101, GAUGE_BOX, 64,
               T_T_MINCELLVALUE);
    add(minCellValue);
    placeLabel(minCellUnit, GAUGE_RIGHT_X, 164, GAUGE_BOX, 20,
               T_T_MINCELLUNIT);
    add(minCellUnit);

    /* Central RPM module replaces a decorative third gauge. */
    placePanel(centerPanel, 176, 44, 128, 178);
    add(centerPanel);
    placeLabel(rpmLabel, 176, 52, 128, 20, T_T_RPMUNIT);
    add(rpmLabel);
    placeValue(rpmText, rpmBuf, 176, 70, 128, 92, T_T_RPM);
    add(rpmText);

    /*
     * 차량 상태 표시.
     *
     * 테두리를 두르고 색을 채우면 그 순간 버튼처럼 읽힌다 — 눌러서 뭔가 되는
     * 물건으로 보이면 안 된다. 배지에서 테두리와 밑줄을 걷어내고 왼쪽 표시등
     * 하나 + 글자만 남겼다. 패널은 글자 뒤 바탕 역할만 하므로 가운데 패널과
     * 같은 흰색이고, 실제로는 보이지 않는다.
     */
    placePanel(statusPanel, 184, 178, 112, 32);
    add(statusPanel);
    placeBox(statusAccent, 188, 190, 8, 8, WARN_R, WARN_G, WARN_B);
    add(statusAccent);
    placeLabel(statusText, 200, 182, 92, 24, T_T_DATALOST);
    add(statusText);

    /*
     * Bottom temperature row.
     *
     * 이니셜과 단위는 값이 바뀌어도 그대로라, 숫자만 달라진 프레임에서는 값
     * 사각형만 전송된다.
     */
    placeTempTile(motorPanel, panelCorner[PANEL_MOTOR], motorTempLabel, motorValue, motorBuf, motorUnit,
                  TILE_X0, T_T_MOTORINITIAL);
    add(motorPanel);
    for (uint8_t i = 0U; i < 4U; i++)
    {
        add(panelCorner[PANEL_MOTOR][i]);
    }
    add(motorTempLabel);
    add(motorValue);
    add(motorUnit);

    placeTempTile(igbtPanel, panelCorner[PANEL_IGBT], igbtTempLabel, igbtValue, igbtBuf, igbtUnit,
                  static_cast<int16_t>(TILE_X0 + TILE_PITCH),
                  T_T_INVERTERINITIAL);
    add(igbtPanel);
    for (uint8_t i = 0U; i < 4U; i++)
    {
        add(panelCorner[PANEL_IGBT][i]);
    }
    add(igbtTempLabel);
    add(igbtValue);
    add(igbtUnit);

    placeTempTile(cellPanel, panelCorner[PANEL_CELL], cellTempLabel, cellValue, cellBuf, cellUnit,
                  static_cast<int16_t>(TILE_X0 + (2 * TILE_PITCH)),
                  T_T_CELLINITIAL);
    add(cellPanel);
    for (uint8_t i = 0U; i < 4U; i++)
    {
        add(panelCorner[PANEL_CELL][i]);
    }
    add(cellTempLabel);
    add(cellValue);
    add(cellUnit);
}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();

    /* The legacy Designer composition is kept in the project, but hidden. */
    Screen1ViewBase::__background.setVisible(false);
    Screen1ViewBase::background.setVisible(false);
    Screen1ViewBase::topDivider.setVisible(false);
    Screen1ViewBase::socTrack.setVisible(false);
    Screen1ViewBase::socFill.setVisible(false);
    Screen1ViewBase::minCellTrack.setVisible(false);
    Screen1ViewBase::minCellFill.setVisible(false);
    Screen1ViewBase::motorTrack.setVisible(false);
    Screen1ViewBase::motorFill.setVisible(false);
    Screen1ViewBase::igbtTrack.setVisible(false);
    Screen1ViewBase::igbtFill.setVisible(false);
    Screen1ViewBase::phaseTrack.setVisible(false);
    Screen1ViewBase::phaseFill.setVisible(false);
    Screen1ViewBase::rpmRing.setVisible(false);
    Screen1ViewBase::statusDot.setVisible(false);
    Screen1ViewBase::statusText.setVisible(false);
    Screen1ViewBase::socText.setVisible(false);
    Screen1ViewBase::rpmValue.setVisible(false);
    Screen1ViewBase::rpmUnit.setVisible(false);
    Screen1ViewBase::minCellLabel.setVisible(false);
    Screen1ViewBase::minCellValue.setVisible(false);
    Screen1ViewBase::minCellUnit.setVisible(false);
    Screen1ViewBase::motorLabel.setVisible(false);
    Screen1ViewBase::motorVal.setVisible(false);
    Screen1ViewBase::igbtLabel.setVisible(false);
    Screen1ViewBase::igbtVal.setVisible(false);
    Screen1ViewBase::torqueLabel.setVisible(false);
    Screen1ViewBase::torqueVal.setVisible(false);

    updateApps(false, 0U, 0U, 0U, 0U, 0U, 0U, 0, 0U, 0U);
    updateTelemetry(false, 0U, 0, 0U, 0U, 0, 0U, 0, 0, 0);
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}

void Screen1View::applyShiftLights(int32_t rpm, bool valid)
{
    int32_t perMille;
    uint8_t lit;

    rpm = clampValue(rpm, 0, RPM_MAX);
    perMille = (RPM_MAX > 0) ? ((rpm * 1000) / RPM_MAX) : 0;

    if (!valid)
    {
        perMille = 0;
    }

    lit = static_cast<uint8_t>((perMille * SHIFT_SEGMENTS) / 1000);
    if (lit > SHIFT_SEGMENTS)
    {
        lit = SHIFT_SEGMENTS;
    }

    shiftBlinkActive = valid && (perMille >= SHIFT_BLINK_PM);
    if (!shiftBlinkActive)
    {
        shiftBlinkOn = true;
        blinkTick = 0U;
    }

    for (uint8_t i = 0U; i < SHIFT_SEGMENTS; i++)
    {
        const int32_t segmentPm = ((i + 1) * 1000) / SHIFT_SEGMENTS;
        touchgfx::colortype color;

        if ((i >= lit) || (shiftBlinkActive && !shiftBlinkOn))
        {
            color = rgb(SEGOFF_R, SEGOFF_G, SEGOFF_B);
        }
        else if (segmentPm >= SHIFT_RED_PM)
        {
            color = rgb(FAULT_R, FAULT_G, FAULT_B);
        }
        else if (segmentPm >= SHIFT_AMBER_PM)
        {
            color = rgb(WARN_R, WARN_G, WARN_B);
        }
        else
        {
            color = rgb(OK_R, OK_G, OK_B);
        }

        if (color != shiftColor[i])
        {
            shiftColor[i] = color;
            shiftSeg[i].setColor(color);
            shiftSeg[i].invalidate();
        }
    }
}

void Screen1View::handleTickEvent()
{
    if (!shiftBlinkActive)
    {
        return;
    }

    blinkTick++;
    if (blinkTick >= 5U)
    {
        blinkTick = 0U;
        shiftBlinkOn = !shiftBlinkOn;
        applyShiftLights(lastRpm, lastRpmValid);
    }
}

/*
 * 온도 타일 갱신.
 *
 * 임계를 넘으면 숫자 색만 바꾸는 대신 타일 배경째 채우고 글자를 흰색으로
 * 뒤집는다. 밝은 바탕에서는 작은 글자의 색보다 큰 면적이 훨씬 빨리 읽히고,
 * 햇빛에 화면이 씻겨도 면은 끝까지 남는다.
 *
 * 배경이 바뀔 때만 타일 전체를 다시 그린다. 숫자만 달라진 프레임에서 타일을
 * 통째로 갱신하면 그게 그대로 프레임 시간이 된다.
 */
void Screen1View::updateTempTile(DrawnField& drawn,
                                 touchgfx::BoxWithBorder& panel,
                                 touchgfx::Image* corners,
                                 touchgfx::TextArea& initial,
                                 touchgfx::TextAreaWithOneWildcard& value,
                                 touchgfx::TextArea& unit,
                                 touchgfx::Unicode::UnicodeChar* buffer,
                                 int16_t decic,
                                 bool valid,
                                 int16_t warnDecic,
                                 int16_t critDecic)
{
    /* 화면에 실제로 뜨는 값. 임계 비교는 0.1 도 단위 그대로 두고, 다시 그릴지
     * 판단하는 비교만 이 값으로 한다. 그래야 표시가 그대로인 0.1 도 흔들림에
     * 칸을 다시 그리지 않는다. */
    const int32_t whole = roundDeci(decic);

    touchgfx::colortype fill;  /* 칸 바탕 */
    touchgfx::colortype ink;   /* 숫자 */
    touchgfx::colortype trim;  /* 이니셜과 단위 */
    const uint16_t* cornerIds; /* 바탕과 같은 색으로 구워 둔 모서리 조각 */
    bool fillChanged;

    if (valid != true)
    {
        fill = rgb(PANEL_R, PANEL_G, PANEL_B);
        ink = rgb(DIM_R, DIM_G, DIM_B);
        trim = ink;
        cornerIds = CORNER_WHITE;
    }
    else if (decic >= critDecic)
    {
        fill = rgb(FAULT_R, FAULT_G, FAULT_B);
        ink = rgb(ONFILL_R, ONFILL_G, ONFILL_B);
        trim = ink;
        cornerIds = CORNER_CRIT;
    }
    else if (decic >= warnDecic)
    {
        fill = rgb(WARN_R, WARN_G, WARN_B);
        ink = rgb(ONFILL_R, ONFILL_G, ONFILL_B);
        trim = ink;
        cornerIds = CORNER_WARN;
    }
    else
    {
        fill = rgb(PANEL_R, PANEL_G, PANEL_B);
        ink = rgb(FG_R, FG_G, FG_B);
        /* 평상시에만 이니셜과 단위를 흐리게 둔다. 칸이 물든 뒤에는 그 위에서
         * 회색이 탁해질 뿐이라 값과 같은 흰색으로 맞춘다. */
        trim = rgb(DIM_R, DIM_G, DIM_B);
        cornerIds = CORNER_WHITE;
    }

    /* fieldChanged 가 drawn 을 갱신해 버리므로 먼저 비교해 둔다. 글자색만
     * 달라지는 전환(값 없음 -> 정상)도 칸을 다시 그려야 하는데, 그때 바탕은
     * 양쪽 다 흰색이라 fill 비교만으로는 잡히지 않는다. */
    fillChanged = (drawn.drawn != true) ||
                  (drawn.color != fill) ||
                  (drawn.ink != ink);

    if ((fieldChanged(drawn, whole, valid, fill) != true) &&
        (fillChanged != true))
    {
        return;
    }
    drawn.ink = ink;

    if (valid)
    {
        formatWhole(buffer, 10U, whole);
    }
    else
    {
        formatUnknown(buffer, 10U);
    }

    if (fillChanged)
    {
        panel.setColor(fill);
        /* 물든 칸에서는 면 자체가 경계다. 그 위에 회색 선을 남기면 칸이
         * 액자처럼 보인다. 모서리 그림도 같은 규칙으로 구워 두었다. */
        panel.setBorderColor((cornerIds == CORNER_WHITE)
                                 ? rgb(BORDER_R, BORDER_G, BORDER_B)
                                 : fill);
        swapCorners(corners, cornerIds);
        initial.setColor(trim);
        value.setColor(ink);
        unit.setColor(trim);

        /* 세 위젯을 따로 갱신하는 대신 칸 전체를 dirty 로 만든다. 그 영역에
         * 겹친 글자들이 함께 다시 그려진다. */
        panel.invalidate();
    }
    else
    {
        value.invalidate();
    }
}

void Screen1View::applyLinkState()
{
    touchgfx::colortype color;

    /* 상태가 그대로면 손대지 않는다. 이 배지는 패널/밑줄/글자 세 위젯이라,
     * APPS 패킷마다 다시 그리면 아무 일도 없는 동안 프레임을 계속 깎는다. */
    if (linkStateDrawn &&
        (drawnConnected == linkConnected) &&
        (drawnFaultActive == appsFaultActive))
    {
        return;
    }

    drawnConnected = linkConnected;
    drawnFaultActive = appsFaultActive;
    linkStateDrawn = true;

    if (!linkConnected)
    {
        statusText.setTypedText(touchgfx::TypedText(T_T_DATALOST));
        color = rgb(WARN_R, WARN_G, WARN_B);
    }
    else if (appsFaultActive)
    {
        statusText.setTypedText(touchgfx::TypedText(T_T_STATUSFAULT));
        color = rgb(FAULT_R, FAULT_G, FAULT_B);
    }
    else
    {
        statusText.setTypedText(touchgfx::TypedText(T_T_STATUS));
        color = rgb(OK_R, OK_G, OK_B);
    }

    /* 배지는 표시등과 글자 둘뿐이라 그 둘만 다시 그린다. 뒤 패널은 색이
     * 바뀌지 않으므로 갱신 대상이 아니다. */
    statusText.setColor(color);
    statusAccent.setColor(color);

    statusText.invalidate();
    statusAccent.invalidate();
}

void Screen1View::updateApps(bool connected,
                             uint8_t flags,
                             uint16_t apps1Raw,
                             uint16_t apps2Raw,
                             uint16_t apps1PerMille,
                             uint16_t apps2PerMille,
                             uint16_t pedalPerMille,
                             int16_t command,
                             uint32_t status,
                             uint32_t fault)
{
    const bool appsValid = connected && ((flags & APPS_VALID_FLAG) != 0U);

    /* 페달과 아날로그 입력은 이 화면에 띄우지 않는다. 주행 중 드라이버가 볼 값이
     * 아니고, 정비용 수치는 CLI(display info) 로 볼 수 있다. 링크와 고장 상태만
     * 가져다 쓴다. */
    (void)apps1Raw;
    (void)apps2Raw;
    (void)apps1PerMille;
    (void)apps2PerMille;
    (void)pedalPerMille;
    (void)command;

    linkConnected = connected;
    appsFaultActive = !appsValid || (status != 0U) || (fault != 0U);

    applyLinkState();
}

void Screen1View::updateTelemetry(bool fresh,
                                  uint32_t validMask,
                                  int32_t motorRpm,
                                  uint16_t minCellMv,
                                  uint16_t packDv,
                                  int16_t packDa,
                                  uint16_t socPerMille,
                                  int16_t cellTempDecic,
                                  int16_t igbtTempDecic,
                                  int16_t motorTempDecic)
{
    const bool haveRpm = fresh && ((validMask & TLM_RPM) != 0U);
    const bool haveSoc = fresh && ((validMask & TLM_SOC) != 0U);
    const bool haveMinCell =
        fresh && ((validMask & TLM_CELL_MIN_V) != 0U);
    const bool haveCell = fresh && ((validMask & TLM_CELL_TEMP) != 0U);
    const bool haveIgbt = fresh && ((validMask & TLM_IGBT_TEMP) != 0U);
    const bool haveMotor = fresh && ((validMask & TLM_MOTOR_TEMP) != 0U);

    (void)packDv;
    (void)packDa;

    lastRpm = motorRpm;
    lastRpmValid = haveRpm;

    /* ---------------- RPM ---------------- */
    {
        const int32_t bounded = clampValue(motorRpm, 0, 9999);
        const int32_t stepped =
            ((bounded + (RPM_DISPLAY_STEP / 2)) / RPM_DISPLAY_STEP) *
            RPM_DISPLAY_STEP;
        /*
         * RPM 숫자는 색으로 상태를 말하지 않는다.
         *
         * 회전수 구간은 바로 위 시프트 바가 초록 -> 앰버 -> 레드로 이미 보여
         * 주고 있다. 화면에서 가장 큰 글자까지 같은 정보를 색으로 되풀이하면,
         * 숫자가 쉬지 않고 색을 갈아입는 통에 정작 레드존의 빨강이 평범해진다.
         * 숫자는 읽는 값으로만 두고 레드존에서만 색을 준다.
         */
        touchgfx::colortype color = rgb(DIM_R, DIM_G, DIM_B);

        if (haveRpm)
        {
            color = (bounded >= RPM_RED_MIN)
                        ? rgb(FAULT_R, FAULT_G, FAULT_B)
                        : rgb(FG_R, FG_G, FG_B);
        }

        if (fieldChanged(rpmDrawn, stepped, haveRpm, color))
        {
            if (haveRpm)
            {
                touchgfx::Unicode::snprintf(rpmBuf, 10U, "%d",
                                            static_cast<int>(stepped));
            }
            else
            {
                formatUnknown(rpmBuf, 10U);
            }
            rpmText.setColor(color);
            rpmText.invalidate();
        }
    }
    applyShiftLights(motorRpm, haveRpm);

    /* ---------------- 배터리 잔량 ---------------- */
    {
        const int32_t boundedSoc =
            haveSoc ? clampValue(socPerMille, 0, 1000) : 0;
        /*
         * 반올림하지 않고 버린다. BMS 가 99.5 % 를 주면 반올림한 100 % 는
         * 만충으로 읽히는데, 실제로는 그 위에 아직 0.5 % 가 남아 있다는
         * 뜻이라 방향이 반대다. 잔량은 모자란 쪽으로 틀리는 편이 안전하고,
         * 100 % 는 BMS 가 정말 100 % 를 말할 때만 나와야 한다.
         */
        const int32_t percent = boundedSoc / 10;
        const touchgfx::colortype textColor =
            haveSoc ? rgb(FG_R, FG_G, FG_B) : rgb(DIM_R, DIM_G, DIM_B);

        touchgfx::colortype fillColor = rgb(DIM_R, DIM_G, DIM_B);

        if (haveSoc)
        {
            if (boundedSoc <= SOC_CRIT_PM)
            {
                fillColor = rgb(FAULT_R, FAULT_G, FAULT_B);
            }
            else if (boundedSoc <= SOC_WARN_PM)
            {
                fillColor = rgb(WARN_R, WARN_G, WARN_B);
            }
            else
            {
                fillColor = rgb(OK_R, OK_G, OK_B);
            }
        }

        /* BoxProgress invalidates only the width delta as SOC changes. A full
         * fill redraw is needed only when crossing a color threshold. */
        if (fillColor != socFillColor)
        {
            socFillColor = fillColor;
            socBatteryFill.setColor(fillColor);
            socBatteryFill.invalidate();
        }
        socBatteryFill.setValue(static_cast<int>(boundedSoc));

        if (fieldChanged(socDrawn, percent, haveSoc, textColor))
        {
            if (haveSoc)
            {
                formatPercent(socBuf, 10U, percent);
            }
            else
            {
                formatUnknown(socBuf, 10U);
            }
            socValue.setColor(textColor);
            socValue.invalidate();
        }
    }

    /* ---------------- 속도 ---------------- */
    {
        const int32_t boundedForArc =
            haveMinCell
                ? clampValue(minCellMv,
                             CELL_MIN_ARC_MIN_MV,
                             CELL_MIN_ARC_MAX_MV)
                : CELL_MIN_ARC_MIN_MV;
        const touchgfx::colortype color =
            haveMinCell ? rgb(FG_R, FG_G, FG_B)
                        : rgb(DIM_R, DIM_G, DIM_B);

        minCellArc.setValue(static_cast<int>(
            (boundedForArc / CELL_MIN_ARC_STEP_MV) *
            CELL_MIN_ARC_STEP_MV));

        if (fieldChanged(minCellDrawn, minCellMv, haveMinCell, color))
        {
            if (haveMinCell)
            {
                formatMilliVolts(minCellBuf, 10U, minCellMv);
            }
            else
            {
                formatUnknown(minCellBuf, 10U);
            }
            minCellValue.setColor(color);
            minCellValue.invalidate();
        }
    }

    /* ---------------- 온도 ---------------- */
    updateTempTile(motorDrawn, motorPanel, panelCorner[PANEL_MOTOR],
                   motorTempLabel, motorValue,
                   motorUnit, motorBuf, motorTempDecic, haveMotor,
                   MOTOR_WARN_DECIC, MOTOR_CRIT_DECIC);
    updateTempTile(igbtDrawn, igbtPanel, panelCorner[PANEL_IGBT],
                   igbtTempLabel, igbtValue,
                   igbtUnit, igbtBuf, igbtTempDecic, haveIgbt,
                   IGBT_WARN_DECIC, IGBT_CRIT_DECIC);
    updateTempTile(cellDrawn, cellPanel, panelCorner[PANEL_CELL],
                   cellTempLabel, cellValue,
                   cellUnit, cellBuf, cellTempDecic, haveCell,
                   CELL_WARN_DECIC, CELL_CRIT_DECIC);
}
