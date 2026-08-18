#include <gui/screen1_screen/Screen1View.hpp>
#include <touchgfx/Color.hpp>
#include <touchgfx/canvas_widget_renderer/CanvasWidgetRenderer.hpp>
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
const uint8_t BG_R = 232,     BG_G = 234,     BG_B = 232;
const uint8_t PANEL_R = 255,  PANEL_G = 255,  PANEL_B = 253;
const uint8_t BORDER_R = 168, BORDER_G = 173, BORDER_B = 175;
/* 밝은 바탕에서는 '꺼짐'도 눈에 보여야 눈금이 성립한다. 배경과 붙어 버리면
 * 시프트 바는 몇 칸이 남았는지, 링은 어디까지 차야 하는지를 알 수 없다. */
const uint8_t TRACK_R = 183,  TRACK_G = 189,  TRACK_B = 191;
const uint8_t SEGOFF_R = 188, SEGOFF_G = 194, SEGOFF_B = 196;

/*
 * 색은 위계를 만드는 데만 쓴다. 화면에서 가장 진한 것은 RPM 숫자여야 하고,
 * 나머지는 그보다 한 단계 흐려야 시선이 먼저 갈 곳을 안다.
 *
 * 경고색은 어두운 테마에서 쓰던 것을 그대로 옮기면 안 된다. 흰 바탕에서 노랑
 * (255,181,46)은 배경과 붙어 버려 거의 보이지 않고, 순색 빨강도 생각보다 약하다.
 * 채도 대신 명도를 낮춰서 배경과 벌린다.
 */
const uint8_t FG_R = 12,      FG_G = 14,      FG_B = 16;
const uint8_t DIM_R = 92,     DIM_G = 98,     DIM_B = 102;
const uint8_t OK_R = 0,       OK_G = 138,     OK_B = 58;
const uint8_t WARN_R = 196,   WARN_G = 88,    WARN_B = 0;
const uint8_t FAULT_R = 198,  FAULT_G = 22,   FAULT_B = 22;

/* 경고/고장 타일은 글자색만 바꾸지 않고 배경째 채운다. 밝은 환경에서는 작은
 * 글자 색보다 큰 면적이 훨씬 빨리 읽힌다. 그 위에 얹는 글자색이다. */
const uint8_t ONFILL_R = 255, ONFILL_G = 255, ONFILL_B = 255;

const uint8_t APPS_VALID_FLAG = 1U << 0;

const uint32_t TLM_SOC = 1UL << 2;
const uint32_t TLM_CELL_TEMP = 1UL << 3;
const uint32_t TLM_RPM = 1UL << 5;
const uint32_t TLM_MOTOR_TEMP = 1UL << 8;
const uint32_t TLM_IGBT_TEMP = 1UL << 9;
const uint32_t TLM_CELL_MIN_V = 1UL << 16;

const int32_t RPM_MAX = 6500;
const int32_t RPM_GREEN_MAX = 3800;
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
const int16_t GAUGE_LINE_WIDTH = 10;
const int16_t ARC_START = -135;
const int16_t ARC_END = 135;

/* Horizontal battery geometry inside the former left circular gauge area. */
const int16_t BATTERY_BODY_X = 19;
const int16_t BATTERY_BODY_Y = 76;
const int16_t BATTERY_BODY_W = 130;
const int16_t BATTERY_BODY_H = 68;
const int16_t BATTERY_BORDER = 4;
const int16_t BATTERY_FILL_X = 26;
const int16_t BATTERY_FILL_Y = 85;
const int16_t BATTERY_FILL_W = 116;
const int16_t BATTERY_FILL_H = 50;
const int16_t BATTERY_TERMINAL_X = 149;
const int16_t BATTERY_TERMINAL_Y = 97;
const int16_t BATTERY_TERMINAL_W = 11;
const int16_t BATTERY_TERMINAL_H = 26;

touchgfx::colortype rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return touchgfx::Color::getColorFromRGB(r, g, b);
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

void placePanel(touchgfx::BoxWithBorder& panel,
                int16_t x, int16_t y, int16_t w, int16_t h)
{
    panel.setPosition(x, y, w, h);
    panel.setColor(rgb(PANEL_R, PANEL_G, PANEL_B));
    panel.setBorderColor(rgb(BORDER_R, BORDER_G, BORDER_B));
    panel.setBorderSize(1U);
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

void formatDeci(touchgfx::Unicode::UnicodeChar* buffer,
                uint16_t size, int32_t deci)
{
    int32_t whole = deci / 10;
    int32_t fraction = deci % 10;

    if (fraction < 0)
    {
        fraction = -fraction;
    }

    if ((deci < 0) && (whole == 0))
    {
        touchgfx::Unicode::snprintf(buffer, size, "-0.%d",
                                    static_cast<int>(fraction));
        return;
    }

    touchgfx::Unicode::snprintf(buffer, size, "%d.%d",
                                static_cast<int>(whole),
                                static_cast<int>(fraction));
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
    socBatteryBody.setColor(rgb(PANEL_R, PANEL_G, PANEL_B));
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

    placePanel(statusPanel, 184, 178, 112, 32);
    add(statusPanel);
    placeBox(statusAccent, 184, 208, 112, 2, WARN_R, WARN_G, WARN_B);
    add(statusAccent);
    placeLabel(statusText, 184, 182, 112, 24, T_T_DATALOST);
    add(statusText);

    /* Bottom temperature tiles. Static initials/units stay outside the dirty
     * value rectangle so a number change transfers only the large digits. */
    placePanel(motorPanel, 8, 236, 146, 76);
    add(motorPanel);
    placeLabel(motorTempLabel, 14, 249, 24, 40, T_T_MOTORINITIAL);
    motorTempLabel.setColor(rgb(FG_R, FG_G, FG_B));
    add(motorTempLabel);
    placeValue(motorValue, motorBuf, 38, 249, 86, 40, T_T_TEMPVALUE);
    add(motorValue);
    /* 단위는 값보다 한 단계 작게(40 -> 30) 두되 색은 값과 같은 흰색으로 맞춘다.
     * y 를 값보다 8 px 내린 것은 두 폰트 크기 차이만큼 밑선을 맞추기 위함이다. */
    placeLabel(motorUnit, 122, 257, 32, 36, T_T_DEGC);
    motorUnit.setColor(rgb(FG_R, FG_G, FG_B));
    add(motorUnit);

    placePanel(igbtPanel, 167, 236, 146, 76);
    add(igbtPanel);
    placeLabel(igbtTempLabel, 173, 249, 24, 40, T_T_INVERTERINITIAL);
    igbtTempLabel.setColor(rgb(FG_R, FG_G, FG_B));
    add(igbtTempLabel);
    placeValue(igbtValue, igbtBuf, 197, 249, 86, 40, T_T_TEMPVALUE);
    add(igbtValue);
    placeLabel(igbtUnit, 281, 257, 32, 36, T_T_DEGC);
    igbtUnit.setColor(rgb(FG_R, FG_G, FG_B));
    add(igbtUnit);

    placePanel(cellPanel, 326, 236, 146, 76);
    add(cellPanel);
    placeLabel(cellTempLabel, 332, 249, 24, 40, T_T_CELLINITIAL);
    cellTempLabel.setColor(rgb(FG_R, FG_G, FG_B));
    add(cellTempLabel);
    placeValue(cellValue, cellBuf, 356, 249, 86, 40, T_T_TEMPVALUE);
    add(cellValue);
    placeLabel(cellUnit, 440, 257, 32, 36, T_T_DEGC);
    cellUnit.setColor(rgb(FG_R, FG_G, FG_B));
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
                                 touchgfx::TextArea& initial,
                                 touchgfx::TextAreaWithOneWildcard& value,
                                 touchgfx::TextArea& unit,
                                 touchgfx::Unicode::UnicodeChar* buffer,
                                 int16_t decic,
                                 bool valid,
                                 int16_t warnDecic,
                                 int16_t critDecic)
{
    touchgfx::colortype fill;
    touchgfx::colortype ink;
    bool fillChanged;

    if (valid != true)
    {
        fill = rgb(PANEL_R, PANEL_G, PANEL_B);
        ink = rgb(DIM_R, DIM_G, DIM_B);
    }
    else if (decic >= critDecic)
    {
        fill = rgb(FAULT_R, FAULT_G, FAULT_B);
        ink = rgb(ONFILL_R, ONFILL_G, ONFILL_B);
    }
    else if (decic >= warnDecic)
    {
        fill = rgb(WARN_R, WARN_G, WARN_B);
        ink = rgb(ONFILL_R, ONFILL_G, ONFILL_B);
    }
    else
    {
        fill = rgb(PANEL_R, PANEL_G, PANEL_B);
        ink = rgb(FG_R, FG_G, FG_B);
    }

    /* fieldChanged 가 drawn 을 갱신해 버리므로 먼저 비교해 둔다.
     * 글자색만 달라지는 전환(값 없음 -> 정상)도 타일을 다시 그려야 한다.
     * 배경은 둘 다 흰색이라 fill 비교만으로는 잡히지 않는다. */
    fillChanged = (drawn.drawn != true) ||
                  (drawn.color != fill) ||
                  (drawn.ink != ink);

    if ((fieldChanged(drawn, decic, valid, fill) != true) &&
        (fillChanged != true))
    {
        return;
    }
    drawn.ink = ink;

    if (valid)
    {
        formatDeci(buffer, 10U, decic);
    }
    else
    {
        formatUnknown(buffer, 10U);
    }

    if (fillChanged)
    {
        const bool filled = (fill != rgb(PANEL_R, PANEL_G, PANEL_B));

        panel.setColor(fill);
        panel.setBorderColor(filled ? fill
                                    : rgb(BORDER_R, BORDER_G, BORDER_B));
        initial.setColor(ink);
        value.setColor(ink);
        unit.setColor(ink);

        /* 타일 영역이 dirty 가 되면 그 위에 겹친 글자들도 함께 다시 그려진다. */
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

    statusText.setColor(color);
    statusAccent.setColor(color);
    statusPanel.setBorderColor(color);

    statusText.invalidate();
    statusAccent.invalidate();
    statusPanel.invalidate();
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
        touchgfx::colortype color = rgb(DIM_R, DIM_G, DIM_B);

        if (haveRpm)
        {
            if (bounded >= RPM_RED_MIN)
            {
                color = rgb(FAULT_R, FAULT_G, FAULT_B);
            }
            else if (bounded > RPM_GREEN_MAX)
            {
                color = rgb(WARN_R, WARN_G, WARN_B);
            }
            else
            {
                color = rgb(OK_R, OK_G, OK_B);
            }
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
        const int32_t percent = (boundedSoc + 5) / 10;
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
    updateTempTile(motorDrawn, motorPanel, motorTempLabel, motorValue,
                   motorUnit, motorBuf, motorTempDecic, haveMotor,
                   MOTOR_WARN_DECIC, MOTOR_CRIT_DECIC);
    updateTempTile(igbtDrawn, igbtPanel, igbtTempLabel, igbtValue,
                   igbtUnit, igbtBuf, igbtTempDecic, haveIgbt,
                   IGBT_WARN_DECIC, IGBT_CRIT_DECIC);
    updateTempTile(cellDrawn, cellPanel, cellTempLabel, cellValue,
                   cellUnit, cellBuf, cellTempDecic, haveCell,
                   CELL_WARN_DECIC, CELL_CRIT_DECIC);
}
