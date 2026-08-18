#include <gui/screen1_screen/Screen1View.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

Screen1Presenter::Screen1Presenter(Screen1View& v)
    : view(v)
{
}

void Screen1Presenter::activate()
{
}

void Screen1Presenter::deactivate()
{
}

void Screen1Presenter::updateApps(bool connected,
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
    view.updateApps(connected,
                    flags,
                    apps1Raw,
                    apps2Raw,
                    apps1PerMille,
                    apps2PerMille,
                    pedalPerMille,
                    command,
                    status,
                    fault);
}

void Screen1Presenter::updateTelemetry(bool fresh,
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
    view.updateTelemetry(fresh,
                         validMask,
                         motorRpm,
                         minCellMv,
                         packDv,
                         packDa,
                         socPerMille,
                         cellTempDecic,
                         igbtTempDecic,
                         motorTempDecic);
}
