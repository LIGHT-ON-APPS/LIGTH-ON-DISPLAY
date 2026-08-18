#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>
#include <stdint.h>

class ModelListener
{
public:
    ModelListener() : model(0) {}
    virtual ~ModelListener() {}

    void bind(Model* m)
    {
        model = m;
    }

    /* L432 가 20 Hz 로 보내는 APPS 상태. 링크 연결 판정도 이 패킷으로 한다. */
    virtual void updateApps(bool connected,
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
        (void)connected;
        (void)flags;
        (void)apps1Raw;
        (void)apps2Raw;
        (void)apps1PerMille;
        (void)apps2PerMille;
        (void)pedalPerMille;
        (void)command;
        (void)status;
        (void)fault;
    }

    /*
     * Orion BMS / Bamocar 에서 디코딩된 차량 신호. 10 Hz.
     * fresh 가 false 면 값은 마지막으로 받은 것이고 이미 낡았다는 뜻이다.
     * validMask 비트는 shared/display_protocol.h 의 DISPLAY_PROTOCOL_TLM_* 이다.
     */
    virtual void updateTelemetry(bool fresh,
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
        (void)fresh;
        (void)validMask;
        (void)motorRpm;
        (void)minCellMv;
        (void)packDv;
        (void)packDa;
        (void)socPerMille;
        (void)cellTempDecic;
        (void)igbtTempDecic;
        (void)motorTempDecic;
    }

protected:
    Model* model;
};

#endif // MODELLISTENER_HPP
