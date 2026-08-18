#ifndef MODEL_HPP
#define MODEL_HPP

#include <stdint.h>

class ModelListener;

class Model
{
public:
    Model();

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();
protected:
    ModelListener* modelListener;
    uint32_t tickCounter;
    uint32_t lastValidPacketCount;
    /* 텔레메트리는 APPS 와 패킷 주기가 달라 갱신 판정을 따로 건다. */
    uint32_t lastTelemetryPacketCount;
    bool lastConnected;
    bool lastTelemetryFresh;
    bool linkStateInitialized;
    bool telemetryStateInitialized;
};

#endif // MODEL_HPP
