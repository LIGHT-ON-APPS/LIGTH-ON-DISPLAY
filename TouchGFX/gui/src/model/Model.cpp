#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#if !defined(SIMULATOR)
#include "hw_def.h"
#endif

#if !defined(SIMULATOR) && (HW_USE_TOUCHGFX_DEMO == 0)
#include "display_link.h"
#else
#include "display_protocol.h"
#endif

#if defined(SIMULATOR) || (HW_USE_TOUCHGFX_DEMO != 0)
namespace
{
/* Returns a repeating 0..1000..0 triangle wave. */
uint16_t trianglePerMille(uint32_t tick, uint32_t periodTicks)
{
    const uint32_t halfPeriod = periodTicks / 2U;
    uint32_t phase = tick % periodTicks;

    if (phase > halfPeriod)
    {
        phase = periodTicks - phase;
    }

    return static_cast<uint16_t>((phase * 1000U) / halfPeriod);
}
}
#endif

Model::Model()
    : modelListener(0),
      tickCounter(0),
      lastValidPacketCount(0U),
      lastTelemetryPacketCount(0U),
      lastConnected(false),
      lastTelemetryFresh(false),
      linkStateInitialized(false),
      telemetryStateInitialized(false)
{
}

void Model::tick()
{
    tickCounter++;

#if !defined(SIMULATOR) && (HW_USE_TOUCHGFX_DEMO == 0)
    display_link_snapshot_t snapshot = {};
    bool hasData = displayLinkGetSnapshot(&snapshot);
    bool connected = hasData && snapshot.connected;
    bool packetChanged = hasData &&
                         (snapshot.valid_packet_count != lastValidPacketCount);

    if (modelListener == 0)
    {
        return;
    }

    if ((!linkStateInitialized) ||
        (connected != lastConnected) ||
        packetChanged)
    {
        modelListener->updateApps(connected,
                                  snapshot.status.flags,
                                  snapshot.status.apps1_raw,
                                  snapshot.status.apps2_raw,
                                  snapshot.status.apps1_per_mille,
                                  snapshot.status.apps2_per_mille,
                                  snapshot.status.pedal_per_mille,
                                  snapshot.status.command,
                                  snapshot.status.status,
                                  snapshot.status.fault);

        lastValidPacketCount = snapshot.valid_packet_count;
        lastConnected = connected;
        linkStateInitialized = true;
    }

    /*
     * 텔레메트리는 APPS 와 갱신 주기도 유실 조건도 다르므로 따로 판정한다.
     * 새 패킷이 왔을 때와, 값은 그대로인데 신선도만 떨어졌을 때 모두 화면을
     * 갱신해야 한다. 후자를 빠뜨리면 CAN 이 끊긴 뒤에도 마지막 숫자가 계속
     * 정상값처럼 남는다.
     */
    {
        bool telemetryChanged =
            snapshot.telemetry_packet_count != lastTelemetryPacketCount;

        if ((!telemetryStateInitialized) ||
            telemetryChanged ||
            (snapshot.telemetry_fresh != lastTelemetryFresh))
        {
            modelListener->updateTelemetry(
                snapshot.telemetry_fresh,
                snapshot.telemetry.valid_mask,
                snapshot.telemetry.motor_rpm,
                snapshot.telemetry.cell_min_mv,
                snapshot.telemetry.pack_dv,
                snapshot.telemetry.pack_da,
                snapshot.telemetry.soc_per_mille,
                snapshot.telemetry.cell_max_temp_decic,
                snapshot.telemetry.igbt_temp_decic,
                snapshot.telemetry.motor_temp_decic);

            lastTelemetryPacketCount = snapshot.telemetry_packet_count;
            lastTelemetryFresh = snapshot.telemetry_fresh;
            telemetryStateInitialized = true;
        }
    }
#else
    /* Simulator/F411 demo: synthetic values for testing without the L432KC. */
    /*
     * Standalone dashboard demo for bring-up without the STM32L432KC.
     * Updating at about 30 Hz makes the standalone RPM motion noticeably less
     * stepped while still leaving SPI bandwidth for the other dirty regions.
     */
    if ((modelListener == 0) || ((tickCounter % 2U) != 0U))
    {
        return;
    }

    const uint16_t driveWave = trianglePerMille(tickCounter, 720U);
    const uint16_t socWave = trianglePerMille(tickCounter + 600U, 2400U);
    const uint16_t tempWave = trianglePerMille(tickCounter + 300U, 1200U);
    const uint16_t apps1Raw = static_cast<uint16_t>(
        310U + (static_cast<uint32_t>(driveWave) * (2792U - 310U) /
                1000U));
    const uint16_t apps2Raw = static_cast<uint16_t>(
        410U + (static_cast<uint32_t>(driveWave) * (3685U - 410U) /
                1000U));

    modelListener->updateApps(
        true,
        0x0FU,
        apps1Raw,
        apps2Raw,
        driveWave,
        driveWave,
        driveWave,
        static_cast<int16_t>(static_cast<int32_t>(driveWave) * 32767L /
                             1000L),
        0U,
        0U);

    modelListener->updateTelemetry(
        true,
        DISPLAY_PROTOCOL_TLM_PACK_V | DISPLAY_PROTOCOL_TLM_PACK_A |
            DISPLAY_PROTOCOL_TLM_SOC | DISPLAY_PROTOCOL_TLM_CELL_TEMP |
            DISPLAY_PROTOCOL_TLM_RPM | DISPLAY_PROTOCOL_TLM_IGBT_TEMP |
            DISPLAY_PROTOCOL_TLM_MOTOR_TEMP |
            DISPLAY_PROTOCOL_TLM_CELL_MIN_V,
        800L + (static_cast<int32_t>(driveWave) * 5700L / 1000L),
        static_cast<uint16_t>(3400U +
                              (static_cast<uint32_t>(socWave) * 700U /
                               1000U)),
        static_cast<uint16_t>(3600U +
                              (static_cast<uint32_t>(socWave) * 450U /
                               1000U)),
        static_cast<int16_t>(static_cast<int32_t>(driveWave) * 1800L /
                             1000L),
        static_cast<uint16_t>(100U +
                              (static_cast<uint32_t>(socWave) * 850U /
                               1000U)),
        static_cast<int16_t>(400L +
                             (static_cast<int32_t>(tempWave) * 180L /
                              1000L)),
        static_cast<int16_t>(420L +
                             (static_cast<int32_t>(tempWave) * 250L /
                              1000L)),
        /* 모터는 인버터보다 천천히, 조금 낮게 오른다. */
        static_cast<int16_t>(650L +
                             (static_cast<int32_t>(tempWave) * 500L /
                              1000L)));
#endif
}
