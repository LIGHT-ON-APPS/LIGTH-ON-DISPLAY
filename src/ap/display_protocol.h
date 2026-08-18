#ifndef SHARED_DISPLAY_PROTOCOL_H_
#define SHARED_DISPLAY_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


/*
 * L432 -> F411 display packets, protocol version 2.
 *
 * All multi-byte fields are little-endian. Never transmit a C struct directly.
 * CRC-16/CCITT-FALSE covers bytes [0, length - 2).
 *
 * Every packet starts with the same 5-byte preamble so a receiver can frame a
 * stream it does not fully understand:
 *
 *  0  magic[0]             0x4C ('L')
 *  1  magic[1]             0x4F ('O')
 *  2  protocol version     1
 *  3  message type
 *  4  total packet length  <- always byte 4, for every type
 *
 * Type 1 - APPS status (34 bytes):
 *
 *  5  state flags
 *  6  sequence             uint16
 *  8  source time          uint32 milliseconds
 * 12  APPS1 raw            uint16 ADC count
 * 14  APPS2 raw            uint16 ADC count
 * 16  APPS1 position       uint16 per-mille
 * 18  APPS2 position       uint16 per-mille
 * 20  final pedal          uint16 per-mille
 * 22  sent command         int16
 * 24  current status       uint32 bit mask
 * 28  latched fault        uint32 bit mask
 * 32  CRC16                uint16
 *
 * Type 2 - vehicle telemetry (36 bytes). Values decoded from the Orion BMS and
 * the Bamocar inverter by the L432 telemetry module. Any field whose bit is
 * clear in the valid mask has never been received, or is stale; the display
 * must show a placeholder rather than the stale number.
 *
 *  5  reserved             0
 *  6  sequence             uint16
 *  8  source time          uint32 milliseconds
 * 12  valid mask           uint32 bit mask
 * 16  motor speed          int32  rpm
 * 20  minimum cell voltage uint16 1 mV
 * 22  pack voltage         uint16 0.1 V
 * 24  pack current         int16  0.1 A   (+ = discharge)
 * 26  pack SOC             uint16 per-mille
 * 28  highest cell temp    int16  0.1 degC
 * 30  IGBT temperature     int16  0.1 degC
 * 32  motor temperature    int16  0.1 degC
 * 34  CRC16                uint16
 */

#define DISPLAY_PROTOCOL_MAGIC_0                  0x4CU
#define DISPLAY_PROTOCOL_MAGIC_1                  0x4FU
#define DISPLAY_PROTOCOL_VERSION                  2U
#define DISPLAY_PROTOCOL_TYPE_APPS_STATUS         1U
#define DISPLAY_PROTOCOL_TYPE_TELEMETRY           2U
#define DISPLAY_PROTOCOL_PACKET_SIZE              34U
#define DISPLAY_PROTOCOL_CRC_OFFSET               32U
#define DISPLAY_PROTOCOL_TELEMETRY_SIZE           36U
#define DISPLAY_PROTOCOL_TELEMETRY_CRC_OFFSET     34U
#define DISPLAY_PROTOCOL_PER_MILLE_MAX            1000U

/* Longest packet of any type. Receive buffers must be at least this large;
 * byte 4 tells the parser how many bytes the packet in flight actually has. */
#define DISPLAY_PROTOCOL_MAX_PACKET_SIZE          DISPLAY_PROTOCOL_TELEMETRY_SIZE

/* Bit layout mirrors TELEMETRY_SIGNAL_* in
 * stm32l432kc/src/ap/sw/telemetry/telemetry.h so the L432 can forward its mask
 * unchanged. Keep both in step. */
#define DISPLAY_PROTOCOL_TLM_PACK_V               (1UL << 0)
#define DISPLAY_PROTOCOL_TLM_PACK_A               (1UL << 1)
#define DISPLAY_PROTOCOL_TLM_SOC                  (1UL << 2)
#define DISPLAY_PROTOCOL_TLM_CELL_TEMP            (1UL << 3)
#define DISPLAY_PROTOCOL_TLM_RPM                  (1UL << 5)
#define DISPLAY_PROTOCOL_TLM_MOTOR_TEMP           (1UL << 8)
#define DISPLAY_PROTOCOL_TLM_IGBT_TEMP            (1UL << 9)
/* Lowest instantaneous Orion cell voltage collected from a complete cell
 * broadcast scan. */
#define DISPLAY_PROTOCOL_TLM_CELL_MIN_V           (1UL << 16)

#define DISPLAY_PROTOCOL_FLAG_VALID               (1U << 0)
#define DISPLAY_PROTOCOL_FLAG_CONFIGURED          (1U << 1)
#define DISPLAY_PROTOCOL_FLAG_ADC_OK              (1U << 2)
#define DISPLAY_PROTOCOL_FLAG_CAN_TX_OK           (1U << 3)
#define DISPLAY_PROTOCOL_FLAG_CAN_TX_MUTED        (1U << 4)
#define DISPLAY_PROTOCOL_FLAG_FAULT_LATCHED       (1U << 5)


typedef struct
{
  uint16_t sequence;
  uint8_t  flags;
  uint32_t source_time_ms;
  uint16_t apps1_raw;
  uint16_t apps2_raw;
  uint16_t apps1_per_mille;
  uint16_t apps2_per_mille;
  uint16_t pedal_per_mille;
  int16_t  command;
  uint32_t status;
  uint32_t fault;
} display_protocol_status_t;

typedef struct
{
  uint16_t sequence;
  uint32_t source_time_ms;
  uint32_t valid_mask;
  int32_t  motor_rpm;
  uint16_t cell_min_mv;          /* 1 mV     */
  uint16_t pack_dv;              /* 0.1 V    */
  int16_t  pack_da;              /* 0.1 A    */
  uint16_t soc_per_mille;
  int16_t  cell_max_temp_decic;  /* 0.1 degC */
  int16_t  igbt_temp_decic;      /* 0.1 degC */
  int16_t  motor_temp_decic;     /* 0.1 degC */
} display_protocol_telemetry_t;

typedef enum
{
  DISPLAY_PROTOCOL_RESULT_OK = 0,
  DISPLAY_PROTOCOL_RESULT_ARGUMENT,
  DISPLAY_PROTOCOL_RESULT_LENGTH,
  DISPLAY_PROTOCOL_RESULT_MAGIC,
  DISPLAY_PROTOCOL_RESULT_VERSION,
  DISPLAY_PROTOCOL_RESULT_TYPE,
  DISPLAY_PROTOCOL_RESULT_CRC,
  DISPLAY_PROTOCOL_RESULT_RANGE,
} display_protocol_result_t;

typedef enum
{
  DISPLAY_PROTOCOL_SEQUENCE_FIRST = 0,
  DISPLAY_PROTOCOL_SEQUENCE_NEXT,
  DISPLAY_PROTOCOL_SEQUENCE_GAP,
  DISPLAY_PROTOCOL_SEQUENCE_DUPLICATE,
  DISPLAY_PROTOCOL_SEQUENCE_STALE,
} display_protocol_sequence_result_t;


static inline void displayProtocolPutU16Le(uint8_t *p_data, uint16_t value)
{
  p_data[0] = (uint8_t)(value & 0xFFU);
  p_data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static inline void displayProtocolPutU32Le(uint8_t *p_data, uint32_t value)
{
  p_data[0] = (uint8_t)(value & 0xFFU);
  p_data[1] = (uint8_t)((value >> 8) & 0xFFU);
  p_data[2] = (uint8_t)((value >> 16) & 0xFFU);
  p_data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static inline uint16_t displayProtocolGetU16Le(const uint8_t *p_data)
{
  return (uint16_t)((uint16_t)p_data[0] |
                    ((uint16_t)p_data[1] << 8));
}

static inline uint32_t displayProtocolGetU32Le(const uint8_t *p_data)
{
  return (uint32_t)p_data[0] |
         ((uint32_t)p_data[1] << 8) |
         ((uint32_t)p_data[2] << 16) |
         ((uint32_t)p_data[3] << 24);
}

static inline uint16_t displayProtocolCrc16(const uint8_t *p_data, size_t length)
{
  uint16_t crc = 0xFFFFU;

  if ((p_data == NULL) && (length > 0U))
  {
    return 0U;
  }

  for (size_t i = 0U; i < length; i++)
  {
    crc ^= (uint16_t)((uint16_t)p_data[i] << 8);

    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc = (uint16_t)(crc << 1);
      }
    }
  }

  return crc;
}

static inline bool displayProtocolEncodeStatus(
    uint8_t *p_packet,
    size_t packet_size,
    const display_protocol_status_t *p_status)
{
  uint16_t crc;

  if ((p_packet == NULL) ||
      (p_status == NULL) ||
      (packet_size < DISPLAY_PROTOCOL_PACKET_SIZE) ||
      (p_status->apps1_per_mille > DISPLAY_PROTOCOL_PER_MILLE_MAX) ||
      (p_status->apps2_per_mille > DISPLAY_PROTOCOL_PER_MILLE_MAX) ||
      (p_status->pedal_per_mille > DISPLAY_PROTOCOL_PER_MILLE_MAX))
  {
    return false;
  }

  p_packet[0] = DISPLAY_PROTOCOL_MAGIC_0;
  p_packet[1] = DISPLAY_PROTOCOL_MAGIC_1;
  p_packet[2] = DISPLAY_PROTOCOL_VERSION;
  p_packet[3] = DISPLAY_PROTOCOL_TYPE_APPS_STATUS;
  p_packet[4] = DISPLAY_PROTOCOL_PACKET_SIZE;
  p_packet[5] = p_status->flags;
  displayProtocolPutU16Le(&p_packet[6], p_status->sequence);
  displayProtocolPutU32Le(&p_packet[8], p_status->source_time_ms);
  displayProtocolPutU16Le(&p_packet[12], p_status->apps1_raw);
  displayProtocolPutU16Le(&p_packet[14], p_status->apps2_raw);
  displayProtocolPutU16Le(&p_packet[16], p_status->apps1_per_mille);
  displayProtocolPutU16Le(&p_packet[18], p_status->apps2_per_mille);
  displayProtocolPutU16Le(&p_packet[20], p_status->pedal_per_mille);
  displayProtocolPutU16Le(&p_packet[22], (uint16_t)p_status->command);
  displayProtocolPutU32Le(&p_packet[24], p_status->status);
  displayProtocolPutU32Le(&p_packet[28], p_status->fault);

  crc = displayProtocolCrc16(p_packet, DISPLAY_PROTOCOL_CRC_OFFSET);
  displayProtocolPutU16Le(&p_packet[DISPLAY_PROTOCOL_CRC_OFFSET], crc);

  return true;
}

static inline display_protocol_result_t displayProtocolDecodeStatus(
    const uint8_t *p_packet,
    size_t packet_size,
    display_protocol_status_t *p_status)
{
  uint16_t received_crc;
  uint16_t calculated_crc;

  if ((p_packet == NULL) || (p_status == NULL))
  {
    return DISPLAY_PROTOCOL_RESULT_ARGUMENT;
  }
  if ((packet_size != DISPLAY_PROTOCOL_PACKET_SIZE) ||
      (p_packet[4] != DISPLAY_PROTOCOL_PACKET_SIZE))
  {
    return DISPLAY_PROTOCOL_RESULT_LENGTH;
  }
  if ((p_packet[0] != DISPLAY_PROTOCOL_MAGIC_0) ||
      (p_packet[1] != DISPLAY_PROTOCOL_MAGIC_1))
  {
    return DISPLAY_PROTOCOL_RESULT_MAGIC;
  }
  if (p_packet[2] != DISPLAY_PROTOCOL_VERSION)
  {
    return DISPLAY_PROTOCOL_RESULT_VERSION;
  }
  if (p_packet[3] != DISPLAY_PROTOCOL_TYPE_APPS_STATUS)
  {
    return DISPLAY_PROTOCOL_RESULT_TYPE;
  }

  received_crc = displayProtocolGetU16Le(
      &p_packet[DISPLAY_PROTOCOL_CRC_OFFSET]);
  calculated_crc = displayProtocolCrc16(
      p_packet,
      DISPLAY_PROTOCOL_CRC_OFFSET);
  if (received_crc != calculated_crc)
  {
    return DISPLAY_PROTOCOL_RESULT_CRC;
  }

  p_status->flags = p_packet[5];
  p_status->sequence = displayProtocolGetU16Le(&p_packet[6]);
  p_status->source_time_ms = displayProtocolGetU32Le(&p_packet[8]);
  p_status->apps1_raw = displayProtocolGetU16Le(&p_packet[12]);
  p_status->apps2_raw = displayProtocolGetU16Le(&p_packet[14]);
  p_status->apps1_per_mille = displayProtocolGetU16Le(&p_packet[16]);
  p_status->apps2_per_mille = displayProtocolGetU16Le(&p_packet[18]);
  p_status->pedal_per_mille = displayProtocolGetU16Le(&p_packet[20]);
  p_status->command = (int16_t)displayProtocolGetU16Le(&p_packet[22]);
  p_status->status = displayProtocolGetU32Le(&p_packet[24]);
  p_status->fault = displayProtocolGetU32Le(&p_packet[28]);

  if ((p_status->apps1_per_mille > DISPLAY_PROTOCOL_PER_MILLE_MAX) ||
      (p_status->apps2_per_mille > DISPLAY_PROTOCOL_PER_MILLE_MAX) ||
      (p_status->pedal_per_mille > DISPLAY_PROTOCOL_PER_MILLE_MAX))
  {
    return DISPLAY_PROTOCOL_RESULT_RANGE;
  }

  return DISPLAY_PROTOCOL_RESULT_OK;
}

/*
 * Expected total length for a message type, or 0 for a type this build does not
 * know. A receiver uses this to validate byte 4 before trusting it as a length,
 * so a corrupted length byte cannot make the parser wait for a packet that will
 * never arrive.
 */
static inline uint8_t displayProtocolSizeForType(uint8_t type)
{
  if (type == DISPLAY_PROTOCOL_TYPE_APPS_STATUS)
  {
    return DISPLAY_PROTOCOL_PACKET_SIZE;
  }
  if (type == DISPLAY_PROTOCOL_TYPE_TELEMETRY)
  {
    return DISPLAY_PROTOCOL_TELEMETRY_SIZE;
  }

  return 0U;
}

static inline bool displayProtocolEncodeTelemetry(
    uint8_t *p_packet,
    size_t packet_size,
    const display_protocol_telemetry_t *p_telemetry)
{
  uint16_t crc;

  if ((p_packet == NULL) ||
      (p_telemetry == NULL) ||
      (packet_size < DISPLAY_PROTOCOL_TELEMETRY_SIZE) ||
      (p_telemetry->soc_per_mille > DISPLAY_PROTOCOL_PER_MILLE_MAX))
  {
    return false;
  }

  p_packet[0] = DISPLAY_PROTOCOL_MAGIC_0;
  p_packet[1] = DISPLAY_PROTOCOL_MAGIC_1;
  p_packet[2] = DISPLAY_PROTOCOL_VERSION;
  p_packet[3] = DISPLAY_PROTOCOL_TYPE_TELEMETRY;
  p_packet[4] = DISPLAY_PROTOCOL_TELEMETRY_SIZE;
  p_packet[5] = 0U;
  displayProtocolPutU16Le(&p_packet[6], p_telemetry->sequence);
  displayProtocolPutU32Le(&p_packet[8], p_telemetry->source_time_ms);
  displayProtocolPutU32Le(&p_packet[12], p_telemetry->valid_mask);
  displayProtocolPutU32Le(&p_packet[16], (uint32_t)p_telemetry->motor_rpm);
  displayProtocolPutU16Le(&p_packet[20], p_telemetry->cell_min_mv);
  displayProtocolPutU16Le(&p_packet[22], p_telemetry->pack_dv);
  displayProtocolPutU16Le(&p_packet[24], (uint16_t)p_telemetry->pack_da);
  displayProtocolPutU16Le(&p_packet[26], p_telemetry->soc_per_mille);
  displayProtocolPutU16Le(&p_packet[28],
                          (uint16_t)p_telemetry->cell_max_temp_decic);
  displayProtocolPutU16Le(&p_packet[30],
                          (uint16_t)p_telemetry->igbt_temp_decic);
  displayProtocolPutU16Le(&p_packet[32],
                          (uint16_t)p_telemetry->motor_temp_decic);

  crc = displayProtocolCrc16(p_packet, DISPLAY_PROTOCOL_TELEMETRY_CRC_OFFSET);
  displayProtocolPutU16Le(&p_packet[DISPLAY_PROTOCOL_TELEMETRY_CRC_OFFSET], crc);

  return true;
}

static inline display_protocol_result_t displayProtocolDecodeTelemetry(
    const uint8_t *p_packet,
    size_t packet_size,
    display_protocol_telemetry_t *p_telemetry)
{
  uint16_t received_crc;
  uint16_t calculated_crc;

  if ((p_packet == NULL) || (p_telemetry == NULL))
  {
    return DISPLAY_PROTOCOL_RESULT_ARGUMENT;
  }
  if ((packet_size != DISPLAY_PROTOCOL_TELEMETRY_SIZE) ||
      (p_packet[4] != DISPLAY_PROTOCOL_TELEMETRY_SIZE))
  {
    return DISPLAY_PROTOCOL_RESULT_LENGTH;
  }
  if ((p_packet[0] != DISPLAY_PROTOCOL_MAGIC_0) ||
      (p_packet[1] != DISPLAY_PROTOCOL_MAGIC_1))
  {
    return DISPLAY_PROTOCOL_RESULT_MAGIC;
  }
  if (p_packet[2] != DISPLAY_PROTOCOL_VERSION)
  {
    return DISPLAY_PROTOCOL_RESULT_VERSION;
  }
  if (p_packet[3] != DISPLAY_PROTOCOL_TYPE_TELEMETRY)
  {
    return DISPLAY_PROTOCOL_RESULT_TYPE;
  }

  received_crc = displayProtocolGetU16Le(
      &p_packet[DISPLAY_PROTOCOL_TELEMETRY_CRC_OFFSET]);
  calculated_crc = displayProtocolCrc16(
      p_packet,
      DISPLAY_PROTOCOL_TELEMETRY_CRC_OFFSET);
  if (received_crc != calculated_crc)
  {
    return DISPLAY_PROTOCOL_RESULT_CRC;
  }

  p_telemetry->sequence = displayProtocolGetU16Le(&p_packet[6]);
  p_telemetry->source_time_ms = displayProtocolGetU32Le(&p_packet[8]);
  p_telemetry->valid_mask = displayProtocolGetU32Le(&p_packet[12]);
  p_telemetry->motor_rpm = (int32_t)displayProtocolGetU32Le(&p_packet[16]);
  p_telemetry->cell_min_mv = displayProtocolGetU16Le(&p_packet[20]);
  p_telemetry->pack_dv = displayProtocolGetU16Le(&p_packet[22]);
  p_telemetry->pack_da = (int16_t)displayProtocolGetU16Le(&p_packet[24]);
  p_telemetry->soc_per_mille = displayProtocolGetU16Le(&p_packet[26]);
  p_telemetry->cell_max_temp_decic =
      (int16_t)displayProtocolGetU16Le(&p_packet[28]);
  p_telemetry->igbt_temp_decic =
      (int16_t)displayProtocolGetU16Le(&p_packet[30]);
  p_telemetry->motor_temp_decic =
      (int16_t)displayProtocolGetU16Le(&p_packet[32]);

  if (p_telemetry->soc_per_mille > DISPLAY_PROTOCOL_PER_MILLE_MAX)
  {
    return DISPLAY_PROTOCOL_RESULT_RANGE;
  }

  return DISPLAY_PROTOCOL_RESULT_OK;
}

static inline display_protocol_sequence_result_t
displayProtocolClassifySequence(bool has_previous,
                                uint16_t previous,
                                uint16_t current,
                                uint16_t *p_gap_count)
{
  uint16_t delta;

  if (p_gap_count != NULL)
  {
    *p_gap_count = 0U;
  }
  if (has_previous != true)
  {
    return DISPLAY_PROTOCOL_SEQUENCE_FIRST;
  }

  delta = (uint16_t)(current - previous);
  if (delta == 0U)
  {
    return DISPLAY_PROTOCOL_SEQUENCE_DUPLICATE;
  }
  if (delta == 1U)
  {
    return DISPLAY_PROTOCOL_SEQUENCE_NEXT;
  }
  if (delta < 0x8000U)
  {
    if (p_gap_count != NULL)
    {
      *p_gap_count = (uint16_t)(delta - 1U);
    }
    return DISPLAY_PROTOCOL_SEQUENCE_GAP;
  }

  return DISPLAY_PROTOCOL_SEQUENCE_STALE;
}


#ifdef __cplusplus
}
#endif

#endif /* SHARED_DISPLAY_PROTOCOL_H_ */
