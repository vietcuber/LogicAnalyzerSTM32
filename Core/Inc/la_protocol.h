#ifndef LA_PROTOCOL_H
#define LA_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LA_PKT_MAGIC_0            0xAAU
#define LA_PKT_MAGIC_1            0x55U
#define LA_MAX_PAYLOAD_SIZE       1024U

#pragma pack(push, 1)
typedef struct {
  uint8_t  magic[2];
  uint8_t  pkt_type;
  uint8_t  seq_num;
  uint16_t data_len;
} PacketHeader_t;
#pragma pack(pop)

typedef enum {
  PKT_STREAM_DATA      = 0x01,
  PKT_SNAPSHOT_DATA    = 0x02,
  PKT_CONFIG_OK        = 0x10,
  PKT_CAPTURE_STARTED  = 0x11,
  PKT_CAPTURE_DONE     = 0x12,
  PKT_CAPTURE_STOPPED  = 0x13,
  PKT_TRIGGER_OK       = 0x20,
  PKT_TRIGGER_ARMED    = 0x21,
  PKT_TRIGGER_FIRED    = 0x22,
  PKT_TRIGGER_TIMEOUT  = 0x23,
  PKT_ERROR            = 0x7F
} PacketType_t;

typedef enum {
  ERR_NOT_CONFIGURED  = 0x01,
  ERR_INVALID_CONFIG  = 0x02,
  ERR_SNAPSHOT_LIMIT  = 0x03,
  ERR_CAPTURE_START   = 0x04,
  ERR_USB_TRANSMIT    = 0x05,
  ERR_DMA_OVERRUN     = 0x06
} PacketError_t;

typedef enum {
  TRIGGER_NONE = 0,
  TRIGGER_RISING,
  TRIGGER_FALLING,
  TRIGGER_EITHER
} TriggerMode_t;

/**
 * @brief Return the shared TX payload staging area inside the frame buffer.
 *        Writing directly here lets streaming packets avoid an extra copy.
 */
uint8_t *LA_Protocol_GetPayloadBuffer(void);

uint8_t LA_Protocol_SendPacket(PacketType_t type, const uint8_t *payload, uint16_t length);
uint8_t LA_Protocol_SendStatus(PacketType_t type);
uint8_t LA_Protocol_SendError(PacketError_t error);
uint8_t LA_Protocol_SendCaptureResult(PacketType_t type, uint32_t total_samples);

#ifdef __cplusplus
}
#endif

#endif /* LA_PROTOCOL_H */
