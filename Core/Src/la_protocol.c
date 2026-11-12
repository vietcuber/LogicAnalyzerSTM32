#include "la_protocol.h"

#include "main.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define LA_USB_TX_TIMEOUT_MS 50U

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Single contiguous frame buffer so header + payload go out in one USB transfer. */
static uint8_t tx_frame_buffer[sizeof(PacketHeader_t) + LA_MAX_PAYLOAD_SIZE];
static uint8_t global_seq = 0U;

static uint8_t USB_Transmit_Safe(uint8_t *data, uint16_t length)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  uint32_t tickstart;

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || hcdc == NULL) {
    return 1U;
  }

  tickstart = HAL_GetTick();
  while (CDC_Transmit_FS(data, length) == USBD_BUSY) {
    if ((HAL_GetTick() - tickstart) > LA_USB_TX_TIMEOUT_MS) {
      return 1U;
    }
  }

  /* Wait until the USB IN endpoint finishes draining this transfer before reusing tx_frame_buffer. */
  tickstart = HAL_GetTick();
  while (hcdc->TxState != 0U) {
    if ((HAL_GetTick() - tickstart) > LA_USB_TX_TIMEOUT_MS) {
      return 2U;
    }
  }

  return 0U;
}

uint8_t *LA_Protocol_GetPayloadBuffer(void)
{
  return &tx_frame_buffer[sizeof(PacketHeader_t)];
}

uint8_t LA_Protocol_SendPacket(PacketType_t type, const uint8_t *payload, uint16_t length)
{
  PacketHeader_t header;
  uint8_t *frame_payload;

  if (length > LA_MAX_PAYLOAD_SIZE || (length > 0U && payload == NULL)) {
    return 3U;
  }

  header.magic[0] = LA_PKT_MAGIC_0;
  header.magic[1] = LA_PKT_MAGIC_1;
  header.pkt_type = (uint8_t)type;
  header.seq_num = global_seq++;
  header.data_len = length;

  memcpy(tx_frame_buffer, &header, sizeof(PacketHeader_t));

  frame_payload = &tx_frame_buffer[sizeof(PacketHeader_t)];
  if (length > 0U && payload != frame_payload) {
    memcpy(frame_payload, payload, length);
  }

  return USB_Transmit_Safe(tx_frame_buffer, (uint16_t)(sizeof(PacketHeader_t) + length));
}

uint8_t LA_Protocol_SendStatus(PacketType_t type)
{
  return LA_Protocol_SendPacket(type, NULL, 0U);
}

uint8_t LA_Protocol_SendError(PacketError_t error)
{
  uint8_t error_code = (uint8_t)error;
  return LA_Protocol_SendPacket(PKT_ERROR, &error_code, 1U);
}

uint8_t LA_Protocol_SendCaptureResult(PacketType_t type, uint32_t total_samples)
{
  uint8_t payload[4];

  payload[0] = (uint8_t)(total_samples & 0xFFU);
  payload[1] = (uint8_t)((total_samples >> 8) & 0xFFU);
  payload[2] = (uint8_t)((total_samples >> 16) & 0xFFU);
  payload[3] = (uint8_t)((total_samples >> 24) & 0xFFU);

  return LA_Protocol_SendPacket(type, payload, (uint16_t)sizeof(payload));
}
