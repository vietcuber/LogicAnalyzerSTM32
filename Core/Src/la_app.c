#include "la_app.h"

#include "la_protocol.h"
#include "la_sampling.h"
#include "la_trigger.h"
#include "main.h"
#include <stddef.h>
#include <string.h>

#define LA_CMD_BUFFER_SIZE     32U
#define LA_TRIGGER_TIMEOUT_MS  2000U

static char cmd_buffer[LA_CMD_BUFFER_SIZE];
static volatile uint8_t flag_cmd_ready = 0U;

static volatile CaptureState_t capture_state = CAPTURE_STATE_IDLE;
static volatile uint8_t flag_trigger_pending = 0U;
static volatile uint8_t flag_trigger_start_error = 0U;
static volatile uint32_t trigger_armed_at_ms = 0U;
static volatile uint8_t trigger_timeout_active = 0U;
static volatile uint32_t samples_captured = 0U;

static void Transition_To_Idle_Or_Ready(void)
{
  capture_state = (LA_Sampling_IsConfigured() != 0U)
                    ? CAPTURE_STATE_READY
                    : CAPTURE_STATE_IDLE;
}

static uint8_t Can_Modify_Capture_Config(void)
{
  return (capture_state != CAPTURE_STATE_ARMED) &&
         (capture_state != CAPTURE_STATE_CAPTURING);
}

static void Reset_Capture_Runtime_State(void)
{
  flag_trigger_pending = 0U;
  flag_trigger_start_error = 0U;
  trigger_armed_at_ms = 0U;
  trigger_timeout_active = 0U;
  samples_captured = 0U;
}

static uint8_t Parse_Uint32(const char **cursor, uint32_t *out_val)
{
  const char *p = *cursor;
  uint32_t val = 0U;

  if (*p < '0' || *p > '9') {
    return 0U;
  }

  while (*p >= '0' && *p <= '9') {
    uint32_t digit = (uint32_t)(*p - '0');
    if (val > ((UINT32_MAX - digit) / 10U)) {
      return 0U;
    }
    val = (val * 10U) + digit;
    p++;
  }

  *cursor = p;
  *out_val = val;
  return 1U;
}

static uint8_t Trigger_Mode_From_String(const char *text, TriggerMode_t *mode)
{
  if (strcmp(text, "NONE") == 0) {
    *mode = TRIGGER_NONE;
    return 1U;
  }
  if (strcmp(text, "RISING") == 0) {
    *mode = TRIGGER_RISING;
    return 1U;
  }
  if (strcmp(text, "FALLING") == 0) {
    *mode = TRIGGER_FALLING;
    return 1U;
  }
  if (strcmp(text, "EITHER") == 0) {
    *mode = TRIGGER_EITHER;
    return 1U;
  }
  return 0U;
}

static void Arm_Trigger_Capture(void)
{
  if (LA_Sampling_IsConfigured() == 0U) {
    return;
  }

  LA_Trigger_Disarm();
  Reset_Capture_Runtime_State();

  if (LA_Sampling_Prepare() != HAL_OK) {
    LA_Sampling_Stop();
    capture_state = CAPTURE_STATE_ERROR;
    (void)LA_Protocol_SendError(ERR_CAPTURE_START);
    Transition_To_Idle_Or_Ready();
    return;
  }

  trigger_armed_at_ms = HAL_GetTick();
  trigger_timeout_active = 1U;
  capture_state = CAPTURE_STATE_ARMED;

  /* Send TRIGGER_ARMED before unmasking EXTI so TRIGGER_FIRED can never overtake it. */
  if (LA_Protocol_SendStatus(PKT_TRIGGER_ARMED) != 0U) {
    trigger_timeout_active = 0U;
    LA_Sampling_Stop();
    Transition_To_Idle_Or_Ready();
    (void)LA_Protocol_SendError(ERR_USB_TRANSMIT);
    return;
  }

  LA_Trigger_Arm();
}

static void Start_Immediate_Capture(void)
{
  if (LA_Sampling_IsConfigured() == 0U) {
    return;
  }

  LA_Trigger_Disarm();
  Reset_Capture_Runtime_State();

  if (LA_Sampling_Prepare() != HAL_OK) {
    LA_Sampling_Stop();
    capture_state = CAPTURE_STATE_ERROR;
    (void)LA_Protocol_SendError(ERR_CAPTURE_START);
    Transition_To_Idle_Or_Ready();
    return;
  }

  /* Notify host before starting TIM1 so data packets never precede CAPTURE_STARTED. */
  if (LA_Protocol_SendStatus(PKT_CAPTURE_STARTED) != 0U) {
    LA_Sampling_Stop();
    Transition_To_Idle_Or_Ready();
    (void)LA_Protocol_SendError(ERR_USB_TRANSMIT);
    return;
  }

  capture_state = CAPTURE_STATE_CAPTURING;
  if (LA_Sampling_BeginTimer() != HAL_OK) {
    LA_Sampling_Stop();
    capture_state = CAPTURE_STATE_ERROR;
    (void)LA_Protocol_SendError(ERR_CAPTURE_START);
    Transition_To_Idle_Or_Ready();
  }
}

static void Handle_Configure_Command(const char *args)
{
  uint32_t freq = 0U;
  uint32_t samples = 0U;
  uint8_t result;

  if (Can_Modify_Capture_Config() == 0U ||
      Parse_Uint32(&args, &freq) == 0U ||
      *args++ != ' ' ||
      Parse_Uint32(&args, &samples) == 0U ||
      *args != '\0') {
    (void)LA_Protocol_SendError(ERR_INVALID_CONFIG);
    return;
  }

  result = LA_Sampling_Configure(freq, samples);
  if (result == 0U) {
    Transition_To_Idle_Or_Ready();
    if (LA_Protocol_SendStatus(PKT_CONFIG_OK) != 0U) {
      (void)LA_Protocol_SendError(ERR_USB_TRANSMIT);
    }
  } else if (result == 2U) {
    (void)LA_Protocol_SendError(ERR_SNAPSHOT_LIMIT);
  } else {
    (void)LA_Protocol_SendError(ERR_INVALID_CONFIG);
  }
}

static void Handle_Trigger_Command(const char *args)
{
  TriggerMode_t mode = TRIGGER_NONE;
  uint8_t channel;

  if (Can_Modify_Capture_Config() == 0U ||
      args[0] < '0' || args[0] > '7' ||
      args[1] != ' ' ||
      Trigger_Mode_From_String(&args[2], &mode) == 0U) {
    (void)LA_Protocol_SendError(ERR_INVALID_CONFIG);
    return;
  }

  channel = (uint8_t)(args[0] - '0');
  (void)LA_Trigger_Configure(channel, mode);
  if (LA_Protocol_SendStatus(PKT_TRIGGER_OK) != 0U) {
    (void)LA_Protocol_SendError(ERR_USB_TRANSMIT);
  }
}

static void Handle_Start_Command(void)
{
  if (LA_Sampling_IsConfigured() == 0U) {
    (void)LA_Protocol_SendError(ERR_NOT_CONFIGURED);
    return;
  }

  if (capture_state == CAPTURE_STATE_ARMED ||
      capture_state == CAPTURE_STATE_CAPTURING) {
    return;
  }

  if (LA_Trigger_GetActiveMask() != 0U) {
    Arm_Trigger_Capture();
  } else {
    Start_Immediate_Capture();
  }
}

static void Handle_Stop_Command(void)
{
  uint32_t primask;
  uint32_t captured = 0U;
  uint8_t active = 0U;

  primask = __get_PRIMASK();
  __disable_irq();
  if (capture_state == CAPTURE_STATE_ARMED ||
      capture_state == CAPTURE_STATE_CAPTURING) {
    LA_Trigger_Disarm();
    trigger_timeout_active = 0U;
    trigger_armed_at_ms = 0U;
    flag_trigger_pending = 0U;
    flag_trigger_start_error = 0U;
    captured = samples_captured;
    capture_state = CAPTURE_STATE_ABORTED;
    active = 1U;
  }
  __set_PRIMASK(primask);

  if (active == 0U) {
    return;
  }

  LA_Sampling_Stop();
  if (LA_Protocol_SendCaptureResult(PKT_CAPTURE_STOPPED, captured) != 0U) {
    (void)LA_Protocol_SendError(ERR_USB_TRANSMIT);
  }
  Transition_To_Idle_Or_Ready();
}

static void Process_Command(const char *cmd)
{
  if (strcmp(cmd, "START") == 0) {
    Handle_Start_Command();
  } else if (strcmp(cmd, "STOP") == 0) {
    Handle_Stop_Command();
  } else if (strncmp(cmd, "TRIGGER ", 8U) == 0) {
    Handle_Trigger_Command(cmd + 8U);
  } else if (strncmp(cmd, "CONFIGURE ", 10U) == 0) {
    Handle_Configure_Command(cmd + 10U);
  }
}

static void Poll_Pending_Command(void)
{
  char local_cmd[LA_CMD_BUFFER_SIZE];
  uint32_t primask;
  uint8_t has_cmd = 0U;

  primask = __get_PRIMASK();
  __disable_irq();
  if (flag_cmd_ready != 0U) {
    memcpy(local_cmd, cmd_buffer, LA_CMD_BUFFER_SIZE);
    local_cmd[LA_CMD_BUFFER_SIZE - 1U] = '\0';
    flag_cmd_ready = 0U;
    has_cmd = 1U;
  }
  __set_PRIMASK(primask);

  if (has_cmd != 0U) {
    Process_Command(local_cmd);
  }
}

static void Check_Trigger_Timeout(void)
{
  uint32_t primask;
  uint8_t timed_out = 0U;

  primask = __get_PRIMASK();
  __disable_irq();
  if (trigger_timeout_active != 0U &&
      capture_state == CAPTURE_STATE_ARMED &&
      (HAL_GetTick() - trigger_armed_at_ms) >= LA_TRIGGER_TIMEOUT_MS) {
    LA_Trigger_Disarm();
    trigger_timeout_active = 0U;
    trigger_armed_at_ms = 0U;
    capture_state = CAPTURE_STATE_ABORTED;
    timed_out = 1U;
  }
  __set_PRIMASK(primask);

  if (timed_out != 0U) {
    LA_Sampling_Stop();
    (void)LA_Protocol_SendStatus(PKT_TRIGGER_TIMEOUT);
    Transition_To_Idle_Or_Ready();
  }
}

static void Handle_Snapshot_Capture_Loop(void)
{
  uint32_t target = LA_Sampling_GetTargetSamples();
  const uint8_t *buf = LA_Sampling_GetBuffer();
  uint32_t offset = 0U;
  uint8_t tx_ok = 1U;

  if (LA_Sampling_IsSnapshotDone() == 0U) {
    return;
  }

  LA_Sampling_ClearSnapshotDone();
  LA_Sampling_Stop();
  capture_state = CAPTURE_STATE_DONE;

  while (offset < target) {
    uint32_t remaining = target - offset;
    uint16_t chunk_len = (remaining < LA_MAX_PAYLOAD_SIZE)
                           ? (uint16_t)remaining
                           : LA_MAX_PAYLOAD_SIZE;

    if (LA_Protocol_SendPacket(PKT_SNAPSHOT_DATA, &buf[offset], chunk_len) != 0U) {
      tx_ok = 0U;
      break;
    }
    offset += chunk_len;
  }

  if (tx_ok != 0U) {
    if (LA_Protocol_SendCaptureResult(PKT_CAPTURE_DONE, target) != 0U) {
      (void)LA_Protocol_SendError(ERR_USB_TRANSMIT);
    }
  } else {
    (void)LA_Protocol_SendError(ERR_USB_TRANSMIT);
  }

  Transition_To_Idle_Or_Ready();
}

static void Handle_Streaming_Capture_Loop(void)
{
  uint32_t target = LA_Sampling_GetTargetSamples();
  uint16_t max_block = LA_Sampling_GetStreamBlockSize();
  uint8_t *staging = LA_Protocol_GetPayloadBuffer();
  uint8_t rounds = 0U;

  if (LA_Sampling_HasStreamOverrun() != 0U) {
    LA_Sampling_Stop();
    capture_state = CAPTURE_STATE_ABORTED;
    (void)LA_Protocol_SendError(ERR_DMA_OVERRUN);
    Transition_To_Idle_Or_Ready();
    return;
  }

  while (rounds < 2U &&
         samples_captured < target &&
         flag_cmd_ready == 0U) {
    uint32_t remaining = target - samples_captured;
    uint16_t block_len = (remaining < max_block) ? (uint16_t)remaining : max_block;
    uint8_t copy_res = LA_Sampling_CopyNextStreamBlock(staging, block_len);

    if (copy_res == 0U) {
      break;
    }

    if (copy_res == 2U) {
      LA_Sampling_Stop();
      capture_state = CAPTURE_STATE_ABORTED;
      (void)LA_Protocol_SendError(ERR_DMA_OVERRUN);
      Transition_To_Idle_Or_Ready();
      return;
    }

    if (LA_Protocol_SendPacket(PKT_STREAM_DATA, staging, block_len) != 0U) {
      LA_Sampling_Stop();
      capture_state = CAPTURE_STATE_ABORTED;
      (void)LA_Protocol_SendError(ERR_USB_TRANSMIT);
      Transition_To_Idle_Or_Ready();
      return;
    }

    samples_captured += block_len;
    rounds++;
  }

  if (samples_captured >= target) {
    LA_Sampling_Stop();
    capture_state = CAPTURE_STATE_DONE;
    if (LA_Protocol_SendCaptureResult(PKT_CAPTURE_DONE, samples_captured) != 0U) {
      (void)LA_Protocol_SendError(ERR_USB_TRANSMIT);
    }
    Transition_To_Idle_Or_Ready();
  }
}

void LA_App_Init(void)
{
  flag_cmd_ready = 0U;
  Reset_Capture_Runtime_State();
  LA_Trigger_Init();
  LA_Sampling_Stop();
  Transition_To_Idle_Or_Ready();
}

void LA_App_OnRxBytes(const uint8_t *buf, uint32_t len)
{
  /* Drop if previous command hasn't been copied out by main loop yet. */
  if (flag_cmd_ready != 0U || buf == NULL || len == 0U || len >= LA_CMD_BUFFER_SIZE) {
    return;
  }

  memcpy(cmd_buffer, buf, len);
  cmd_buffer[len] = '\0';
  flag_cmd_ready = 1U;
}

void LA_App_Poll(void)
{
  Poll_Pending_Command();
  Check_Trigger_Timeout();

  if (flag_trigger_start_error != 0U) {
    flag_trigger_start_error = 0U;
    LA_Sampling_Stop();
    capture_state = CAPTURE_STATE_ERROR;
    (void)LA_Protocol_SendError(ERR_CAPTURE_START);
    Transition_To_Idle_Or_Ready();
    return;
  }

  if (flag_trigger_pending != 0U) {
    flag_trigger_pending = 0U;
    if (LA_Protocol_SendStatus(PKT_TRIGGER_FIRED) != 0U) {
      LA_Sampling_Stop();
      capture_state = CAPTURE_STATE_ABORTED;
      (void)LA_Protocol_SendError(ERR_USB_TRANSMIT);
      Transition_To_Idle_Or_Ready();
      return;
    }
  }

  if (capture_state == CAPTURE_STATE_CAPTURING) {
    if (LA_Sampling_GetMode() == MODE_SNAPSHOT) {
      Handle_Snapshot_Capture_Loop();
    } else {
      Handle_Streaming_Capture_Loop();
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (capture_state != CAPTURE_STATE_ARMED) {
    return;
  }

  if (LA_Trigger_MatchPin(GPIO_Pin) == 0U) {
    return;
  }

  LA_Trigger_Disarm();
  trigger_timeout_active = 0U;
  trigger_armed_at_ms = 0U;
  samples_captured = 0U;

  if (LA_Sampling_BeginTimer() != HAL_OK) {
    flag_trigger_start_error = 1U;
    return;
  }

  flag_trigger_pending = 1U;
  capture_state = CAPTURE_STATE_CAPTURING;
}
