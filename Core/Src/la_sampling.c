#include "la_sampling.h"

#include "la_protocol.h"
#include <string.h>

typedef struct {
  uint32_t freq;
  uint16_t psc;
  uint16_t arr;
} FreqConfig_t;

extern TIM_HandleTypeDef htim1;

static uint8_t capture_buffer[LA_BUFFER_SIZE];

static volatile uint8_t flag_half_ready = 0U;
static volatile uint8_t flag_full_ready = 0U;
static volatile uint8_t flag_snapshot_done = 0U;
static volatile uint8_t flag_stream_overrun = 0U;

static uint8_t flag_configured = 0U;
static CaptureMode_t current_mode = MODE_STREAMING;
static uint32_t target_samples = 0U;
static uint16_t stream_block_size = 1U;
static uint8_t next_half_index = 0U;

static const FreqConfig_t Freq_LUT[] = {
  {       1U, 1199U, 59999U },
  {       2U,  599U, 59999U },
  {       5U,  239U, 59999U },
  {      10U,  119U, 59999U },
  {      20U,   59U, 59999U },
  {      50U,   23U, 59999U },
  {     100U,   11U, 59999U },
  {     200U,    5U, 59999U },
  {     500U,    2U, 47999U },
  {    1000U,    1U, 35999U },
  {    2000U,    0U, 35999U },
  {    5000U,    0U, 14399U },
  {   10000U,    0U,  7199U },
  {   20000U,    0U,  3599U },
  {   50000U,    0U,  1439U },
  {  100000U,    0U,   719U },
  {  200000U,    0U,   359U },
  {  500000U,    0U,   143U },
  { 1000000U,    0U,    71U },
  { 2000000U,    0U,    35U }
};

static const uint32_t ALLOWED_SAMPLES[] = {
  100U, 200U, 500U, 1000U, 2000U, 5000U, 10000U, 20000U, 50000U, 100000U
};

static void DMA_HalfConvCallback(DMA_HandleTypeDef *hdma)
{
  (void)hdma;
  if (current_mode == MODE_STREAMING) {
    if (flag_half_ready != 0U) {
      flag_stream_overrun = 1U;
    } else {
      flag_half_ready = 1U;
    }
  }
}

static void DMA_FullConvCallback(DMA_HandleTypeDef *hdma)
{
  (void)hdma;
  if (current_mode == MODE_STREAMING) {
    if (flag_full_ready != 0U) {
      flag_stream_overrun = 1U;
    } else {
      flag_full_ready = 1U;
    }
  } else {
    flag_snapshot_done = 1U;
  }
}

uint8_t LA_Sampling_Configure(uint32_t freq, uint32_t samples)
{
  int32_t freq_index = -1;
  uint8_t sample_valid = 0U;
  uint32_t i;

  for (i = 0U; i < (uint32_t)(sizeof(Freq_LUT) / sizeof(Freq_LUT[0])); i++) {
    if (freq == Freq_LUT[i].freq) {
      freq_index = (int32_t)i;
      break;
    }
  }

  for (i = 0U; i < (uint32_t)(sizeof(ALLOWED_SAMPLES) / sizeof(ALLOWED_SAMPLES[0])); i++) {
    if (samples == ALLOWED_SAMPLES[i]) {
      sample_valid = 1U;
      break;
    }
  }

  if (freq_index < 0 || sample_valid == 0U) {
    return 1U;
  }

  if (freq >= LA_SNAPSHOT_FREQ_THRESHOLD) {
    if (samples > LA_SNAPSHOT_MAX_SAMPLES) {
      return 2U;
    }
    current_mode = MODE_SNAPSHOT;
  } else {
    uint32_t block_size;

    if (samples > LA_STREAMING_MAX_SAMPLES) {
      return 1U;
    }
    current_mode = MODE_STREAMING;
    block_size = (freq + 19U) / 20U;
    if (block_size == 0U) {
      block_size = 1U;
    }
    if (block_size > LA_MAX_PAYLOAD_SIZE) {
      block_size = LA_MAX_PAYLOAD_SIZE;
    }
    if (block_size > samples) {
      block_size = samples;
    }
    stream_block_size = (uint16_t)block_size;
  }

  target_samples = samples;

  htim1.Instance->PSC = Freq_LUT[freq_index].psc;
  htim1.Instance->ARR = Freq_LUT[freq_index].arr;
  htim1.Instance->EGR = TIM_EGR_UG;
  htim1.Instance->SR = (uint32_t)(~TIM_SR_UIF);

  flag_configured = 1U;
  return 0U;
}

HAL_StatusTypeDef LA_Sampling_Prepare(void)
{
  DMA_HandleTypeDef *hdma = htim1.hdma[TIM_DMA_ID_UPDATE];
  uint32_t dma_length;

  if (flag_configured == 0U || hdma == NULL || target_samples == 0U) {
    return HAL_ERROR;
  }

  HAL_TIM_Base_Stop(&htim1);
  __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
  if (hdma->State == HAL_DMA_STATE_BUSY) {
    (void)HAL_DMA_Abort(hdma);
  }

  if (current_mode == MODE_SNAPSHOT) {
    if (target_samples > LA_SNAPSHOT_MAX_SAMPLES || target_samples > LA_BUFFER_SIZE) {
      return HAL_ERROR;
    }
    hdma->Init.Mode = DMA_NORMAL;
    dma_length = target_samples;
  } else {
    if (target_samples > LA_STREAMING_MAX_SAMPLES ||
        stream_block_size == 0U ||
        stream_block_size > LA_MAX_PAYLOAD_SIZE ||
        (2U * (uint32_t)stream_block_size) > LA_BUFFER_SIZE) {
      return HAL_ERROR;
    }
    hdma->Init.Mode = DMA_CIRCULAR;
    dma_length = 2U * (uint32_t)stream_block_size;
  }

  if (HAL_DMA_Init(hdma) != HAL_OK) {
    return HAL_ERROR;
  }

  flag_half_ready = 0U;
  flag_full_ready = 0U;
  flag_snapshot_done = 0U;
  flag_stream_overrun = 0U;
  next_half_index = 0U;

  hdma->XferHalfCpltCallback = (current_mode == MODE_STREAMING) ? DMA_HalfConvCallback : NULL;
  hdma->XferCpltCallback = DMA_FullConvCallback;

  if (HAL_DMA_Start_IT(hdma,
                       (uint32_t)&GPIOA->IDR,
                       (uint32_t)capture_buffer,
                       dma_length) != HAL_OK) {
    return HAL_ERROR;
  }

  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);

  return HAL_OK;
}

HAL_StatusTypeDef LA_Sampling_BeginTimer(void)
{
  return HAL_TIM_Base_Start(&htim1);
}

void LA_Sampling_Stop(void)
{
  DMA_HandleTypeDef *hdma = htim1.hdma[TIM_DMA_ID_UPDATE];

  (void)HAL_TIM_Base_Stop(&htim1);
  __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
  if (hdma != NULL && hdma->State == HAL_DMA_STATE_BUSY) {
    (void)HAL_DMA_Abort(hdma);
  }

  flag_half_ready = 0U;
  flag_full_ready = 0U;
  flag_snapshot_done = 0U;
  flag_stream_overrun = 0U;
  next_half_index = 0U;
}

uint8_t LA_Sampling_CopyNextStreamBlock(uint8_t *dst, uint16_t length)
{
  volatile uint8_t *ready_flag;
  const uint8_t *source;
  uint32_t primask;
  uint8_t ready;
  uint8_t overrun;

  if (length == 0U || length > LA_MAX_PAYLOAD_SIZE || dst == NULL) {
    return 2U;
  }

  if (flag_stream_overrun != 0U) {
    return 2U;
  }

  if (next_half_index == 0U) {
    ready_flag = &flag_half_ready;
    source = capture_buffer;
  } else {
    ready_flag = &flag_full_ready;
    source = &capture_buffer[stream_block_size];
  }

  primask = __get_PRIMASK();
  __disable_irq();
  ready = *ready_flag;
  if (ready != 0U) {
    *ready_flag = 0U;
  }
  __set_PRIMASK(primask);

  if (ready == 0U) {
    return 0U;
  }

  memcpy(dst, source, length);

  primask = __get_PRIMASK();
  __disable_irq();
  overrun = (uint8_t)(flag_stream_overrun != 0U || *ready_flag != 0U);
  __set_PRIMASK(primask);

  if (overrun != 0U) {
    return 2U;
  }

  next_half_index ^= 1U;
  return 1U;
}

uint8_t LA_Sampling_IsConfigured(void)
{
  return flag_configured;
}

CaptureMode_t LA_Sampling_GetMode(void)
{
  return current_mode;
}

uint32_t LA_Sampling_GetTargetSamples(void)
{
  return target_samples;
}

uint16_t LA_Sampling_GetStreamBlockSize(void)
{
  return stream_block_size;
}

uint8_t LA_Sampling_IsSnapshotDone(void)
{
  return flag_snapshot_done;
}

void LA_Sampling_ClearSnapshotDone(void)
{
  flag_snapshot_done = 0U;
}

uint8_t LA_Sampling_HasStreamOverrun(void)
{
  return flag_stream_overrun;
}

const uint8_t *LA_Sampling_GetBuffer(void)
{
  return capture_buffer;
}
