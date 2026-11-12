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

/* Circular DMA trigger state */
static volatile uint8_t triggered_capture_active = 0U;
static volatile uint32_t dma_wrap_count = 0U;
static uint32_t requested_pre_samples = 0U;
static uint32_t snapshot_start_idx = 0U;
static uint32_t ring_read_count = 0U;

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

static uint32_t Compute_Ring_Total_Count(uint32_t cndtr, uint32_t dma_isr)
{
  uint32_t wraps = dma_wrap_count;

  if (cndtr > LA_BUFFER_SIZE) {
    cndtr = LA_BUFFER_SIZE;
  }

  /* Account for a hardware DMA wrap whose TC ISR (lower prio) has not run yet. */
  if ((dma_isr & DMA_ISR_TCIF5) != 0U && cndtr > (LA_BUFFER_SIZE / 2U)) {
    wraps++;
  }

  return (wraps * LA_BUFFER_SIZE) + (LA_BUFFER_SIZE - cndtr);
}

static uint32_t Read_DMA_Ring_Total_Count(void)
{
  DMA_HandleTypeDef *hdma = htim1.hdma[TIM_DMA_ID_UPDATE];
  return Compute_Ring_Total_Count(hdma->Instance->CNDTR, DMA1->ISR);
}

static void Stop_TIM3_Counter(void)
{
  TIM3->DIER &= ~TIM_DIER_UIE;
  TIM3->CR1 &= ~TIM_CR1_CEN;
  TIM3->SR = (uint16_t)(~TIM_SR_UIF);
  HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
}

static void DMA_HalfConvCallback(DMA_HandleTypeDef *hdma)
{
  (void)hdma;
  if (triggered_capture_active == 0U && current_mode == MODE_STREAMING) {
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
  if (triggered_capture_active != 0U) {
    dma_wrap_count++;
    return;
  }

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

void LA_Sampling_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();

  TIM3->CR1 = 0U;
  TIM3->DIER = 0U;
  TIM3->PSC = 0U;
  /* External Clock Mode 1 fed by ITR0 (TIM1_TRGO). */
  TIM3->SMCR = TIM_SLAVEMODE_EXTERNAL1 | TIM_TS_ITR0;
  TIM3->SR = 0U;

  /* Keep EXTI & TIM3 at highest priority (0,0); lower DMA & USB below them. */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0U, 0U);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 1U, 0U);
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 2U, 0U);
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

  LA_Sampling_Stop();
  triggered_capture_active = 0U;

  if (current_mode == MODE_SNAPSHOT) {
    hdma->Init.Mode = DMA_NORMAL;
    dma_length = target_samples;
  } else {
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

HAL_StatusTypeDef LA_Sampling_PrepareTriggered(uint8_t pre_trigger_pct)
{
  DMA_HandleTypeDef *hdma = htim1.hdma[TIM_DMA_ID_UPDATE];
  uint32_t pct = (pre_trigger_pct > LA_MAX_PRETRIGGER_PCT)
                   ? LA_MAX_PRETRIGGER_PCT
                   : (uint32_t)pre_trigger_pct;
  uint32_t pre_samples;

  if (flag_configured == 0U || hdma == NULL || target_samples == 0U) {
    return HAL_ERROR;
  }

  LA_Sampling_Stop();

  pre_samples = (target_samples * pct) / 100U;
  if (current_mode == MODE_STREAMING && pre_samples > LA_STREAM_PRE_MAX_SAMPLES) {
    pre_samples = LA_STREAM_PRE_MAX_SAMPLES;
  }
  if (pre_samples >= target_samples) {
    pre_samples = target_samples - 1U;
  }
  requested_pre_samples = pre_samples;

  hdma->Init.Mode = DMA_CIRCULAR;
  if (HAL_DMA_Init(hdma) != HAL_OK) {
    return HAL_ERROR;
  }

  flag_half_ready = 0U;
  flag_full_ready = 0U;
  flag_snapshot_done = 0U;
  flag_stream_overrun = 0U;
  next_half_index = 0U;
  dma_wrap_count = 0U;
  snapshot_start_idx = 0U;
  ring_read_count = 0U;
  triggered_capture_active = 1U;

  hdma->XferHalfCpltCallback = NULL;
  hdma->XferCpltCallback = DMA_FullConvCallback;
  DMA1->IFCR = DMA_IFCR_CGIF5;

  if (HAL_DMA_Start_IT(hdma,
                       (uint32_t)&GPIOA->IDR,
                       (uint32_t)capture_buffer,
                       LA_BUFFER_SIZE) != HAL_OK) {
    triggered_capture_active = 0U;
    return HAL_ERROR;
  }

  if (current_mode == MODE_SNAPSHOT) {
    /* Output TIM1 update events on TRGO -> TIM3 ITR0 for exact hardware post-count. */
    TIM1->CR2 = (TIM1->CR2 & ~TIM_CR2_MMS) | TIM_TRGO_UPDATE;
  } else {
    TIM1->CR2 &= ~TIM_CR2_MMS;
  }

  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);

  return HAL_TIM_Base_Start(&htim1);
}

uint32_t LA_Sampling_OnTriggerFired(uint32_t dma_cndtr, uint32_t dma_isr)
{
  uint32_t total_armed = Compute_Ring_Total_Count(dma_cndtr, dma_isr);
  uint32_t actual_pre = (total_armed < requested_pre_samples)
                          ? total_armed
                          : requested_pre_samples;
  uint32_t post_samples = target_samples - actual_pre;

  if (current_mode == MODE_SNAPSHOT) {
    uint32_t trig_pos = total_armed % LA_BUFFER_SIZE;
    snapshot_start_idx = (trig_pos + LA_BUFFER_SIZE - actual_pre) % LA_BUFFER_SIZE;

    if (post_samples == 0U) {
      TIM1->CR1 &= ~TIM_CR1_CEN;
      flag_snapshot_done = 1U;
    } else {
      TIM3->CR1 &= ~TIM_CR1_CEN;
      TIM3->DIER &= ~TIM_DIER_UIE;
      TIM3->CNT = 0U;
      /* ARR = 0 blocks timer counting on STM32F1; requested_pre_samples < target guarantees >= 1. */
      TIM3->ARR = (post_samples > 1U) ? (uint16_t)(post_samples - 1U) : 1U;
      TIM3->EGR = TIM_EGR_UG;
      TIM3->SR = (uint16_t)(~TIM_SR_UIF);
      HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
      TIM3->DIER |= TIM_DIER_UIE;
      TIM3->CR1 |= TIM_CR1_CEN;
    }
  } else {
    /* Rewind read cursor by actual_pre samples; DMA + TIM1 stay running continuously. */
    ring_read_count = total_armed - actual_pre;
  }

  return actual_pre;
}

void LA_Sampling_OnPostTimerIRQ(void)
{
  if ((TIM3->SR & TIM_SR_UIF) != 0U) {
    TIM3->SR = (uint16_t)(~TIM_SR_UIF);
    TIM3->DIER &= ~TIM_DIER_UIE;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    if (triggered_capture_active != 0U && current_mode == MODE_SNAPSHOT) {
      flag_snapshot_done = 1U;
    }
  }
}

uint8_t LA_Sampling_OnDmaIRQ(void)
{
  uint32_t primask;

  if (triggered_capture_active == 0U) {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if ((DMA1->ISR & DMA_ISR_TCIF5) != 0U) {
    DMA1->IFCR = DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5;
    dma_wrap_count++;
  } else {
    DMA1->IFCR = DMA_IFCR_CHTIF5;
  }
  __set_PRIMASK(primask);
  return 1U;
}

HAL_StatusTypeDef LA_Sampling_BeginTimer(void)
{
  return HAL_TIM_Base_Start(&htim1);
}

void LA_Sampling_Stop(void)
{
  DMA_HandleTypeDef *hdma = htim1.hdma[TIM_DMA_ID_UPDATE];

  (void)HAL_TIM_Base_Stop(&htim1);
  Stop_TIM3_Counter();
  TIM1->CR2 &= ~TIM_CR2_MMS;
  __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
  if (hdma != NULL && hdma->State == HAL_DMA_STATE_BUSY) {
    (void)HAL_DMA_Abort(hdma);
  }

  triggered_capture_active = 0U;
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

uint8_t LA_Sampling_CopyNextTriggeredStreamBlock(uint8_t *dst, uint16_t length)
{
  uint32_t primask;
  uint32_t write_count;
  uint32_t available;
  uint32_t start_idx;
  uint32_t first_part;

  if (length == 0U || length > LA_MAX_PAYLOAD_SIZE || dst == NULL) {
    return 2U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  write_count = Read_DMA_Ring_Total_Count();
  __set_PRIMASK(primask);

  available = write_count - ring_read_count;
  if (available > LA_BUFFER_SIZE) {
    return 2U;
  }
  if (available < (uint32_t)length) {
    return 0U;
  }

  start_idx = ring_read_count % LA_BUFFER_SIZE;
  first_part = LA_BUFFER_SIZE - start_idx;
  if (first_part >= (uint32_t)length) {
    memcpy(dst, &capture_buffer[start_idx], length);
  } else {
    memcpy(dst, &capture_buffer[start_idx], first_part);
    memcpy(&dst[first_part], capture_buffer, (uint32_t)length - first_part);
  }

  primask = __get_PRIMASK();
  __disable_irq();
  write_count = Read_DMA_Ring_Total_Count();
  __set_PRIMASK(primask);

  if ((write_count - ring_read_count) > LA_BUFFER_SIZE) {
    return 2U;
  }

  ring_read_count += (uint32_t)length;
  return 1U;
}

void LA_Sampling_CopySnapshotWindow(uint32_t offset, uint8_t *dst, uint16_t length)
{
  uint32_t start_idx;
  uint32_t first_part;

  if (dst == NULL || length == 0U) {
    return;
  }

  start_idx = (snapshot_start_idx + offset) % LA_BUFFER_SIZE;
  first_part = LA_BUFFER_SIZE - start_idx;
  if (first_part >= (uint32_t)length) {
    memcpy(dst, &capture_buffer[start_idx], length);
  } else {
    memcpy(dst, &capture_buffer[start_idx], first_part);
    memcpy(&dst[first_part], capture_buffer, (uint32_t)length - first_part);
  }
}

uint8_t LA_Sampling_IsConfigured(void)
{
  return flag_configured;
}

uint8_t LA_Sampling_IsTriggeredCapture(void)
{
  return triggered_capture_active;
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
