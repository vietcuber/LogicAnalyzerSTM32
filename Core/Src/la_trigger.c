#include "la_trigger.h"

#include "main.h"

static TriggerConfig_t trigger_cfg = {
  {
    TRIGGER_NONE, TRIGGER_NONE, TRIGGER_NONE, TRIGGER_NONE,
    TRIGGER_NONE, TRIGGER_NONE, TRIGGER_NONE, TRIGGER_NONE
  },
  0U,
  0U,
  0U,
  TRIGGER_LOGIC_OR
};

static IRQn_Type Channel_To_IRQn(uint8_t channel)
{
  if (channel <= 4U) {
    return (IRQn_Type)((uint32_t)EXTI0_IRQn + (uint32_t)channel);
  }
  return EXTI9_5_IRQn;
}

static void Clear_Trigger_Hardware(void)
{
  EXTI->IMR &= ~0xFFUL;
  EXTI->RTSR &= ~0xFFUL;
  EXTI->FTSR &= ~0xFFUL;
  EXTI->PR = 0xFFUL;
}

static void Configure_Channel_Hardware(uint8_t channel, TriggerMode_t mode)
{
  uint32_t mask = (1UL << channel);
  uint32_t exti_cr_index = (uint32_t)(channel >> 2U);
  uint32_t exti_cr_shift = (uint32_t)((channel & 0x03U) * 4U);

  /* 0b0000 maps EXTIx to GPIOA (PA0..PA7). */
  AFIO->EXTICR[exti_cr_index] &= ~(0xFUL << exti_cr_shift);

  if (mode == TRIGGER_RISING || mode == TRIGGER_EITHER) {
    EXTI->RTSR |= mask;
  }
  if (mode == TRIGGER_FALLING || mode == TRIGGER_EITHER) {
    EXTI->FTSR |= mask;
  }
}

void LA_Trigger_Init(void)
{
  uint8_t ch;

  LA_Trigger_Disarm();
  Clear_Trigger_Hardware();

  for (ch = 0U; ch < LA_NUM_CHANNELS; ch++) {
    trigger_cfg.mode[ch] = TRIGGER_NONE;
    if (ch < 5U) {
      HAL_NVIC_SetPriority(Channel_To_IRQn(ch), 0U, 0U);
    }
  }
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0U, 0U);

  trigger_cfg.active_mask = 0U;
  trigger_cfg.level_mask = 0U;
  trigger_cfg.level_value = 0U;
  trigger_cfg.logic_mode = TRIGGER_LOGIC_OR;
}

uint8_t LA_Trigger_Configure(uint8_t channel, TriggerMode_t mode)
{
  TriggerMode_t modes[LA_NUM_CHANNELS];
  uint8_t ch;

  if (channel >= LA_NUM_CHANNELS) {
    return 0U;
  }

  for (ch = 0U; ch < LA_NUM_CHANNELS; ch++) {
    modes[ch] = TRIGGER_NONE;
  }
  modes[channel] = mode;

  return LA_Trigger_ConfigurePattern(
    (mode == TRIGGER_NONE) ? 0U : (uint8_t)(1U << channel),
    modes,
    0U,
    0U,
    TRIGGER_LOGIC_OR
  );
}

uint8_t LA_Trigger_ConfigurePattern(uint8_t edge_mask,
                                    const TriggerMode_t modes[LA_NUM_CHANNELS],
                                    uint8_t level_mask,
                                    uint8_t level_value,
                                    TriggerLogic_t logic_mode)
{
  uint8_t ch;
  uint8_t active = 0U;

  if (modes == NULL) {
    return 0U;
  }

  LA_Trigger_Disarm();
  Clear_Trigger_Hardware();

  if (edge_mask != 0U) {
    __HAL_RCC_AFIO_CLK_ENABLE();
  }

  for (ch = 0U; ch < LA_NUM_CHANNELS; ch++) {
    uint8_t bit = (uint8_t)(1U << ch);
    if ((edge_mask & bit) != 0U && modes[ch] != TRIGGER_NONE) {
      Configure_Channel_Hardware(ch, modes[ch]);
      trigger_cfg.mode[ch] = modes[ch];
      active |= bit;
    } else {
      trigger_cfg.mode[ch] = TRIGGER_NONE;
    }
  }

  trigger_cfg.active_mask = active;
  trigger_cfg.level_mask = level_mask;
  trigger_cfg.level_value = (uint8_t)(level_value & level_mask);
  trigger_cfg.logic_mode = logic_mode;
  return 1U;
}

void LA_Trigger_Arm(void)
{
  uint8_t ch;
  uint8_t mask = trigger_cfg.active_mask;

  if (mask == 0U) {
    return;
  }

  /* Keep EXTI masked while preparing NVIC so no edge is dropped by ClearPendingIRQ. */
  EXTI->IMR &= ~0xFFUL;

  for (ch = 0U; ch < 5U; ch++) {
    if ((mask & (1U << ch)) != 0U) {
      IRQn_Type irqn = Channel_To_IRQn(ch);
      HAL_NVIC_ClearPendingIRQ(irqn);
      HAL_NVIC_EnableIRQ(irqn);
    }
  }

  if ((mask & 0xE0U) != 0U) {
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  }

  EXTI->PR = (uint32_t)mask;
  EXTI->IMR |= (uint32_t)mask;
}

void LA_Trigger_DisarmImmediate(void)
{
  EXTI->IMR &= ~0xFFUL;
  EXTI->PR = 0xFFUL;
}

void LA_Trigger_Disarm(void)
{
  uint8_t ch;

  LA_Trigger_DisarmImmediate();

  for (ch = 0U; ch < 5U; ch++) {
    IRQn_Type irqn = Channel_To_IRQn(ch);
    HAL_NVIC_DisableIRQ(irqn);
    HAL_NVIC_ClearPendingIRQ(irqn);
  }
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
}

uint8_t LA_Trigger_GetActiveMask(void)
{
  return trigger_cfg.active_mask;
}

uint8_t LA_Trigger_EvaluateMatch(uint16_t exti_pending_mask, uint8_t gpio_idr)
{
  uint8_t matched_edges = (uint8_t)(exti_pending_mask & (uint16_t)trigger_cfg.active_mask);

  if (matched_edges == 0U) {
    return 0U;
  }

  if (trigger_cfg.logic_mode == TRIGGER_LOGIC_AND && trigger_cfg.level_mask != 0U) {
    if ((gpio_idr & trigger_cfg.level_mask) != trigger_cfg.level_value) {
      return 0U;
    }
  }

  return 1U;
}

uint8_t LA_Trigger_MatchPin(uint16_t gpio_pin)
{
  return LA_Trigger_EvaluateMatch(gpio_pin, 0U);
}
