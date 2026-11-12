#include "la_trigger.h"

#include "main.h"

static TriggerMode_t trigger_mode[LA_NUM_CHANNELS] = {
  TRIGGER_NONE, TRIGGER_NONE, TRIGGER_NONE, TRIGGER_NONE,
  TRIGGER_NONE, TRIGGER_NONE, TRIGGER_NONE, TRIGGER_NONE
};
static volatile uint8_t trigger_active_mask = 0U;

static IRQn_Type Channel_To_IRQn(uint8_t channel)
{
  if (channel <= 4U) {
    return (IRQn_Type)((uint32_t)EXTI0_IRQn + (uint32_t)channel);
  }
  return EXTI9_5_IRQn;
}

void LA_Trigger_Init(void)
{
  uint8_t ch;

  EXTI->EMR &= ~0xFFUL;
  EXTI->RTSR &= ~0xFFUL;
  EXTI->FTSR &= ~0xFFUL;
  for (ch = 0U; ch < LA_NUM_CHANNELS; ch++) {
    trigger_mode[ch] = TRIGGER_NONE;
  }
  trigger_active_mask = 0U;
  LA_Trigger_Disarm();
}

uint8_t LA_Trigger_Configure(uint8_t channel, TriggerMode_t mode)
{
  uint32_t mask;
  uint32_t exti_cr_index;
  uint32_t exti_cr_shift;

  if (channel >= LA_NUM_CHANNELS) {
    return 0U;
  }

  /* Clear all channels first so switching channels or applying NONE cannot
     leave a previously configured channel armed in trigger_active_mask. */
  LA_Trigger_Init();

  if (mode == TRIGGER_NONE) {
    return 1U;
  }

  mask = (1UL << channel);
  __HAL_RCC_AFIO_CLK_ENABLE();

  exti_cr_index = (uint32_t)(channel >> 2U);
  exti_cr_shift = (uint32_t)((channel & 0x03U) * 4U);
  /* 0b0000 maps EXTIx to GPIOA (PA0..PA7). */
  AFIO->EXTICR[exti_cr_index] &= ~(0xFUL << exti_cr_shift);

  if (mode == TRIGGER_RISING || mode == TRIGGER_EITHER) {
    EXTI->RTSR |= mask;
  }
  if (mode == TRIGGER_FALLING || mode == TRIGGER_EITHER) {
    EXTI->FTSR |= mask;
  }

  trigger_mode[channel] = mode;
  trigger_active_mask = (uint8_t)mask;
  return 1U;
}

void LA_Trigger_Arm(void)
{
  uint8_t ch;
  uint8_t mask = trigger_active_mask;

  if (mask == 0U) {
    return;
  }

  EXTI->PR = (uint32_t)mask;
  EXTI->IMR |= (uint32_t)mask;

  for (ch = 0U; ch < 5U; ch++) {
    if ((mask & (1U << ch)) != 0U) {
      IRQn_Type irqn = Channel_To_IRQn(ch);
      HAL_NVIC_ClearPendingIRQ(irqn);
      HAL_NVIC_SetPriority(irqn, 0U, 0U);
      HAL_NVIC_EnableIRQ(irqn);
    }
  }

  if ((mask & 0xE0U) != 0U) {
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  }
}

void LA_Trigger_Disarm(void)
{
  uint8_t ch;

  EXTI->IMR &= ~0xFFUL;
  EXTI->PR = 0xFFUL;

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
  return trigger_active_mask;
}

uint8_t LA_Trigger_MatchPin(uint16_t gpio_pin)
{
  uint32_t pin_mask = (uint32_t)gpio_pin & 0xFFUL;
  uint8_t channel;

  if (pin_mask == 0U) {
    return 0U;
  }

  channel = (uint8_t)__builtin_ctz((unsigned int)pin_mask);
  if (channel >= LA_NUM_CHANNELS) {
    return 0U;
  }

  if ((trigger_active_mask & (1U << channel)) == 0U ||
      trigger_mode[channel] == TRIGGER_NONE) {
    return 0U;
  }

  return 1U;
}
