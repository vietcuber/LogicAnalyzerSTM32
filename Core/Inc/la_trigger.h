#ifndef LA_TRIGGER_H
#define LA_TRIGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "la_protocol.h"
#include <stdint.h>

#define LA_NUM_CHANNELS 8U

void LA_Trigger_Init(void);

/**
 * @brief Configure trigger edge mode for a channel (0..7) without arming EXTI yet.
 * @return 1 on valid channel, 0 otherwise.
 */
uint8_t LA_Trigger_Configure(uint8_t channel, TriggerMode_t mode);

/**
 * @brief Unmask EXTI lines and enable NVIC IRQs for all configured trigger channels.
 */
void LA_Trigger_Arm(void);

/**
 * @brief Mask EXTI0..7 lines and disable trigger NVIC IRQs while keeping config intact.
 */
void LA_Trigger_Disarm(void);

uint8_t LA_Trigger_GetActiveMask(void);
uint8_t LA_Trigger_MatchPin(uint16_t gpio_pin);

#ifdef __cplusplus
}
#endif

#endif /* LA_TRIGGER_H */
