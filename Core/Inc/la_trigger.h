#ifndef LA_TRIGGER_H
#define LA_TRIGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "la_protocol.h"
#include <stdint.h>

#define LA_NUM_CHANNELS 8U

typedef enum {
  TRIGGER_LOGIC_OR = 0,
  TRIGGER_LOGIC_AND
} TriggerLogic_t;

typedef struct {
  TriggerMode_t mode[LA_NUM_CHANNELS];
  uint8_t active_mask;
  uint8_t level_mask;
  uint8_t level_value;
  TriggerLogic_t logic_mode;
} TriggerConfig_t;

void LA_Trigger_Init(void);

/**
 * @brief Configure single-channel edge trigger (0..7) without arming EXTI yet.
 *        Clears any prior channel selection to match single-channel UI control.
 * @return 1 on valid channel, 0 otherwise.
 */
uint8_t LA_Trigger_Configure(uint8_t channel, TriggerMode_t mode);

/**
 * @brief Configure multi-channel edge + level pattern trigger without arming EXTI yet.
 *        Edge-capable channels arm EXTI; optional level_mask/level_value supports AND logic.
 * @return 1 on valid configuration, 0 otherwise.
 */
uint8_t LA_Trigger_ConfigurePattern(uint8_t edge_mask,
                                    const TriggerMode_t modes[LA_NUM_CHANNELS],
                                    uint8_t level_mask,
                                    uint8_t level_value,
                                    TriggerLogic_t logic_mode);

/**
 * @brief Unmask EXTI lines and enable NVIC IRQs in glitch-free order.
 */
void LA_Trigger_Arm(void);

/**
 * @brief Fast EXTI mask/clear suitable for ISR critical path.
 */
void LA_Trigger_DisarmImmediate(void);

/**
 * @brief Mask EXTI0..7 lines and disable trigger NVIC IRQs while keeping config intact.
 */
void LA_Trigger_Disarm(void);

uint8_t LA_Trigger_GetActiveMask(void);

/**
 * @brief Evaluate whether pending EXTI pins (and optional GPIO IDR pattern) satisfy trigger.
 */
uint8_t LA_Trigger_EvaluateMatch(uint16_t exti_pending_mask, uint8_t gpio_idr);

/**
 * @brief Legacy helper evaluating edge-pin match only.
 */
uint8_t LA_Trigger_MatchPin(uint16_t gpio_pin);

#ifdef __cplusplus
}
#endif

#endif /* LA_TRIGGER_H */
