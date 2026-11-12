#ifndef LA_SAMPLING_H
#define LA_SAMPLING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define LA_BUFFER_SIZE               10240U
#define LA_SNAPSHOT_MAX_SAMPLES      10000U
#define LA_STREAMING_MAX_SAMPLES     100000U
#define LA_SNAPSHOT_FREQ_THRESHOLD   500000U

typedef enum {
  MODE_STREAMING = 0,
  MODE_SNAPSHOT
} CaptureMode_t;

/**
 * @brief Validate frequency and sample count, configure TIM1 prescaler/ARR.
 * @return 0 on success, 1 on ERR_INVALID_CONFIG, 2 on ERR_SNAPSHOT_LIMIT.
 */
uint8_t LA_Sampling_Configure(uint32_t freq, uint32_t samples);

/**
 * @brief Prepare DMA and TIM1 registers without starting the TIM1 counter.
 */
HAL_StatusTypeDef LA_Sampling_Prepare(void);

/**
 * @brief Start TIM1 counter so DMA begins sampling PA0..PA7.
 *        Safe to invoke from EXTI ISR or main loop after LA_Sampling_Prepare().
 */
HAL_StatusTypeDef LA_Sampling_BeginTimer(void);

void LA_Sampling_Stop(void);

/**
 * @brief Copy the next chronological streaming half-buffer block into dst.
 * @return 0 if next half-buffer is not ready, 1 if copied, 2 on DMA overrun/error.
 */
uint8_t LA_Sampling_CopyNextStreamBlock(uint8_t *dst, uint16_t length);

uint8_t LA_Sampling_IsConfigured(void);
CaptureMode_t LA_Sampling_GetMode(void);
uint32_t LA_Sampling_GetTargetSamples(void);
uint16_t LA_Sampling_GetStreamBlockSize(void);
uint8_t LA_Sampling_IsSnapshotDone(void);
void LA_Sampling_ClearSnapshotDone(void);
uint8_t LA_Sampling_HasStreamOverrun(void);
const uint8_t *LA_Sampling_GetBuffer(void);

#ifdef __cplusplus
}
#endif

#endif /* LA_SAMPLING_H */
