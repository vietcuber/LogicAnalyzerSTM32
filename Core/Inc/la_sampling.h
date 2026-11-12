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
#define LA_STREAM_PRE_MAX_SAMPLES    5120U
#define LA_SNAPSHOT_FREQ_THRESHOLD   500000U
#define LA_MAX_PRETRIGGER_PCT        90U

typedef enum {
  MODE_STREAMING = 0,
  MODE_SNAPSHOT
} CaptureMode_t;

/**
 * @brief Initialize TIM3 post-trigger slave counter and set IRQ priorities.
 */
void LA_Sampling_Init(void);

/**
 * @brief Validate frequency and sample count, configure TIM1 prescaler/ARR.
 * @return 0 on success, 1 on ERR_INVALID_CONFIG, 2 on ERR_SNAPSHOT_LIMIT.
 */
uint8_t LA_Sampling_Configure(uint32_t freq, uint32_t samples);

/**
 * @brief Prepare DMA and TIM1 registers without starting the TIM1 counter (immediate mode).
 */
HAL_StatusTypeDef LA_Sampling_Prepare(void);

/**
 * @brief Prepare circular DMA + start TIM1 immediately so pre-trigger samples fill while ARMED.
 */
HAL_StatusTypeDef LA_Sampling_PrepareTriggered(uint8_t pre_trigger_pct);

/**
 * @brief Latch DMA ring position on trigger edge and start TIM3 post-sample counter (Snapshot).
 * @return Clamped actual pre-trigger sample index (= trigger_sample_index).
 */
uint32_t LA_Sampling_OnTriggerFired(uint32_t dma_cndtr, uint32_t dma_isr);

/**
 * @brief Handle DMA1 Channel5 TC wrap atomically in triggered circular mode.
 * @return 1 if handled internally (triggered mode), 0 to fall through to HAL_DMA_IRQHandler.
 */
uint8_t LA_Sampling_OnDmaIRQ(void);

/**
 * @brief TIM3 Update ISR handler stopping TIM1 after exact Snapshot post-samples.
 */
void LA_Sampling_OnPostTimerIRQ(void);

/**
 * @brief Start TIM1 counter for immediate (NONE trigger) capture.
 */
HAL_StatusTypeDef LA_Sampling_BeginTimer(void);

void LA_Sampling_Stop(void);

/**
 * @brief Copy the next chronological streaming half-buffer block into dst (immediate mode).
 * @return 0 if next half-buffer is not ready, 1 if copied, 2 on DMA overrun/error.
 */
uint8_t LA_Sampling_CopyNextStreamBlock(uint8_t *dst, uint16_t length);

/**
 * @brief Copy the next chronological streaming block from the 10 KB circular ring (triggered mode).
 * @return 0 if fewer than length samples are available, 1 if copied, 2 on ring overrun/error.
 */
uint8_t LA_Sampling_CopyNextTriggeredStreamBlock(uint8_t *dst, uint16_t length);

/**
 * @brief Copy chronological slice from triggered Snapshot circular buffer starting at offset.
 */
void LA_Sampling_CopySnapshotWindow(uint32_t offset, uint8_t *dst, uint16_t length);

uint8_t LA_Sampling_IsConfigured(void);
uint8_t LA_Sampling_IsTriggeredCapture(void);
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
