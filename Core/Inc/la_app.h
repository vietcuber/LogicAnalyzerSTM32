#ifndef LA_APP_H
#define LA_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
  CAPTURE_STATE_IDLE = 0,
  CAPTURE_STATE_READY,
  CAPTURE_STATE_ARMED,
  CAPTURE_STATE_CAPTURING,
  CAPTURE_STATE_DONE,
  CAPTURE_STATE_ABORTED,
  CAPTURE_STATE_ERROR
} CaptureState_t;

void LA_App_Init(void);
void LA_App_Poll(void);

/**
 * @brief Receive raw USB CDC OUT bytes from ISR context.
 */
void LA_App_OnRxBytes(const uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* LA_APP_H */
