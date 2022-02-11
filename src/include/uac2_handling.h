#ifndef UAC2_HANDLING_H
#define UAC2_HANDLING_H

//#include "bsp/board.h"
#include "tusb.h"

#ifndef AUDIO_SAMPLE_RATE
#define AUDIO_SAMPLE_RATE     48000
#endif

#define PWM_CLOCK_RATE     125000000
#define USB_CLOCK_RATE      48000000
#define MAX_OVER_SAMPLE 4

#define DBG_PIN 5
#define PWM_PIN 2
#define ADC_PIN 26

// General parameters for sound, USB side
#define SAMPLES_PER_FRAME   48              // This is AUDIO_SAMPLE_RATE / USB_FRAME_TIME
#define BYTES_PER_SAMPLE     2              // We only do 16 bit integers over USB
#define USB_FRAME_TIME    1000              // 1000 ms per USB frame, USB Full Spee only
#define USB_SYNC_FRAMES     16              // Assume we can sync with io side in 16 USB frames

// General parameters for sound, ADC/PWM side
#define MAX_IO_CHUNK SAMPLES_PER_FRAME

typedef enum {
  AUDIO_IDLE,
  AUDIO_SYNC,
  AUDIO_RUN,
  AUDIO_ERROR
} audio_state_t;

inline char* stateString(audio_state_t state) {
  switch (state) {
  case AUDIO_IDLE:
    return "Idle";
    break;
  case AUDIO_SYNC:
    return "Sync";
    break;
  case AUDIO_RUN:
    return "Run";
    break;
  default:
    return "Error";
    break;
  }
}



#ifdef __cplusplus
extern "C" {
#endif

void createUAC2Handler();
void reportUSB_UAC2();

#ifdef __cplusplus
}
#endif

#endif /*UAC2_HANDLING_H*/
