#ifndef UAC2_HANDLING_H
#define UAC2_HANDLING_H

//#include "bsp/board.h"
#include "tusb.h"

#define MIN_SAMPLE_RATE         1000
#define BASE_SAMPLE_RATE       48000
#define MAX_SAMPLE_RATE        96000

#define MAX_OVER_SAMPLE            4

#define DBG_PIN 5
#define PWM_PIN 2
#define ADC_PIN 26

// General parameters for sound, USB side
#define BYTES_PER_SAMPLE           2        // We only do 16 bit integers over USB
#define USB_FRAME_TIME          1000        // 1000 ms per USB frame, USB Full Speed only
#define MAX_SAMPLES_PER_FRAME   (MAX_SAMPLE_RATE / USB_FRAME_TIME)
#define USB_SYNC_FRAMES           16        // Assume we can sync with io side in 16 USB frames
#define USB_LOSS_FRAMES           32        // Assume USB stopped communicating if we lag by 32 USB frames

// General parameters for sound, ADC/PWM side
#define MAX_IO_CHUNK            MAX_SAMPLES_PER_FRAME

#ifdef __cplusplus
extern "C" {
#endif

void createUAC2Handler();
void reportUSB_UAC2();

#ifdef __cplusplus
}
#endif

#endif /*UAC2_HANDLING_H*/
