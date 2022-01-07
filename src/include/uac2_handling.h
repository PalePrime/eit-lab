#ifndef UAC2_HANDLING_H
#define UAC2_HANDLING_H

#include "bsp/board.h"
#include "tusb.h"

#ifndef AUDIO_SAMPLE_RATE
#define AUDIO_SAMPLE_RATE     48000
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum
{
  VOLUME_CTRL_0_DB = 0,
  VOLUME_CTRL_10_DB = 2560,
  VOLUME_CTRL_20_DB = 5120,
  VOLUME_CTRL_30_DB = 7680,
  VOLUME_CTRL_40_DB = 10240,
  VOLUME_CTRL_50_DB = 12800,
  VOLUME_CTRL_60_DB = 15360,
  VOLUME_CTRL_70_DB = 17920,
  VOLUME_CTRL_80_DB = 20480,
  VOLUME_CTRL_90_DB = 23040,
  VOLUME_CTRL_100_DB = 25600,
  VOLUME_CTRL_SILENCE = 0x8000,
};

void createUAC2Handler();
void reportUSB_UAC2();

#ifdef __cplusplus
}
#endif

#endif /*UAC2_HANDLING_H*/
