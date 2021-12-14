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

typedef struct TU_ATTR_PACKED
{
  union
  {
    struct TU_ATTR_PACKED
    {
      uint8_t recipient :  5; ///< Recipient type tusb_request_recipient_t.
      uint8_t type      :  2; ///< Request type tusb_request_type_t.
      uint8_t direction :  1; ///< Direction type. tusb_dir_t
    } bmRequestType_bit;

    uint8_t bmRequestType;
  };

  uint8_t bRequest;  ///< Request type audio_cs_req_t
  uint8_t bChannelNumber;
  uint8_t bControlSelector;
  union
  {
    uint8_t bInterface;
    uint8_t bEndpoint;
  };
  uint8_t bEntityID;
  uint16_t wLength;
} audio_control_request_t;

void createUAC2Handler();
void reportUSB_UAC2();

#ifdef __cplusplus
}
#endif

#endif /*UAC2_HANDLING_H*/
