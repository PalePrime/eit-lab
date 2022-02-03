#ifndef SPK_CHANNEL_H
#define SPK_CHANNEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "channel_controller.h"

usb_channel_t spkChannel;

void createSpkChannel();

#ifdef __cplusplus
}
#endif

#endif /*SPK_CHANNEL_H*/
