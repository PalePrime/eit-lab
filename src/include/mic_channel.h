#ifndef MIC_CHANNEL_H
#define MIC_CHANNEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "channel_controller.h"

usb_channel_t micChannel;

void createMicChannel();

#ifdef __cplusplus
}
#endif

#endif /*MIC_CHANNEL_H*/
