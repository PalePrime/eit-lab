#ifndef CHANNEL_CONTROLLER_H
#define CHANNEL_CONTROLLER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "program_config.h"
#include "program_state.h"
#include "uac2_handling.h"

#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  CH_DATA_RECEIVED,
  CH_DATA_SENT,
  CH_SET_RATE
} ch_ctrl_t;

typedef struct {
  ch_ctrl_t cmd;
  uint64_t time;
  uint64_t count;
} ch_ctrl_msg_t;

#define CH_CTRL_MSG_SIZE sizeof(ch_ctrl_msg_t)
#define CH_CTRL_MSG_Q_LEN 4

// This is the full state for the complete channel
// USB <-> ADC/PWM for Mic/Spk
// The values in this struct will be updated only by a
// single task and all individual updates are atomic
// 32-bit writes, so no mutex required
typedef struct {
  // Updated from controller task
  audio_state_t state;
  uint32_t ioChunk;
  uint32_t usbChunk;
  uint32_t clkDiv;
  int32_t queuedSamples;
  int32_t minQueuedSamples;
  int32_t maxQueuedSamples;
  uint32_t receiveCalls;
  uint32_t sendCalls;
  uint32_t sampleRate;
  uint32_t oversampling;
  // Updated from CODEC task
  uint32_t offset;
  // Updated from ISR
  uint32_t isrCtrlFails;
  // Updated from USB task
  uint32_t usbCtrlFails;
  uint32_t overRuns;
  uint32_t underRuns;
} usb_channel_state_t;

// This is the interface a channel needs to
// implement. It is created at initialization
// and constitutes the API used by the controller
typedef struct {
  const char *idStr;
  bool toUsb;
  uint32_t baseClock;
  void (*init)(void);
  void (*setRate)(usb_channel_state_t *, uint32_t, uint32_t);
  void (*setDiv)(uint32_t);
  void (*open)(void);
  void (*close)(void);
  state_register_t progStateReg; 
} usb_channel_settings_t;

// This represents the channel, i.e. the
// combination of its API, state and controller
// task
typedef struct {
  usb_channel_state_t state;
  // Fields below are private to the channel controller
  usb_channel_settings_t *settings;
  StackType_t   stack[CH_STACK_SIZE];
  char          taskname[10];
  StaticTask_t  taskdef;
  TaskHandle_t  handle;
  QueueHandle_t cmd_q;
  StaticQueue_t cmd_q_def;
  uint8_t       cmd_q_buf[CH_CTRL_MSG_Q_LEN * CH_CTRL_MSG_SIZE];
} usb_channel_t;

inline static void cloneChannelState(usb_channel_state_t *dest, const usb_channel_state_t *src) {
  memcpy(dest, src, sizeof(usb_channel_state_t));
}

inline static uint32_t controlMsg(usb_channel_t *channel, ch_ctrl_t cmd, uint64_t value) {
  ch_ctrl_msg_t msg = {
    .cmd = cmd,
    .count = value,
    .time = to_us_since_boot(get_absolute_time())
  };
  return xQueueSendToBack(channel->cmd_q, &msg, 0);
}

inline static uint32_t controlMsgFromISR(usb_channel_t *channel, ch_ctrl_t cmd, uint64_t value) {
  ch_ctrl_msg_t msg = {
    .cmd = cmd,
    .count = value,
    .time = to_us_since_boot(get_absolute_time())
  };
  return xQueueSendToBackFromISR(channel->cmd_q, &msg, 0);
}

void newChannelController(usb_channel_settings_t *channelSettings, usb_channel_t *channel);

#ifdef __cplusplus
}
#endif

#endif /*CHANNEL_CONTROLLER_H*/
