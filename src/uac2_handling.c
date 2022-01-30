#include "FreeRTOS.h"
#include "task.h"

#include "bsp/board.h"
#include "tusb.h"

#include "program_config.h"
#include "uac2_handling.h"
#include "usb_descriptors.h"
#include "program_state.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/structs/usb.h"
#include "pico/stdlib.h"

static StackType_t  uac2_out_stack[UAC2_OUT_STACK_SIZE];
static StaticTask_t uac2_out_taskdef;
static TaskHandle_t uac2_out_handle;

static StackType_t  uac2_in_stack[UAC2_IN_STACK_SIZE];
static StaticTask_t uac2_in_taskdef;
static TaskHandle_t uac2_in_handle;

static StackType_t  spk_ch_stack[SPK_CH_STACK_SIZE];
static StaticTask_t spk_ch_taskdef;
static TaskHandle_t spk_ch_handle;

static StackType_t  mic_ch_stack[MIC_CH_STACK_SIZE];
static StaticTask_t mic_ch_taskdef;
static TaskHandle_t mic_ch_handle;

#define DBG_PIN 5
#define PWM_PIN 2
#define ADC_PIN 26

static uint8_t pinState = 0;

// These are computed at program start and then remain constant
static uint   pwmSlice;
static uint   pwmDmaCh;
static uint   adcDmaCh;
static volatile uint32_t * pwmDataPtr;
static volatile uint32_t * pwmDivPtr;
static volatile uint32_t * adcDataPtr;
static volatile uint32_t * adcDivPtr;

static volatile uint32_t audioSampleRate;

typedef enum {
  AUDIO_IDLE,
  AUDIO_SYNC,
  AUDIO_RUN,
  AUDIO_ERROR
} audio_state_t;

char* stateString(audio_state_t state) {
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

// General buffer parameters for sound
#define SAMPLES_PER_FRAME 48
#define BYTES_PER_SAMPLE 2

// Buffer for incoming sound data
#define MAX_IO_CHUNK SAMPLES_PER_FRAME

static int16_t micBuf[2 * MAX_IO_CHUNK];    // Double buffer incoming data from ADC 
static int16_t zeroBuf[MAX_IO_CHUNK];

// Buffer for outgoing sound data
// #define SPK_CHUNKS 16

static int16_t spkBuf[2 * MAX_IO_CHUNK];

// The values in this struct will be updated only by a
// single task and all individual updates are atomic
// 32-bit writes, so no mutex required
typedef struct {
  // Updated from controller task
  audio_state_t state;
  uint32_t ioChunk;
  uint32_t clkDiv;
  int32_t queuedSamples;
  int32_t minQueuedSamples;
  int32_t maxQueuedSamples;
  uint32_t receiveCalls;
  uint32_t sendCalls;
  // Updated from ISR
  uint32_t isrCtrlFails;
  // Updated from USB task
  uint32_t usbCtrlFails;
} usb_channel_t;

static usb_channel_t micCh = {
  .state = AUDIO_IDLE,
  .ioChunk = SAMPLES_PER_FRAME / 2
};

static usb_channel_t spkCh = {
  .state = AUDIO_IDLE,
  .ioChunk = SAMPLES_PER_FRAME / 2
};

typedef enum {
  CH_DATA_RECEIVED,
  CH_DATA_SENT,
  CH_SET_RATE
} ch_cmd_t;

typedef struct {
  ch_cmd_t cmd;
  uint64_t time;
  uint64_t count;
} ch_cmd_msg_t;

#define CH_SYNC_TIME 16000
#define CH_USB_FRAME_TIME 1000
#define CH_MSG_SIZE sizeof(ch_cmd_msg_t)
#define CH_MSG_Q_LEN 4

static StaticQueue_t mic_cmd_q_def;
static uint8_t       mic_cmd_q_buf[CH_MSG_Q_LEN*CH_MSG_SIZE];
static QueueHandle_t mic_cmd_q;

static StaticQueue_t spk_cmd_q_def;
static uint8_t       spk_cmd_q_buf[CH_MSG_Q_LEN*CH_MSG_SIZE];
static QueueHandle_t spk_cmd_q;

inline static uint32_t controlMsg(QueueHandle_t queue, ch_cmd_t cmd, uint64_t value) {
  ch_cmd_msg_t msg = {
    .cmd = cmd,
    .count = value,
    .time = to_us_since_boot(get_absolute_time())
  };
  return xQueueSendToBack(queue, &msg, 0);
}

static volatile int64_t debugInfo[DEBUG_INFO_COUNT];

void usbDebugInc(uint32_t index) {
  debugInfo[index]++;
}

void usbDebugSet(uint32_t index, int64_t value) {
  debugInfo[index] = value;
}

void usbDebugMax(uint32_t index, int64_t value) {
  if (value > debugInfo[index]) {
    debugInfo[index] = value;
  }
}

void reportUSB_UAC2() {

  printf("Audio spk, mic:       %15s %15s\n", stateString(spkCh.state), stateString(micCh.state));
  
  printf("Spk frames in, out:   %15lu %15lu\n",  spkCh.receiveCalls, spkCh.sendCalls);
  printf("Spk Q-len, clkDiv:    %15li %15lu\n",  spkCh.queuedSamples, spkCh.clkDiv);
  printf("Spk Q-len min, max:   %15li %15lu\n",  spkCh.minQueuedSamples, spkCh.maxQueuedSamples);
  printf("Spk C-fails usb, dma: %15lu %15lu\n",  spkCh.usbCtrlFails, spkCh.isrCtrlFails);

  printf("Mic frames in, out:   %15lu %15lu\n",  micCh.receiveCalls, micCh.sendCalls);
  printf("Mic Q-len, clkDiv:    %15li %15lu\n",  micCh.queuedSamples, micCh.clkDiv);
  printf("Mic Q-len min, max:   %15li %15lu\n",  micCh.minQueuedSamples, micCh.maxQueuedSamples);
  printf("Mic C-fails usb, dma: %15lu %15lu\n",  micCh.usbCtrlFails, micCh.isrCtrlFails);

  printf("Debug info:           ");
  uint32_t dbgIdx=0;
  while (dbgIdx<DEBUG_INFO_COUNT) {
    printf("%15lli ", debugInfo[dbgIdx++]);
    if (!(dbgIdx & 0x3u)) {
      printf("\n                      ");
    }
  }
  printf("\n");

}

static void pwmDmaHandler() {
  static uint32_t base = 0;
  BaseType_t taskWoken = pdFALSE;
  ch_cmd_msg_t msg = {
    .cmd = CH_DATA_SENT,
    .count = spkCh.ioChunk,
    .time = to_us_since_boot(get_absolute_time())
  };
  uint32_t lastBase = base;
  if (base == 0) {
    base = spkCh.ioChunk;
  } else {
    base = 0;
  }
  if (spkCh.state != AUDIO_IDLE) {
    dma_channel_set_read_addr(pwmDmaCh, &spkBuf[base], true);
    if (xQueueSendToBackFromISR(spk_cmd_q, &msg, NULL) != pdPASS) {
      spkCh.isrCtrlFails++;
    }
    xTaskNotifyFromISR(uac2_out_handle, lastBase, eSetValueWithOverwrite, &taskWoken);
    portYIELD_FROM_ISR(taskWoken);
  }
  dma_hw->ints0 = 1u << pwmDmaCh;
}

static void uac2OutTask(void *pvParameters) {

  //uint16_t pwmDiv = initialSpkDiv(audioSampleRate, spkChSettings.baseClock);

  // Set up PWM pin for audio out
  gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
  pwmSlice = pwm_gpio_to_slice_num(PWM_PIN);
  pwm_set_wrap(pwmSlice, 255);
  pwmDataPtr = &pwm_hw->slice[pwmSlice].cc;
  *pwmDataPtr = 16;
  pwmDivPtr = &pwm_hw->slice[pwmSlice].div;
  //*pwmDivPtr = pwmDiv >> 12;
  //pwm_set_enabled(pwmSlice, true);

  
  // Set up DMA channel for audio out
  pwmDmaCh = dma_claim_unused_channel(true);
  dma_channel_config c1 = dma_channel_get_default_config(pwmDmaCh);
  channel_config_set_transfer_data_size(&c1, DMA_SIZE_16);
  channel_config_set_write_increment(&c1, false);
  channel_config_set_read_increment(&c1, true);
  channel_config_set_dreq(&c1, DREQ_PWM_WRAP1);

  dma_channel_configure(
    pwmDmaCh,
    &c1,
    pwmDataPtr,       // Write address
    spkBuf,           // Read address
    spkCh.ioChunk,
    false             // Start
  );

  dma_channel_set_irq0_enabled(pwmDmaCh, false);
  dma_channel_set_irq1_enabled(pwmDmaCh, true);
  irq_set_exclusive_handler(DMA_IRQ_1, pwmDmaHandler);
  irq_set_enabled(DMA_IRQ_1, true);

  int16_t buf[MAX_IO_CHUNK];
  while(true) {
    uint32_t base = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uint32_t availableSamples = tud_audio_available() >> 1;
    if (availableSamples >= spkCh.ioChunk) {
      tud_audio_read(buf, spkCh.ioChunk << 1);
    } else {
      memset(buf, 0, spkCh.ioChunk << 1);
      usbDebugInc(16);
    }
    for (uint32_t i=0; i<spkCh.ioChunk; i++) {
      int16_t sample = buf[i];
      sample = sample>>8;           // -128 -- +127 in low byte, sign in high byte
      sample = (sample+128) & 0xff; //    0 --  255 in low byte, zero in high byte
      spkBuf[base + i] = sample;
    }
  }
}

/*
    if (dmaRunning && dmaCount > 16 && (frameCount & 0x000fu)==8) {
      pwmLagTime = pwmTotalTime - usbTotalTime ;
      if (pwmLagTime<pwmMinLagTime) {
        pwmMinLagTime = pwmLagTime;
      } else if (pwmLagTime>pwmMaxLagTime) {
        pwmMaxLagTime = pwmLagTime;
      }

      pwmAccLagTime += pwmLagTime;

      if (pwmAccLagTime>4000) {
        pwmAccLagTime = 4000;
      } else if (pwmAccLagTime<-4000) {
        pwmAccLagTime = -4000;
      }

      int32_t fiveLagTime = (pwmLagTime<<2) + (pwmLagTime);
      adjust = ((adjust<<2) - adjust + fiveLagTime + (pwmAccLagTime>>3)) >> 2;
      pwmDiv = PWM_DIV_INIT - adjust;
      
    }
*/

typedef struct {
  usb_channel_t *ch;
  QueueHandle_t *queue;
  const char *idStr;
  bool toUsb;
  bool debug;
  uint32_t baseClock;
  void (*init)(void);
  uint32_t (*initialDiv)(uint32_t, uint32_t);
  void (*setDiv)(uint32_t);
  void (*open)(void);
  void (*close)(void);
} usb_channel_settings_t;

static void initSpkCh() {
  pwm_set_enabled(pwmSlice, true);
}

static void openSpkCh() {
  dma_channel_set_read_addr(pwmDmaCh, spkBuf, true);
}

static void closeSpkCh() {
  dma_channel_abort(pwmDmaCh);
}

uint32_t initialSpkDiv(uint32_t sampleRate, uint32_t baseRate) {
  return ((baseRate + (sampleRate >> 1)) / sampleRate) + (1 << 4);
}

static void setSpkDiv(uint32_t clockDiv) {
  *pwmDivPtr = (clockDiv >> 4);
}

static usb_channel_settings_t spkChSettings = {
  .ch = &spkCh,
  .queue = &spk_cmd_q,
  .idStr = "Spk",
  .toUsb = false,
  .debug = true,
  .baseClock = 125000000,
  .init = initSpkCh,
  .initialDiv = initialSpkDiv,
  .setDiv = setSpkDiv,
  .open = openSpkCh,
  .close = closeSpkCh
};

static void initMicCh() {
}

static void openMicCh() {
  tu_fifo_t* ff = tud_audio_get_ep_in_ff();
  tu_fifo_clear(ff);
  tud_audio_write(zeroBuf, micCh.ioChunk);
  dma_channel_set_write_addr(adcDmaCh, micBuf, true);
  adc_run(true);
}

static void closeMicCh() {
  dma_channel_abort(adcDmaCh);
  adc_run(false);
}

uint32_t initialMicDiv(uint32_t sampleRate, uint32_t baseRate) {
  float clockDiv = ((float)baseRate / (float)sampleRate) - 1.0;
  adc_set_clkdiv(clockDiv); // Example: 48 MHz / (999+1) => 48 kHz
  return 0;
}

void setMicDiv(uint32_t clockDiv) {
  
}

static usb_channel_settings_t micChSettings = {
  .ch = &micCh,
  .queue = &mic_cmd_q,
  .idStr = "Spk",
  .toUsb = true,
  .debug = false,
  .baseClock = 48000000,
  .init = initMicCh,
  .initialDiv = initialMicDiv,
  .setDiv = setMicDiv,
  .open = openMicCh,
  .close = closeMicCh
};

static void adcDmaHandler() {
  static uint32_t base = 0;
  BaseType_t taskWoken = pdFALSE;
  ch_cmd_msg_t msg = {
    .cmd = CH_DATA_RECEIVED,
    .count = micCh.ioChunk,
    .time = to_us_since_boot(get_absolute_time())
  };
  uint32_t lastBase = base;
  if (base == 0) {
    base = micCh.ioChunk;
  } else {
    base = 0;
  }
  if (micCh.state != AUDIO_IDLE) {
    dma_channel_set_write_addr(adcDmaCh, &micBuf[base], true);
    if (xQueueSendToBackFromISR(mic_cmd_q, &msg, NULL) != pdPASS) {
      micCh.isrCtrlFails++;
    }
    xTaskNotifyFromISR(uac2_in_handle, lastBase, eSetValueWithOverwrite, &taskWoken);
    portYIELD_FROM_ISR(taskWoken);
  }
  dma_hw->ints0 = 1u << adcDmaCh;
}

static void uac2InTask(void *pvParameters) {
  // Set up DMA channel for audio in
  adcDmaCh = dma_claim_unused_channel(true);
  dma_channel_config c2 = dma_channel_get_default_config(adcDmaCh);
  channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);
  channel_config_set_write_increment(&c2, true);
  channel_config_set_read_increment(&c2, false);
  channel_config_set_dreq(&c2, DREQ_ADC);

  dma_channel_configure(
    adcDmaCh,
    &c2,
    micBuf,         // Write address
    adcDataPtr,     // Read address
    micCh.ioChunk,  // Words to transfer
    false           // Start
  );

  dma_channel_set_irq0_enabled(adcDmaCh, true);
  dma_channel_set_irq1_enabled(adcDmaCh, false);
  irq_set_exclusive_handler(DMA_IRQ_0, adcDmaHandler);
  irq_set_enabled(DMA_IRQ_0, true);

  int32_t high = INT16_MIN;
  int32_t low  = INT16_MAX;
  while(true) {
    uint32_t base = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int32_t sum = 0;
    for (uint32_t i=0; i<micCh.ioChunk; i++) {
      int32_t sample = ((micBuf[base + i] & 0x0fff) - 2048) << 4;
      sum += sample;
      if (sample>high) {
        high = sample;
      }
      if (sample<low) {
        low = sample;
      }
      micBuf[base + i] = sample;
    }
    tud_audio_write(&micBuf[base], micCh.ioChunk << 1);
  }
}

static void startChannel(usb_channel_t *ch) {
  ch->isrCtrlFails = 0;
  ch->usbCtrlFails = 0;
  ch->receiveCalls = 0;
  ch->sendCalls = 0;
  ch->queuedSamples = 0;
  ch->maxQueuedSamples = -10000;
  ch->minQueuedSamples =  10000;
  ch->state = AUDIO_SYNC;
}

static void chController(void *pvParameters) {
  usb_channel_settings_t *p = (usb_channel_settings_t *) pvParameters;
  QueueHandle_t q = *(p->queue);
  usb_channel_t *ch = p->ch;
  ch_cmd_msg_t cmd;
  //int64_t resetTime = to_us_since_boot(get_absolute_time());
  int64_t recTime = 0;
  int64_t sendTime = 0;
  int32_t sampleRate = AUDIO_SAMPLE_RATE;
  uint64_t recSamples = 0;
  uint64_t sendSamples = 0;
  
  ch->clkDiv = (p->initialDiv)(AUDIO_SAMPLE_RATE, p->baseClock);
  (p->setDiv)(ch->clkDiv);
  (p->init)();
  while (true) {
    if (xQueueReceive(q, &cmd, portMAX_DELAY) == pdTRUE) {
      switch (cmd.cmd) {
        case CH_SET_RATE:
          ch->clkDiv = (p->initialDiv)(cmd.count, p->baseClock);
          (p->setDiv)(ch->clkDiv);
          sampleRate = cmd.count;
          (p->close)();
          ch->state = AUDIO_IDLE;
          break;
        case CH_DATA_RECEIVED:
          if (!p->toUsb && ch->state == AUDIO_IDLE) {
            // USB sends data, start channel
            //resetTime = cmd.time - CH_USB_FRAME_TIME;
            sendTime = cmd.time;
            recSamples = 0;
            sendSamples = 0;
            startChannel(ch);
            (p->open)();
          }
          ch->receiveCalls++;
          recTime = cmd.time;
          recSamples += cmd.count;
          break;
        case CH_DATA_SENT:
          if (p->toUsb && ch->state == AUDIO_IDLE) {
            // USB wants data, start channel
            //resetTime = cmd.time - CH_USB_FRAME_TIME;
            recTime = cmd.time;
            recSamples = 0;
            sendSamples = 0;
            startChannel(ch);
            (p->open)();
          }
          ch->sendCalls++;
          sendTime = cmd.time;
          sendSamples += cmd.count;
          break;
      }
      if (ch->state == AUDIO_SYNC && recTime > CH_SYNC_TIME && sendTime > CH_SYNC_TIME) {
        ch->state = AUDIO_RUN;
      }
      if (ch->state != AUDIO_IDLE) {
        int32_t currentLag = recSamples - sendSamples;
        int32_t estimatedTimeLag = (sampleRate * (sendTime - recTime)) >> 20;
        ch->queuedSamples = currentLag + estimatedTimeLag;
        if (cmd.cmd == CH_DATA_RECEIVED && ((ch->receiveCalls) & 0x1f) == 7 || cmd.cmd == CH_DATA_SENT && ((ch->sendCalls) & 0x1f) == 7) {
          //printf("%li %li %li %li %li\n", currentLag, estimatedTimeLag, samplesInQueue, ch->isrCtrlFails, ch->usbCtrlFails);
        }
        if (ch->queuedSamples > ch->maxQueuedSamples) {
          ch->maxQueuedSamples = ch->queuedSamples;
        }
        if (ch->queuedSamples < ch->minQueuedSamples) {
          ch->minQueuedSamples = ch->queuedSamples;
        }
        if (!p->toUsb) {
          if (ch->queuedSamples < 50) {
            (p->setDiv)(ch->clkDiv + (1 << 4));
          } else if (ch->queuedSamples < 100) {
            (p->setDiv)(ch->clkDiv);
          } else if (ch->queuedSamples < 150) {
            (p->setDiv)(ch->clkDiv - (1 << 4));
          } else {
            (p->setDiv)(ch->clkDiv - (2 << 4));
          }
        }
        if (p->toUsb && recTime > sendTime + CH_SYNC_TIME ||
            !p->toUsb && sendTime > recTime + CH_SYNC_TIME) {
          // USB not active, stop channel
          (p->close)();
          ch->state = AUDIO_IDLE;
        }
      }
    }
  }
}

void createUAC2Handler() {
  gpio_set_function(DBG_PIN, GPIO_FUNC_SIO);
  gpio_set_dir(DBG_PIN, GPIO_OUT);
  pinState = 0;
  gpio_put(DBG_PIN, pinState);

  audioSampleRate = AUDIO_SAMPLE_RATE;
  // Set up ADC pin for audio in
  adc_init();
  adc_gpio_init(ADC_PIN);
  adc_select_input(0);
  //adc_set_clkdiv(999.0); // 48 MHz / (999+1) => 48 kHz
  adc_fifo_setup(true, true, 2, false, false);
  adcDataPtr = &adc_hw->fifo;

  uac2_out_handle = xTaskCreateStatic(
    uac2OutTask,
    "UAC2 Out",
    UAC2_OUT_STACK_SIZE,
    NULL,
    UAC2_OUT_TASK_PRIO,
    uac2_out_stack,
    &uac2_out_taskdef
  );

  vTaskCoreAffinitySet(uac2_out_handle, 1<<0);

  uac2_in_handle = xTaskCreateStatic(
    uac2InTask,
    "UAC2 In",
    UAC2_IN_STACK_SIZE,
    NULL,
    UAC2_IN_TASK_PRIO,
    uac2_in_stack,
    &uac2_in_taskdef
  );

  vTaskCoreAffinitySet(uac2_in_handle, 1<<0);

  spk_cmd_q = xQueueCreateStatic(
    CH_MSG_Q_LEN,
    CH_MSG_SIZE,
    spk_cmd_q_buf,
    &spk_cmd_q_def
  ); 

  mic_cmd_q = xQueueCreateStatic(
    CH_MSG_Q_LEN,
    CH_MSG_SIZE,
    mic_cmd_q_buf,
    &mic_cmd_q_def
  ); 

  spk_ch_handle = xTaskCreateStatic(
    chController,
    "Spk Ch",
    SPK_CH_STACK_SIZE,
    &spkChSettings,
    SPK_CH_TASK_PRIO,
    spk_ch_stack,
    &spk_ch_taskdef
  );

  mic_ch_handle = xTaskCreateStatic(
    chController,
    "Mic Ch",
    MIC_CH_STACK_SIZE,
    &micChSettings,
    MIC_CH_TASK_PRIO,
    mic_ch_stack,
    &mic_ch_taskdef
  );

}

bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
  pinState = !pinState;
  gpio_put(DBG_PIN, pinState);
  if(controlMsg(spk_cmd_q, CH_DATA_RECEIVED, n_bytes_received >> 1) != pdPASS) {
    spkCh.usbCtrlFails++;
  }
  return true;
}

bool tud_audio_rx_done_post_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
  return true;
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting) {
  tu_fifo_t* ff = tud_audio_get_ep_in_ff();
  if (micCh.state == AUDIO_IDLE) {
    tu_fifo_clear(ff);
  //   tud_audio_write(zeroBuf, micCh.ioChunk);
  //   tud_audio_write(zeroBuf, micCh.ioChunk);
  // } else {
  //   uint16_t count = tu_fifo_count(ff);
  //   usbDebugSet(4, count);
  }
  return true;
}

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting) {
  if(controlMsg(mic_cmd_q, CH_DATA_SENT, n_bytes_copied >> 1) != pdPASS) {
    micCh.usbCtrlFails++;
  }
  return true;
}

// Audio controls
// Current states
static int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];       // +1 for master channel 0
static int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];    // +1 for master channel 0

// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request)
{
  TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
  if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
  {
    if (request->bRequest == AUDIO_CS_REQ_CUR)
    {
      // sendMessageEventf("Clock get current freq %u", audioSampleRate);
      audio_control_cur_4_t curf = { tu_htole32(audioSampleRate) };
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
    }
    else if (request->bRequest == AUDIO_CS_REQ_RANGE)
    {
      audio_control_range_4_n_t(1) rangef =
      {
        .wNumSubRanges = tu_htole16(1),
        .subrange[0] = { tu_htole32(44100), tu_htole32(AUDIO_SAMPLE_RATE), tu_htole32(1)}
      };
      // sendMessageEventf("Clock get freq range (%d, %d, %d)", (int)rangef.subrange[0].bMin, (int)rangef.subrange[0].bMax, (int)rangef.subrange[0].bRes);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
    }
  }
  else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
           request->bRequest == AUDIO_CS_REQ_CUR)
  {
    audio_control_cur_1_t cur_valid = { .bCur = 1 };
    sendMessageEventf("Clock get is valid %u", cur_valid.bCur);
    return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
  }
  sendMessageEventf("Clock get request not supported, entity = %u, selector = %u, request = %u",
          request->bEntityID, request->bControlSelector, request->bRequest);
  return false;
}

// Helper for clock set requests
static bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
  TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
  if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
  {
    if (request->bRequest == AUDIO_CS_REQ_CUR)
    {
      TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));
      audioSampleRate = ((audio_control_cur_4_t const *)buf)->bCur;
      controlMsg(mic_cmd_q, CH_SET_RATE, audioSampleRate);
      controlMsg(spk_cmd_q, CH_SET_RATE, audioSampleRate);
      sendMessageEventf("Clock set current freq %lu", audioSampleRate);
      return true;
    }
  }
  sendMessageEventf("Clock set request not supported, entity = %u, selector = %u, request = %u",
          request->bEntityID, request->bControlSelector, request->bRequest);
  return false;
}

// Helper for feature unit get requests
static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request)
{
  TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);

  if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && request->bRequest == AUDIO_CS_REQ_CUR)
  {
    audio_control_cur_1_t mute1 = { .bCur = mute[request->bChannelNumber] };
    // sendMessageEventf("Get channel %u mute %d", request->bChannelNumber, mute1.bCur);
    return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &mute1, sizeof(mute1));
  }
  else if (UAC2_ENTITY_SPK_FEATURE_UNIT && request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
  {
    if (request->bRequest == AUDIO_CS_REQ_RANGE)
    {
      audio_control_range_2_n_t(1) range_vol = {
        .wNumSubRanges = tu_htole16(1),
        .subrange[0] = { .bMin = tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(256) }
      };
      // sendMessageEventf("Get channel %u volume range (%d, %d, %u) dB", request->bChannelNumber,
      //         range_vol.subrange[0].bMin, range_vol.subrange[0].bMax, range_vol.subrange[0].bRes);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
    }
    else if (request->bRequest == AUDIO_CS_REQ_CUR)
    {
      audio_control_cur_2_t cur_vol = { .bCur = tu_htole16(volume[request->bChannelNumber]) };
      // sendMessageEventf("Get channel %u volume %d dB", request->bChannelNumber, cur_vol.bCur);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
    }
  }
  sendMessageEventf("Feature unit get request not supported, entity = %u, selector = %u, request = %u",
          request->bEntityID, request->bControlSelector, request->bRequest);

  return false;
}

// Helper for feature unit set requests
static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
  TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
  TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);
  if (request->bControlSelector == AUDIO_FU_CTRL_MUTE)
  {
    TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));
    mute[request->bChannelNumber] = ((audio_control_cur_1_t *)buf)->bCur;
    sendMessageEventf("Set channel %d Mute: %d", request->bChannelNumber, mute[request->bChannelNumber]);
    return true;
  }
  else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
  {
    TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));
    volume[request->bChannelNumber] = ((audio_control_cur_2_t const *)buf)->bCur;
    sendMessageEventf("Set channel %d volume: %d dB", request->bChannelNumber, volume[request->bChannelNumber]);
    return true;
  }
  else
  {
    sendMessageEventf("Feature unit set request not supported, entity = %u, selector = %u, request = %u",
            request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
  }
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
  audio_control_request_t *request = (audio_control_request_t *)p_request;
  if (request->bEntityID == UAC2_ENTITY_CLOCK)
    return tud_audio_clock_get_request(rhport, request);
  if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
    return tud_audio_feature_unit_get_request(rhport, request);
  sendMessageEventf("Get request not handled, entity = %d, selector = %d, request = %d",
          request->bEntityID, request->bControlSelector, request->bRequest);
  return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
  audio_control_request_t const *request = (audio_control_request_t const *)p_request;
  if (request->bEntityID == UAC2_ENTITY_CLOCK)
    return tud_audio_clock_set_request(rhport, request, buf);
  if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
    return tud_audio_feature_unit_set_request(rhport, request, buf);
  sendMessageEventf("Set request not handled, entity = %d, selector = %d, request = %d",
          request->bEntityID, request->bControlSelector, request->bRequest);
  return false;
}
 
bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request) {
  uint8_t const itf = tu_u16_low(p_request->wIndex);
  uint8_t const alt = tu_u16_low(p_request->wValue);
  if (itf == ITF_NUM_AUDIO_STREAMING_SPK) {
    for (uint32_t i = 2; i<DEBUG_INFO_COUNT; i++) {
      debugInfo[i]=0;
    }
    debugInfo[0]++;
    debugInfo[3] = to_us_since_boot(get_absolute_time());
    return true;
  } else if (itf == ITF_NUM_AUDIO_STREAMING_MIC) {
    debugInfo[1]++;
    return true;
  }
  return false;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request) {
  uint8_t const itf = tu_u16_low(p_request->wIndex);
  if (itf == ITF_NUM_AUDIO_STREAMING_MIC) {
    debugInfo[2]++;
    return true;
  }
  return true;
}


