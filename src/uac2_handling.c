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

static StackType_t  mic_ch_stack[MIC_CH_STACK_SIZE];
static StaticTask_t mic_ch_taskdef;
static TaskHandle_t mic_ch_handle;

#define DBG_PIN 5
#define PWM_PIN 2
#define ADC_PIN 26

static uint8_t pinState = 0;

// These are computed at UAC task start and then remain constant
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

// The values in these structs will be updated only by the
// channels dedicated controller task and all individual values are atomic
typedef struct {
    audio_state_t state;
    uint32_t sendClkDiv;
    uint32_t recClkDiv;
} usb_channel_t;

static usb_channel_t micCh;

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

static StaticQueue_t input_cmd_q_def;
static uint8_t       input_cmd_q_buf[CH_MSG_Q_LEN*CH_MSG_SIZE];
static QueueHandle_t input_cmd_q;

inline static void controlMsg(QueueHandle_t queue, ch_cmd_t cmd, uint64_t value) {
  ch_cmd_msg_t msg = {
    .cmd = cmd,
    .count = value,
    .time = to_us_since_boot(get_absolute_time())
  };
  xQueueSendToBack(queue, &msg, 0);
}

static volatile uint32_t uac2OutActive;
static volatile uint32_t uac2ResetCount;

static volatile uint32_t frameCountCB;
static volatile uint32_t longFramesCB;
static volatile uint32_t shortFramesCB;
static volatile uint32_t byteCountCB;
static volatile uint64_t firstFrameTimeCB;
static volatile uint64_t lastFrameTimeCB;
static volatile uint64_t totalTimeCB;

static volatile audio_state_t audioState;
static volatile uint32_t frameCount;
static volatile uint32_t byteCount;
static volatile uint32_t zeroCount;
static volatile uint32_t nonZeroCount;


static volatile int64_t finalByteRate;

//static volatile uint32_t rateUpdateCount;
static volatile uint32_t timeOutCount;
//static volatile uint32_t bufferWrapCount;

//static volatile uint64_t runningDmaTime;

static volatile uint32_t dmaCount;
//static volatile uint64_t savedFrameTime;
//static volatile int64_t targetTimeSkew = 8500;
//static volatile int64_t timeSkew;

static volatile uint8_t dmaRunning;
//static volatile uint8_t skewUpdated;

static volatile uint32_t pwmDiv;
static volatile uint32_t pwmDivMax;
static volatile uint32_t pwmDivMin;

#define PWM_DIV_INIT ((10 << 16) + (0x2c3u << 4))
#define PWM_DIV_SPAN (2<<12)
// static volatile uint8_t pwmDivInt;
// static volatile uint8_t pwmDivFrac;

// Max number of missing USB frames before dropping connection
#define MAX_DROPPED_FRAMES 2500                    // as count
#define MAX_DROPOUT_TIME (MAX_DROPPED_FRAMES*1000) // as time in us

// General buffer parameters for sound
#define SAMPLES_PER_FRAME 48
#define BYTES_PER_SAMPLE 2

// Buffer for outgoing sound data
#define FRAME_BUFFER_COUNT 16
#define BYTES_PER_FRAME (SAMPLES_PER_FRAME * BYTES_PER_SAMPLE)
#define BUFFER_SIZE_SAMPLES (FRAME_BUFFER_COUNT * SAMPLES_PER_FRAME)
#define BUFFER_SIZE (BUFFER_SIZE_SAMPLES * BYTES_PER_SAMPLE)

typedef union
{
  uint8_t bytes[BUFFER_SIZE];
  int16_t samples[BUFFER_SIZE_SAMPLES];
} out_buffer_t;

static out_buffer_t outBuffer;

// Buffer for incoming sound data
#define ADC_CHUNK SAMPLES_PER_FRAME
#define ADC_BUFFER_COUNT 16
#define ADC_BUFFER_SIZE_SAMPLES (ADC_BUFFER_COUNT * ADC_CHUNK)

static int16_t dummyBuf[ADC_BUFFER_SIZE_SAMPLES];
static int16_t zeroBuf[SAMPLES_PER_FRAME];

// Updated on every 16th usb frame interval
static volatile int32_t usb16FrameTime;
static volatile int32_t pwm16FrameTime;
static volatile int64_t usbTotalTime; // TODO protect by semaphore
static volatile int64_t pwmTotalTime;

static volatile int32_t pwmLagTime;
static volatile int32_t pwmAccLagTime;
static volatile int32_t pwmMinLagTime;
static volatile int32_t pwmMaxLagTime;

static volatile int64_t startPwmTime;

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

  printf("Audio state:          %15s %15s\n", stateString(audioState), stateString(micCh.state));
  printf("Audio active, resets: %15lu %15lu\n", uac2OutActive, uac2ResetCount);
  
  printf("Frames CB, task, dma: %15lu %15lu %15lu\n",  frameCountCB, frameCount, dmaCount);
  printf("Frames zero, data:    %15lu %15lu\n",  zeroCount, nonZeroCount);
  printf("Bytes CB, task:       %15lu %15lu\n",  byteCountCB, byteCount);

  printf("Time first, last, tot:%15llu %15llu %15llu\n", firstFrameTimeCB, lastFrameTimeCB, totalTimeCB);
  printf("Time usb, pwm, lag:   %15llu %15llu %15li\n", usbTotalTime, pwmTotalTime, pwmLagTime);
  
  // printf("Lag current, target:  %15lli %15lli\n", timeSkew, targetTimeSkew);
  printf("Lag curr, acc:        %15li %15li\n", pwmLagTime, pwmAccLagTime);
  printf("Lag min, max:         %15li %15li\n", pwmMinLagTime, pwmMaxLagTime);

  // printf("PWM div int, frac:    %15hhu %15hhu\n", pwmDivInt, pwmDivFrac);
  printf("PWM div int, frac:    %15lu %15lu\n", pwmDiv>>16, (pwmDiv)&0xffff);
  printf("16 frames usb, pwm:   %15li %15li\n", usb16FrameTime, pwm16FrameTime);

  //printf("DMA rate updates:     %15lu\n",  rateUpdateCount);  
  printf("Final byte rate:      %15lli\n", finalByteRate);  

  printf("Frame per long, short:%15lu %15lu\n",  longFramesCB, shortFramesCB);
  printf("Time out count:       %15lu\n",  timeOutCount);
  //printf("Buffer wrap count:    %15lu\n",  bufferWrapCount);

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
  static uint32_t rdIndex;
  static uint64_t savePwmTime;
  uint64_t runningPwmTime = to_us_since_boot(get_absolute_time());

  if (!dmaRunning) {
    dmaRunning = 1;
    dmaCount = 1;
    rdIndex = SAMPLES_PER_FRAME;    // First frame already transferred
    startPwmTime = runningPwmTime;
    savePwmTime = runningPwmTime;
    pwm16FrameTime = 16000;
    pwmTotalTime = 0;
    pwmLagTime = 0;
  } else {
    dmaCount++;
    if ((dmaCount & 0x000fu)==7) {
      pwm16FrameTime = runningPwmTime - savePwmTime;
      savePwmTime = runningPwmTime;
      pwmTotalTime = runningPwmTime - startPwmTime;
      //pwmLagTime = pwmTotalTime - usbTotalTime;
    }
    rdIndex += SAMPLES_PER_FRAME;
    if (rdIndex >= BUFFER_SIZE_SAMPLES) {
      rdIndex = 0;
    }
  }
  
  if (frameCount > dmaCount) {
    dma_channel_set_read_addr(pwmDmaCh, &outBuffer.samples[rdIndex], true);
  } else {
    dmaRunning = 0;
  }
  dma_hw->ints1 = 1u << pwmDmaCh;
}

static void uac2OutTask(void *pvParameters) {
  uint32_t      newCount;
  uint8_t       *bytePtr;
  uint8_t       *maxBytePtr = outBuffer.bytes + BUFFER_SIZE;
  int16_t       *samplePtr;
  int32_t       adjust;
  TickType_t    timeOut = portMAX_DELAY;
  int64_t       lastTaskTime;
  int64_t       startUsbTime;
  //uint32_t      filledFrames;
  while(true)
  {
    newCount = ulTaskNotifyTake(pdTRUE, timeOut);
    uint64_t now = to_us_since_boot(get_absolute_time());

    uint32_t xx = tud_audio_available();
    if (xx<96) {
      debugInfo[10]++;
    } else if (xx>96) {
      debugInfo[11]++;
    }

    // Fake possible missing USB frames
    while (dmaRunning && now > (lastTaskTime + 1500)) {
      lastTaskTime += 1000;
      //filledFrames++;
      timeOutCount++;
      frameCount++;
      bytePtr += BYTES_PER_FRAME;
      if (bytePtr>=maxBytePtr) {
        bytePtr -= BUFFER_SIZE;
      }
      for (uint32_t i=0; i<SAMPLES_PER_FRAME; i++) {
        *(samplePtr++) = 0;
        if (samplePtr>=(int16_t*)maxBytePtr) {
          samplePtr = outBuffer.samples;
        }
      }
      byteCount += BYTES_PER_FRAME;
      if ((frameCount & 0x000fu)==0) {
        usbTotalTime += usb16FrameTime;
        lastTaskTime += (usb16FrameTime - 16000);
      }
    }

    if (newCount>0) {
      lastTaskTime = now;
      //filledFrames = 0;
      switch (audioState) {
      case AUDIO_IDLE:
        bytePtr = outBuffer.bytes;
        samplePtr = outBuffer.samples;
        frameCount = 1;
        dmaCount = 0;
        byteCount = 0;
        zeroCount = 0;
        nonZeroCount = 0;
        finalByteRate = 0;
        timeOutCount = 0;
        pwmMinLagTime = INT32_MAX;
        pwmMaxLagTime = INT32_MIN;
        pwmAccLagTime = 0;
        pwmDiv = PWM_DIV_INIT;
        adjust = 0;
        *pwmDivPtr = pwmDiv >> 12;
        timeOut = pdMS_TO_TICKS(1);
        audioState = AUDIO_SYNC;
        startUsbTime = now;
        break;
      case AUDIO_SYNC:
        if ((frameCount & 0xf)==7) {
          audioState = AUDIO_RUN;
          startPwmTime = now;
          dma_channel_set_read_addr(pwmDmaCh, outBuffer.samples, true);
        }
        frameCount++;
      case AUDIO_RUN:
        frameCount++;
        break;
      default:
        panic("Bad audio state");
        break;
      }

      int16_t nonZero = 0;
      uint32_t chunkSize = 1;
      uint32_t needBytes = 96;
      while (needBytes) {
        uint32_t maxChunk = MIN(needBytes, maxBytePtr - bytePtr);
        chunkSize = tud_audio_read(bytePtr, maxChunk);
        samplePtr = (int16_t*)bytePtr;
        bytePtr += chunkSize;
        needBytes -= chunkSize;

        while (samplePtr<(int16_t*)bytePtr) {
          int16_t sample = *samplePtr;
          nonZero = nonZero | sample;
          sample = sample>>8;           // -128 -- +127 in low byte, sign in high byte
          sample = (sample+128) & 0xff; //    0 --  255 in low byte, zero in high byte
          (*samplePtr++) = sample;
        }

        byteCount += chunkSize;
        if(bytePtr==maxBytePtr) {
          bytePtr = outBuffer.bytes;
        } else if (bytePtr>maxBytePtr) {
          panic("Bad out buffer state");
        }
      }

      if (nonZero) {
        nonZeroCount++;
      } else {
        zeroCount++;
      }

      if ((frameCount & 0x000fu)==0) {
        usbTotalTime = now - startPwmTime;
      }

    } else {
      if (now > lastFrameTimeCB + MAX_DROPOUT_TIME) {
        if (audioState!=AUDIO_IDLE) {
          finalByteRate = 1000000*((uint64_t)byteCount) / (totalTimeCB+1000);
          //uac2Active = 0;
          uac2ResetCount++;
          timeOut = portMAX_DELAY;
          audioState = AUDIO_IDLE;
          pinState = 0;
          gpio_put(DBG_PIN, pinState);
        }
      }
    }

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

    if ((frameCount & 0x000fu) < ((pwmDiv >> 8) & 0x000fu)) {
      *pwmDivPtr = (pwmDiv >> 12)+1;
    } else {
      *pwmDivPtr = pwmDiv >> 12;
    }
  }
}

static void initMicCh() {
}

static void openMicCh() {
  for (uint32_t i=0; i<ADC_BUFFER_SIZE_SAMPLES; i++) {
    dummyBuf[i] = 2048;
  }
  dma_channel_set_write_addr(adcDmaCh, dummyBuf, true);
  adc_run(true);
}

static void closeMicCh() {
  dma_channel_abort(adcDmaCh);
  adc_run(false);
}

static void setMicChRate(float sampleRate) {
  float clockDiv = (48000000.0 / (float)sampleRate) - 1.0;
  adc_set_clkdiv(clockDiv); // Example: 48 MHz / (999+1) => 48 kHz
}

typedef struct {
  usb_channel_t *ch;
  QueueHandle_t *queue;
  bool toUsb;
  void (*init)(void);
  void (*setRate)(float);
  void (*open)(void);
  void (*close)(void);
} usb_channel_settings_t;

static usb_channel_settings_t micChSettings = {
  .ch = &micCh,
  .queue = &input_cmd_q,
  .toUsb = true,
  .init = initMicCh,
  .setRate = setMicChRate,
  .open = openMicCh,
  .close = closeMicCh
};

static void adcDmaHandler() {
  static uint32_t bufferNr = 0;
  BaseType_t taskWoken = pdFALSE;
  ch_cmd_msg_t msg = {
    .cmd = CH_DATA_RECEIVED,
    .count = ADC_CHUNK * BYTES_PER_SAMPLE,
    .time = to_us_since_boot(get_absolute_time())
  };
  uint32_t filledBufferNr = bufferNr;
  bufferNr++;
  if (bufferNr>=ADC_BUFFER_COUNT) {
    bufferNr = 0;
  }
  dma_channel_set_write_addr(adcDmaCh, &dummyBuf[bufferNr * ADC_CHUNK], true);
  xQueueSendToBackFromISR(input_cmd_q, &msg, NULL);
  xTaskNotifyFromISR(uac2_in_handle, filledBufferNr, eSetValueWithOverwrite, &taskWoken);
  portYIELD_FROM_ISR(taskWoken);
  dma_hw->ints0 = 1u << adcDmaCh;
}

static void uac2InTask(void *pvParameters) {
  int32_t high = INT16_MIN;
  int32_t low  = INT16_MAX;
  while(true) {
    uint32_t bufferNr = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    bufferNr += ADC_BUFFER_COUNT >> 1;
    if (bufferNr>=ADC_BUFFER_COUNT) {
      bufferNr -= ADC_BUFFER_COUNT;
    }
    uint32_t base = bufferNr * ADC_CHUNK;
    usbDebugInc(19);
    int32_t sum  = 0;
    for (uint32_t i=0; i<ADC_CHUNK; i++) {
      int32_t sample = ((dummyBuf[base + i] & 0x0fff) - 2048) << 4;
      sum += sample;
      if (sample>high) {
        high = sample;
      }
      if (sample<low) {
        low = sample;
      }
      dummyBuf[base + i] = sample;
    }
    tud_audio_write(&dummyBuf[base], ADC_CHUNK * BYTES_PER_SAMPLE);
    usbDebugSet(16, sum/ADC_CHUNK);
    usbDebugSet(17, low);
    usbDebugSet(18, high);
  }
}

static void chController(void *pvParameters) {
  usb_channel_settings_t *p = (usb_channel_settings_t *) pvParameters;
  QueueHandle_t q = *(p->queue);
  usb_channel_t *ch = p->ch;
  ch_cmd_msg_t cmd;
  int64_t resetTime = to_us_since_boot(get_absolute_time());
  int64_t recTime = 0;
  int64_t sendTime = 0;
  int32_t byteRate = AUDIO_SAMPLE_RATE * BYTES_PER_SAMPLE;
  uint64_t recBytes = 0;
  uint64_t sendBytes = 0;
  
  (p->init)();
  (p->setRate)((float)AUDIO_SAMPLE_RATE);
  while (true) {
    if (xQueueReceive(q, &cmd, portMAX_DELAY) == pdTRUE) {
      switch (cmd.cmd) {
        case CH_SET_RATE:
          (p->setRate)((float)cmd.count);
          byteRate = cmd.count * BYTES_PER_SAMPLE;
          ch->state = AUDIO_IDLE;
          break;
        case CH_DATA_RECEIVED:
          if (!p->toUsb && ch->state == AUDIO_IDLE) {
            // USB sends data, start channel
            resetTime = cmd.time - CH_USB_FRAME_TIME;
            sendTime = 0;
            recBytes = 0;
            sendBytes = 0;
            (p->open)();
            ch->state = AUDIO_SYNC;
          }
          recTime = cmd.time - resetTime;
          recBytes += cmd.count;
          break;
        case CH_DATA_SENT:
          if (p->toUsb && ch->state == AUDIO_IDLE) {
            // USB wants data, start channel
            resetTime = cmd.time - CH_USB_FRAME_TIME;
            recTime = 0;
            recBytes = 0;
            sendBytes = 0;
            (p->open)();
            ch->state = AUDIO_SYNC;
          }
          sendTime = cmd.time - resetTime;
          sendBytes += cmd.count;
          break;
      }
      if (recTime > CH_SYNC_TIME) {
        usbDebugSet(14, recTime);
        usbDebugSet(15, recBytes);
        usbDebugSet(13, (1000000 * recBytes) / recTime);
      } else {
        usbDebugInc(12);
      }
      if (sendTime > CH_SYNC_TIME) {
        usbDebugSet(10, sendTime);
        usbDebugSet(11, sendBytes);
        usbDebugSet(9, (1000000 * sendBytes) / sendTime);
      } else {
        usbDebugInc(8);
      }
      if (ch->state == AUDIO_SYNC && recTime > CH_SYNC_TIME && sendTime > CH_SYNC_TIME) {
        ch->state = AUDIO_RUN;
      }
      if (ch->state != AUDIO_IDLE) {

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

  // pwmDivInt = 10;
  // pwmDivFrac = 3;

  pwmDiv = PWM_DIV_INIT;
  pwmDivMax = PWM_DIV_INIT + PWM_DIV_SPAN;
  pwmDivMin = PWM_DIV_INIT - PWM_DIV_SPAN;

  audioSampleRate = AUDIO_SAMPLE_RATE;

  // Set up PWM pin for audio out
  gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
  pwmSlice = pwm_gpio_to_slice_num(PWM_PIN);
  pwm_set_wrap(pwmSlice, 255);
  //pwm_set_clkdiv_int_frac(pwmSlice, pwmDivInt, pwmDivFrac);
  //pwm_set_chan_level(pwmSlice, PWM_CHAN_A, 127);
  pwmDataPtr = &pwm_hw->slice[pwmSlice].cc;
  *pwmDataPtr = 16;
  pwmDivPtr = &pwm_hw->slice[pwmSlice].div;
  *pwmDivPtr = pwmDiv >> 12;
  pwm_set_enabled(pwmSlice, true);

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
    outBuffer.samples,// Read address
    SAMPLES_PER_FRAME,
    false             // Start
  );

  dma_channel_set_irq0_enabled(pwmDmaCh, false);
  dma_channel_set_irq1_enabled(pwmDmaCh, true);
  irq_set_exclusive_handler(DMA_IRQ_1, pwmDmaHandler);
  irq_set_enabled(DMA_IRQ_1, true);

  // Set up ADC pin for audio in
  adc_init();
  adc_gpio_init(ADC_PIN);
  adc_select_input(0);
  //adc_set_clkdiv(999.0); // 48 MHz / (999+1) => 48 kHz
  adc_fifo_setup(true, true, 2, false, false);
  adcDataPtr = &adc_hw->fifo;

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
    dummyBuf,         // Write address
    adcDataPtr,       // Read address
    ADC_CHUNK,        // Words to transfer
    false             // Start
  );

  dma_channel_set_irq0_enabled(adcDmaCh, true);
  dma_channel_set_irq1_enabled(adcDmaCh, false);
  irq_set_exclusive_handler(DMA_IRQ_0, adcDmaHandler);
  irq_set_enabled(DMA_IRQ_0, true);

  uac2_out_handle = xTaskCreateStatic(
    uac2OutTask,
    "UAC2 Out",
    UAC2_OUT_STACK_SIZE,
    NULL,
    UAC2_OUT_TASK_PRIO,
    uac2_out_stack,
    &uac2_out_taskdef
  );

  uac2_in_handle = xTaskCreateStatic(
    uac2InTask,
    "UAC2 In",
    UAC2_IN_STACK_SIZE,
    NULL,
    UAC2_IN_TASK_PRIO,
    uac2_in_stack,
    &uac2_in_taskdef
  );

  input_cmd_q = xQueueCreateStatic(
    CH_MSG_Q_LEN,
    CH_MSG_SIZE,
    input_cmd_q_buf,
    &input_cmd_q_def
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

  static uint64_t saveUsbTime;
  uint64_t currentFrameTime = to_us_since_boot(get_absolute_time());
  pinState = !pinState;
  gpio_put(DBG_PIN, pinState);

  int32_t frameNr = usb_hw->sof_rd;
  static int32_t lastFrameNr;
  static int32_t maxFrameSkip;

  if (!uac2OutActive) {
    uac2OutActive = 1;
    maxFrameSkip = 0;
    frameCountCB = 1;
    byteCountCB = n_bytes_received;
    firstFrameTimeCB = currentFrameTime;
    longFramesCB = 0;
    shortFramesCB = 0;
    usb16FrameTime = 16000;
    saveUsbTime = currentFrameTime;
  } else {
    // TODO - block this if we recently had a dropout
    if ((frameCountCB & 0x000fu)==0) {
      usb16FrameTime = currentFrameTime - saveUsbTime;
      saveUsbTime = currentFrameTime;
    }

    frameCountCB++;
    byteCountCB += n_bytes_received;
    int64_t timeDiff = currentFrameTime - lastFrameTimeCB;
    if (timeDiff > 1100) {
      longFramesCB++;
    } else if (timeDiff < 900) {
      shortFramesCB++;
    }
    int32_t frameSkip = (frameNr - lastFrameNr) & 0x7ff; // USB frame numbers are 12 bits
    if (frameSkip>1) {
      debugInfo[1]++;
      debugInfo[2] += (frameSkip - 1);
    }
  }
  lastFrameTimeCB = currentFrameTime;
  totalTimeCB = currentFrameTime - firstFrameTimeCB;
  lastFrameNr = frameNr;
  debugInfo[7] = frameNr;
  return true;
}

bool tud_audio_rx_done_post_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
  // xTaskNotifyGive(uac2_out_handle);
  return true;
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting) {
  // tud_audio_write(zeroBuf, BYTES_PER_FRAME);
  tu_fifo_t* ff = tud_audio_get_ep_in_ff();
  if (micCh.state == AUDIO_IDLE) {
    tu_fifo_clear(ff);
    tud_audio_write(zeroBuf, BYTES_PER_FRAME);
    tud_audio_write(zeroBuf, BYTES_PER_FRAME);
  } else {
    uint16_t count = tu_fifo_count(ff);
    usbDebugSet(4, count);
  }
  return true;
}

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting) {
  controlMsg(input_cmd_q, CH_DATA_SENT, n_bytes_copied);
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
      sendMessageEventf("Clock get current freq %u", audioSampleRate);
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
      sendMessageEventf("Clock get freq range (%d, %d, %d)", (int)rangef.subrange[0].bMin, (int)rangef.subrange[0].bMax, (int)rangef.subrange[0].bRes);
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
      controlMsg(input_cmd_q, CH_SET_RATE, audioSampleRate);
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
    sendMessageEventf("Get channel %u mute %d", request->bChannelNumber, mute1.bCur);
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
      sendMessageEventf("Get channel %u volume range (%d, %d, %u) dB", request->bChannelNumber,
              range_vol.subrange[0].bMin, range_vol.subrange[0].bMax, range_vol.subrange[0].bRes);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
    }
    else if (request->bRequest == AUDIO_CS_REQ_CUR)
    {
      audio_control_cur_2_t cur_vol = { .bCur = tu_htole16(volume[request->bChannelNumber]) };
      sendMessageEventf("Get channel %u volume %d dB", request->bChannelNumber, cur_vol.bCur);
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
    uac2OutActive = 0;
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


