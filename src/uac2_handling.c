#include "FreeRTOS.h"
#include "task.h"

#include "bsp/board.h"
#include "tusb.h"

#include "program_config.h"
#include "uac2_handling.h"
#include "usb_descriptors.h"
#include "program_state.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/structs/usb.h"
#include "pico/stdlib.h"

static StackType_t  uac2_stack[UAC2_STACK_SIZE];
static StaticTask_t uac2_taskdef;
static TaskHandle_t uac2_handle;

#define DbgPin 5
#define PwmPin 2

static uint8_t pinState = 0;

static uint   pwmSlice;
static uint   pwmDmaCh;
static volatile uint32_t * pwmDataPtr;
static volatile uint32_t * pwmDivPtr;

typedef enum {
  AUDIO_IDLE,
  AUDIO_STARTING,
  AUDIO_RUNNING,
  AUDIO_ERROR
} audio_state_t;

static volatile uint8_t uac2Active;
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

static volatile uint64_t debugInfo[DEBUG_INFO_COUNT];

#define PWM_DIV_INIT ((10 << 16) + (0x2c3u << 4))
#define PWM_DIV_SPAN (2<<12)
// static volatile uint8_t pwmDivInt;
// static volatile uint8_t pwmDivFrac;

// Max number of missing USB frames before dropping connection
#define MAX_DROPPED_FRAMES 2500                    // as count
#define MAX_DROPOUT_TIME (MAX_DROPPED_FRAMES*1000) // as time in us

// Buffer for outgoing sound data
#define FRAME_BUFFER_COUNT 16
#define SAMPLES_PER_FRAME 48
#define BYTES_PER_SAMPLE 2
#define BYTES_PER_FRAME (SAMPLES_PER_FRAME*BYTES_PER_SAMPLE)
#define BUFFER_SIZE_SAMPLES (FRAME_BUFFER_COUNT*SAMPLES_PER_FRAME)
#define BUFFER_SIZE (FRAME_BUFFER_COUNT*SAMPLES_PER_FRAME*BYTES_PER_SAMPLE)

typedef union
{
  uint8_t bytes[BUFFER_SIZE];
  int16_t samples[BUFFER_SIZE_SAMPLES];
} out_buffer_t;

static out_buffer_t outBuffer;

static int16_t dummyBuf[1000];

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


void usbDebugInc(uint32_t index) {
  debugInfo[index]++;
}

void usbDebugSet(uint32_t index, uint64_t value) {
  debugInfo[index] = value;
}

void usbDebugMax(uint32_t index, uint64_t value) {
  if (value > debugInfo[index]) {
    debugInfo[index] = value;
  }
}

void reportUSB_UAC2() {
  char * state = "...";
  switch (audioState) {
  case AUDIO_IDLE:
    state = "Idle";
    break;
  case AUDIO_STARTING:
    state = "Starting";
    break;
  case AUDIO_RUNNING:
    state = "Running";
    break;
  default:
    break;
  }

  printf("Audio state:          %15s\n",   state);
  printf("Audio active, resets: %15hhu %15lu\n", uac2Active, uac2ResetCount);
  
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
    printf("%15llu ", debugInfo[dbgIdx++]);
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
  dma_hw->ints0 = 1u << pwmDmaCh;
}

static void uac2Task(void *pvParameters) {
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
        audioState = AUDIO_STARTING;
        startUsbTime = now;
        break;
      case AUDIO_STARTING:
        if ((frameCount & 0xf)==7) {
          audioState = AUDIO_RUNNING;
          startPwmTime = now;
          dma_channel_set_read_addr(pwmDmaCh, outBuffer.samples, true);
        }
        frameCount++;
      case AUDIO_RUNNING:
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
          gpio_put(DbgPin, pinState);
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

void createUAC2Handler() {
  gpio_set_function(DbgPin, GPIO_FUNC_SIO);
  gpio_set_dir(DbgPin, GPIO_OUT);
  pinState = 0;
  gpio_put(DbgPin, pinState);

  // pwmDivInt = 10;
  // pwmDivFrac = 3;

  pwmDiv = PWM_DIV_INIT;
  pwmDivMax = PWM_DIV_INIT + PWM_DIV_SPAN;
  pwmDivMin = PWM_DIV_INIT - PWM_DIV_SPAN;


  for (int32_t i=0; i<16; i++) dummyBuf[i]=256;
  //dummyBuf[47] = 127;


  
  gpio_set_function(PwmPin, GPIO_FUNC_PWM);
  pwmSlice = pwm_gpio_to_slice_num(PwmPin);
  pwm_set_wrap(pwmSlice, 255);
  //pwm_set_clkdiv_int_frac(pwmSlice, pwmDivInt, pwmDivFrac);
  //pwm_set_chan_level(pwmSlice, PWM_CHAN_A, 127);
  pwmDataPtr = &pwm_hw->slice[pwmSlice].cc;
  *pwmDataPtr = 16;
  pwmDivPtr = &pwm_hw->slice[pwmSlice].div;
  *pwmDivPtr = pwmDiv >> 12;
  pwm_set_enabled(pwmSlice, true);

  pwmDmaCh = dma_claim_unused_channel(true);
  dma_channel_config c = dma_channel_get_default_config(pwmDmaCh);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
  channel_config_set_write_increment(&c, false);
  channel_config_set_read_increment(&c, true);
  channel_config_set_dreq(&c, DREQ_PWM_WRAP1);

  dma_channel_configure(
    pwmDmaCh,
    &c,
    pwmDataPtr,       // Write address
    dummyBuf,         // Read address
    SAMPLES_PER_FRAME,
    false             // Start
  );

  dma_channel_set_irq0_enabled(pwmDmaCh, false);
  dma_channel_set_irq1_enabled(pwmDmaCh, true);
  irq_set_exclusive_handler(DMA_IRQ_1, pwmDmaHandler);
  irq_set_enabled(DMA_IRQ_1, true);

  uac2_handle = xTaskCreateStatic(uac2Task,
    "USB UAC2",
    UAC2_STACK_SIZE,
    NULL,
    UAC2_TASK_PRIO,
    uac2_stack,
    &uac2_taskdef);
}

bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {

  static uint64_t saveUsbTime;
  uint64_t currentFrameTime = to_us_since_boot(get_absolute_time());
  pinState = !pinState;
  gpio_put(DbgPin, pinState);

  int32_t frameNr = usb_hw->sof_rd;
  static int32_t lastFrameNr;
  static int32_t maxFrameSkip;

  if (!uac2Active) {
    uac2Active = 1;
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
      debugInfo[4] += timeDiff;
      //debugInfo[6] = frameCountCB;
    } else if (timeDiff < 900) {
      shortFramesCB++;
    }
    int32_t frameSkip = (frameNr - lastFrameNr) & 0x7ff; // USB frame numbers are 12 bits
    if (frameSkip>1) {
      debugInfo[1]++;
      debugInfo[2] += (frameSkip - 1);
    }
    if (frameSkip>maxFrameSkip) {
      maxFrameSkip = frameSkip;
      debugInfo[5] = maxFrameSkip;
    }
    if (n_bytes_received!=96) {
      //debugInfo[5] = n_bytes_received;
      //debugInfo[2]++;
    }
  }
  lastFrameTimeCB = currentFrameTime;
  totalTimeCB = currentFrameTime - firstFrameTimeCB;
  lastFrameNr = frameNr;
  debugInfo[7] = frameNr;
  return true;
}

bool tud_audio_rx_done_post_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
  xTaskNotifyGive(uac2_handle);
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

  // Example supports only single frequency, same value will be used for current value and range
  if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
  {
    if (request->bRequest == AUDIO_CS_REQ_CUR)
    {
      TU_LOG2("Clock get current freq %u\r\n", AUDIO_SAMPLE_RATE);

      audio_control_cur_4_t curf = { tu_htole32(AUDIO_SAMPLE_RATE) };
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
    }
    else if (request->bRequest == AUDIO_CS_REQ_RANGE)
    {
      audio_control_range_4_n_t(1) rangef =
      {
        .wNumSubRanges = tu_htole16(1),
        .subrange[0] = { tu_htole32(AUDIO_SAMPLE_RATE), tu_htole32(AUDIO_SAMPLE_RATE), 0}
      };
      TU_LOG2("Clock get freq range (%d, %d, %d)\r\n", (int)rangef.subrange[0].bMin, (int)rangef.subrange[0].bMax, (int)rangef.subrange[0].bRes);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
    }
  }
  else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
           request->bRequest == AUDIO_CS_REQ_CUR)
  {
    audio_control_cur_1_t cur_valid = { .bCur = 1 };
    TU_LOG2("Clock get is valid %u\r\n", cur_valid.bCur);
    return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
  }
  TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n",
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
    TU_LOG2("Get channel %u mute %d\r\n", request->bChannelNumber, mute1.bCur);
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
      TU_LOG2("Get channel %u volume range (%d, %d, %u) dB\r\n", request->bChannelNumber,
              range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
    }
    else if (request->bRequest == AUDIO_CS_REQ_CUR)
    {
      audio_control_cur_2_t cur_vol = { .bCur = tu_htole16(volume[request->bChannelNumber]) };
      TU_LOG2("Get channel %u volume %u dB\r\n", request->bChannelNumber, cur_vol.bCur);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
    }
  }
  TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n",
          request->bEntityID, request->bControlSelector, request->bRequest);

  return false;
}

// Helper for feature unit set requests
static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
  (void)rhport;

  TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
  TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

  if (request->bControlSelector == AUDIO_FU_CTRL_MUTE)
  {
    TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));

    mute[request->bChannelNumber] = ((audio_control_cur_1_t *)buf)->bCur;

    TU_LOG2("Set channel %d Mute: %d\r\n", request->bChannelNumber, mute[request->bChannelNumber]);

    return true;
  }
  else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
  {
    TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));

    volume[request->bChannelNumber] = ((audio_control_cur_2_t const *)buf)->bCur;

    TU_LOG2("Set channel %d volume: %d dB\r\n", request->bChannelNumber, volume[request->bChannelNumber] / 256);

    return true;
  }
  else
  {
    TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n",
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
  else
  {
    TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);
  }
  return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
  audio_control_request_t const *request = (audio_control_request_t const *)p_request;

  if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
    return tud_audio_feature_unit_set_request(rhport, request, buf);

  TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n",
          request->bEntityID, request->bControlSelector, request->bRequest);

  return false;
}

 
bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request) {
  uint8_t const itf = tu_u16_low(p_request->wIndex);
  uint8_t const alt = tu_u16_low(p_request->wValue);
  uac2Active = 0;
  for (uint32_t i = 1; i<DEBUG_INFO_COUNT; i++) {
    debugInfo[i]=0;
  }
  debugInfo[0]++;
  debugInfo[3] = to_us_since_boot(get_absolute_time());
  return true;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request) {
  return true;
}


