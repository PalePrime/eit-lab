#include "FreeRTOS.h"
#include "task.h"

#include "bsp/board.h"
#include "tusb.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/structs/usb.h"
#include "pico/stdlib.h"

#include "program_config.h"
#include "program_state.h"
#include "uac2_handling.h"
#include "mic_channel.h"
#include "filters.h"

static StackType_t  mic_codec_stack[MIC_CODEC_STACK_SIZE];
static StaticTask_t mic_codec_def;
static TaskHandle_t mic_codec_handle;

usb_channel_t micChannel;


// These are computed at program start and then remain constant
static uint   adcDmaCh;
static uint   adcDmaBufCh;
static volatile const uint32_t * adcDataPtr;
static volatile uint32_t * adcDivPtr;

// DMA buffers for incoming sound data
static int16_t micBuf[2][MAX_OVER_SAMPLE * MAX_IO_CHUNK]; 

// Fifo for unprocessed, incoming sound data
// Use the fifo implementation from the TinyUSB package
TU_FIFO_DEF(dataIn, 512, int16_t, false);

// Fifo for processed, sound data waiting to be sent over USB
TU_FIFO_DEF(dataToUSB, 512, int16_t, false);

// Buffer location of next buffer to use
static int16_t* nextBuffer = micBuf[1];

// Interrupt Service Routine, get sound data from buffer
// filled by DMA from the pwm pin
static void adcDmaHandler() {
  BaseType_t taskWoken = pdFALSE;
  // Get data from buffer just filled and swap pointer to next
  //int16_t *buffer = nextBuffer;
  if (nextBuffer != micBuf[0]) {
    nextBuffer = micBuf[0];
  } else {
    nextBuffer = micBuf[1];
  }
  // Unless channel has been marked as idle by our controller
  if (micChannel.state.state != AUDIO_IDLE) {
    // 1. Kick off next transfer asap,
    ///dma_channel_set_write_addr(adcDmaCh, nextBuffer, true);
    // 2. inform controller that a chunk of data was sent,
    if (controlMsgFromISR(&micChannel, CH_DATA_RECEIVED, micChannel.state.ioChunk) != pdPASS) {
      micChannel.state.isrCtrlFails++;
    }
    // 3. transfer captured buffer to the fifo of unprocessed sound data,
    tu_fifo_write_n(&dataIn, nextBuffer, micChannel.state.ioChunk << micChannel.state.oversampling);
    // 4. let the codec know, and
    vTaskNotifyGiveFromISR(mic_codec_handle, &taskWoken);
    // 5. swap it in if need be
    portYIELD_FROM_ISR(taskWoken);
  }
  // Clear the interrupt
  dma_hw->ints0 = 1u << adcDmaBufCh;
}

typedef struct {
  int32_t count;
  int32_t sum;
  uint32_t await;
  int32_t current;
} running_avg_t;

static inline void updateAvg(running_avg_t *avg, int32_t sample) {
  avg->count++;
  avg->sum += sample;
  if ((avg->count & 0xfff) == 0) {
    int32_t roundedAvg = (avg->sum + 8) >> 4;
    if (avg->await > 0) {
      avg->await--;
      if (avg->await == 0) {
        avg->current = roundedAvg;
      }
    } else {
      avg->current += (roundedAvg - avg->current) >> 4;
    }
    avg->sum = 0;
    avg->count = 0;
    micChannel.state.offset = (avg->current + 128) >> 8;
  }
}

static void micCodecTask(void *pvParameters) {
  int16_t buf[MAX_OVER_SAMPLE * MAX_IO_CHUNK];
  running_avg_t avg = {.await = 2};
  while(true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // Loop as long as at least one oversampled item is available
    uint32_t availableSamples = (tu_fifo_count(&dataIn) >> micChannel.state.oversampling);
    while (availableSamples) {
      uint32_t count = availableSamples;
      if (count > MAX_IO_CHUNK) {
        count = MAX_IO_CHUNK;
      }
      tu_fifo_read_n(&dataIn, buf, count << micChannel.state.oversampling);
      availableSamples -= count;
      for (uint32_t i=0; i<count; i++) {
        int32_t reset = micChannel.state.resetCodec;
        if (reset) {
          avg.sum = 0;
          avg.count = 0;
          avg.await = 2;
          avg.current = 0;
          micChannel.state.resetCodec = false;
        }
        int32_t sample;
        for (uint32_t j=0; j<(1 << micChannel.state.oversampling); j++) {
          sample = buf[(i << micChannel.state.oversampling) + j];
          sample &= 0xfff;
          if (micChannel.state.autozero) {
            updateAvg(&avg, sample);
            if (avg.await) {
              sample = 0;
            } else {
              sample -= (avg.current >> 8);
            }
          } else {
            sample -= 2048;
          }
          sample = sample << micChannel.state.amplify;
          if (sample > 2047) {
            sample = 2047;
          } else if (sample < -2048) {
            sample = -2048;
          }
          if (micChannel.state.filter) {
            sample = iir_TrFII_filter(sample, reset, micChannel.state.filter);
          }
        }
        buf[i] = sample & micChannel.state.samplemask;
      }
      tu_fifo_write_n(&dataToUSB, buf, count);
    }
  }
}

static void openMicCh() {
  float clockDiv = ((float)USB_CLOCK_RATE / (float)(micChannel.state.sampleRate << micChannel.state.oversampling)) - 1.0;
  adc_set_clkdiv(clockDiv);
  dma_channel_set_trans_count(adcDmaCh, micChannel.state.ioChunk << micChannel.state.oversampling, false);
  dma_channel_start(adcDmaCh);
}

static void closeMicCh() {
  dma_channel_abort(adcDmaCh);
}

void setMicDiv(uint32_t clockDiv) {
  
}

static usb_channel_settings_t micChSettings = {
  // .state = &micCh,
  // .queue = &mic_cmd_q,
  .idStr = "Mic",
  .toUsb = true,
  .setDiv = setMicDiv,
  .open = openMicCh,
  .close = closeMicCh,
  .stateReg = MIC_AUDIO_STATE,
  .autoZeroReg = MIC_AUTOZERO,
  .overReg = MIC_OVERSAMPLE,
  .maskReg = MIC_BITMASK,
  .amplReg = MIC_AMPLIFY,
  .filterReg = MIC_FILTER
};

void createMicChannel() {
  // Set up ADC pin for audio in
  adc_init();
  adc_gpio_init(ADC_PIN);
  adc_select_input(0);
  adc_set_clkdiv(999.0); // 48 MHz / (999+1) => 48 kHz
  adc_fifo_setup(true, true, 1, false, false);
  adcDataPtr = &adc_hw->fifo;

  // Set up DMA channel for audio in
  adcDmaCh = dma_claim_unused_channel(true);
  adcDmaBufCh = dma_claim_unused_channel(true);

  dma_channel_config c1 = dma_channel_get_default_config(adcDmaCh);
  channel_config_set_transfer_data_size(&c1, DMA_SIZE_16);
  channel_config_set_write_increment(&c1, true);
  channel_config_set_read_increment(&c1, false);
  channel_config_set_dreq(&c1, DREQ_ADC);
  channel_config_set_chain_to(&c1, adcDmaBufCh); 

  dma_channel_configure(
    adcDmaCh,
    &c1,
    micBuf[0],      // Write address
    adcDataPtr,     // Read address
    24 * 4,         // Words to transfer
    false           // Start
  );

  dma_channel_config c2 = dma_channel_get_default_config(adcDmaBufCh);
  channel_config_set_transfer_data_size(&c2, DMA_SIZE_32);
  channel_config_set_write_increment(&c2, false);
  channel_config_set_read_increment(&c2, false);
  channel_config_set_dreq(&c2, DREQ_ADC);

  dma_channel_configure(
    adcDmaBufCh,
    &c2,
    &dma_channel_hw_addr(adcDmaCh)->al2_write_addr_trig,
    &nextBuffer,
    1,
    false
  );

  dma_channel_set_irq0_enabled(adcDmaBufCh, true);
  irq_set_exclusive_handler(DMA_IRQ_0, adcDmaHandler);
  irq_set_enabled(DMA_IRQ_0, true);

  adc_run(true);

  mic_codec_handle = xTaskCreateStatic(
    micCodecTask,
    "Mic codec",
    MIC_CODEC_STACK_SIZE,
    NULL,
    MIC_CODEC_TASK_PRIO,
    mic_codec_stack,
    &mic_codec_def
  );

  //vTaskCoreAffinitySet(mic_codec_handle, 1<<0);

  newChannelController(&micChSettings, &micChannel);
  //vTaskCoreAffinitySet(mic_ch_handle, 1<<0);

}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting) {
  int16_t buf[MAX_SAMPLES_PER_FRAME+4];
  uint32_t targetSamples = micChannel.state.usbChunk;
  if (micChannel.state.state == AUDIO_IDLE) {
    tu_fifo_t* ff = tud_audio_get_ep_in_ff();
    tu_fifo_clear(ff);
    tu_fifo_clear(&dataToUSB);
  //  tu_fifo_clear(&dataIn);
  //   tud_audio_write(zeroBuf, micCh.ioChunk);
  //   tud_audio_write(zeroBuf, micCh.ioChunk);
  // } else {
  //   uint16_t count = tu_fifo_count(ff);
  //   usbDebugSet(4, count);
  } else {
    uint32_t availableSamples = tu_fifo_count(&dataToUSB);
    if (availableSamples < (targetSamples << 1)) {
      targetSamples--;
    } else if (availableSamples > (targetSamples << 1)+targetSamples) {
      targetSamples++;
    }
    assert (targetSamples<MAX_SAMPLES_PER_FRAME+4);
    uint32_t count = tu_fifo_read_n(&dataToUSB, buf, targetSamples);
    tud_audio_write(buf, (count << 1));
  }
  return true;
}

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting) {
  if(controlMsg(&micChannel, CH_DATA_SENT, n_bytes_copied >> 1) != pdPASS) {
    micChannel.state.usbCtrlFails++;
  }
  return true;
}

