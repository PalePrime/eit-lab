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
#include "uac2_handling.h"
#include "mic_channel.h"
#include "channel_controller.h"

static StackType_t  mic_codec_stack[MIC_CODEC_STACK_SIZE];
static StaticTask_t mic_codec_def;
static TaskHandle_t mic_codec_handle;

usb_channel_t micChannel;


// These are computed at program start and then remain constant
static uint   adcDmaCh;
static volatile const uint32_t * adcDataPtr;
static volatile uint32_t * adcDivPtr;

// Buffer for incoming sound data
static int16_t micBuf[2 * MAX_IO_CHUNK];    // Double buffer incoming data from ADC 
static int16_t zeroBuf[MAX_IO_CHUNK];

static void initMicCh() {
}

static void openMicCh() {
  tu_fifo_t* ff = tud_audio_get_ep_in_ff();
  tu_fifo_clear(ff);
  tud_audio_write(zeroBuf, micChannel.state.ioChunk);
  //dma_channel_set_trans_count(adcDmaCh, micChannel.state.ioChunk, false);
  dma_channel_set_write_addr(adcDmaCh, micBuf, true);
}

static void closeMicCh() {
  dma_channel_abort(adcDmaCh);
}

uint32_t initialMicDiv(uint32_t sampleRate, uint32_t baseRate) {
  float clockDiv = ((float)baseRate / (float)sampleRate) - 1.0;
  adc_set_clkdiv(clockDiv); // Example: 48 MHz / (999+1) => 48 kHz
  return 0;
}

void setMicDiv(uint32_t clockDiv) {
  
}

static usb_channel_settings_t micChSettings = {
  // .state = &micCh,
  // .queue = &mic_cmd_q,
  .idStr = "Mic",
  .toUsb = true,
  .baseClock = USB_CLOCK_RATE,
  .init = initMicCh,
  .initialDiv = initialMicDiv,
  .setDiv = setMicDiv,
  .open = openMicCh,
  .close = closeMicCh
};

static void adcDmaHandler() {
  static uint32_t base = 0;
  BaseType_t taskWoken = pdFALSE;
  // ch_ctrl_msg_t msg = {
  //   .cmd = CH_DATA_RECEIVED,
  //   .count = micChannel.state.ioChunk,
  //   .time = to_us_since_boot(get_absolute_time())
  // };
  uint32_t lastBase = base;
  if (base == 0) {
    base = micChannel.state.ioChunk;
  } else {
    base = 0;
  }
  if (micChannel.state.state != AUDIO_IDLE) {
    dma_channel_set_write_addr(adcDmaCh, &micBuf[base], true);
    if (controlMsgFromISR(&micChannel, CH_DATA_RECEIVED, micChannel.state.ioChunk) != pdPASS) {
      micChannel.state.isrCtrlFails++;
    }
    xTaskNotifyFromISR(mic_codec_handle, lastBase, eSetValueWithOverwrite, &taskWoken);
    portYIELD_FROM_ISR(taskWoken);
  }
  dma_hw->ints0 = 1u << adcDmaCh;
}

static void micCodecTask(void *pvParameters) {
  int32_t high = INT16_MIN;
  int32_t low  = INT16_MAX;
  while(true) {
    uint32_t base = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int32_t sum = 0;
    for (uint32_t i=0; i<micChannel.state.ioChunk; i++) {
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
    tud_audio_write(&micBuf[base], micChannel.state.ioChunk << 1);
  }
}

void createMicChannel() {
  // Set up ADC pin for audio in
  adc_init();
  adc_gpio_init(ADC_PIN);
  adc_select_input(0);
  adc_set_clkdiv(999.0); // 48 MHz / (999+1) => 48 kHz
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
    micBuf,         // Write address
    adcDataPtr,     // Read address
    24,  // Words to transfer
    false           // Start
  );

  dma_channel_set_irq0_enabled(adcDmaCh, true);
  dma_channel_set_irq1_enabled(adcDmaCh, false);
  irq_set_exclusive_handler(DMA_IRQ_0, adcDmaHandler);
  irq_set_enabled(DMA_IRQ_0, true);

  adc_run(true);

  mic_codec_handle = xTaskCreateStatic(
    micCodecTask,
    "UAC2 In",
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
  tu_fifo_t* ff = tud_audio_get_ep_in_ff();
  if (micChannel.state.state == AUDIO_IDLE) {
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
  if(controlMsg(&micChannel, CH_DATA_SENT, n_bytes_copied >> 1) != pdPASS) {
    micChannel.state.usbCtrlFails++;
  }
  return true;
}

