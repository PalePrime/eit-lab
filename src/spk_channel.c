#include "FreeRTOS.h"
#include "task.h"

#include "bsp/board.h"
#include "tusb.h"

#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/structs/usb.h"
#include "hardware/structs/pwm.h"
#include "pico/stdlib.h"

#include "program_config.h"
#include "uac2_handling.h"
#include "spk_channel.h"

static StackType_t  spk_codec_stack[SPK_CODEC_STACK_SIZE];
static StaticTask_t spk_codec_def;
static TaskHandle_t spk_codec_handle;

usb_channel_t spkChannel;

static uint8_t pinState = 0;

// These are computed at program start and then remain constant
static uint   pwmSlice;                // The pwm slice that controls our pwm pin
static uint   pwmDmaCh;                // This DMA channel will feed data to the pwm pin
static uint   pwmDmaBufCh;             // This DMA channel will feed buffer addresses to the first DMA channel
static volatile uint32_t * pwmDataPtr; // Memory address controlling the pwm level
static volatile uint32_t * pwmDivPtr;  // Memory address controlling the pwm clock rate by setting a divider

// DMA buffers for outgoing sound data
static int16_t spkBuf[2][MAX_IO_CHUNK];

// Fifo for processed, outgoing sound data
// Use the fifo implementation from the TinyUSB package
TU_FIFO_DEF(dataOut, 256, int16_t, false);

// Buffer location used by 2nd DMA to initiate transfers by 1st DMA
static int16_t* nextBuffer = spkBuf[0];

// Interrupt Service Routine, needs to fill sound data into next buffer
// to be transferred by DMA to the pwm pin
static void pwmDmaHandler() {
  // When we get here, the second DMA channel (pwmDmaBufCh) has already
  // initiated a transfer to the buffer referenced by nextBuffer so we
  // should swap buffers and the fill
  if (nextBuffer != spkBuf[0]) {
    gpio_put(DBG_PIN, 1);
    nextBuffer = spkBuf[0];
  } else {
    nextBuffer = spkBuf[1];
  }
  // Unless channel has been marked as idle by our controller
  if (spkChannel.state.state != AUDIO_IDLE) {
    // 1. Inform controller that a chunk of data was sent, and
    if (controlMsgFromISR(&spkChannel, CH_DATA_SENT, spkChannel.state.ioChunk) != pdPASS) {
      spkChannel.state.isrCtrlFails++;
    }
    // 2. Fill the next buffer from the fifo of processed sound data
    if (tu_fifo_read_n(&dataOut, nextBuffer, spkChannel.state.ioChunk) < spkChannel.state.ioChunk) {
      spkChannel.state.underRuns++;
    }
  } else {
    // If idle close down transfers
    dma_channel_abort(pwmDmaCh);
  }
  // Clear the interrupt
  dma_hw->ints0 = 1u << pwmDmaBufCh;
  gpio_put(DBG_PIN, 0);
  // and go back to whatever went on before
}

static void spkCodecTask(void *pvParameters) {
  bool loop;
  int16_t buf[MAX_IO_CHUNK];
  while(true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    loop = true;
    while (loop) {
      uint32_t availableSamples = tud_audio_available() >> 1;
      if (availableSamples > MAX_IO_CHUNK) {
        availableSamples = MAX_IO_CHUNK;
      } else {
        loop = false;
      }
      tud_audio_read(buf, availableSamples << 1);
      for (uint32_t i=0; i<availableSamples; i++) {
        int16_t sample = buf[i];
        sample = sample>>8;           // -128 -- +127 in low byte, sign in high byte
        sample = (sample+128) & 0xff; //    0 --  255 in low byte, zero in high byte
        if (!tu_fifo_write(&dataOut, &sample)) {
            spkChannel.state.overRuns++;
        }
      }
    }
  }
}

static void initSpkCh() {
  pwm_set_enabled(pwmSlice, true);
}

static void openSpkCh() {
  dma_channel_set_trans_count(pwmDmaCh, spkChannel.state.ioChunk, false);
  dma_channel_start(pwmDmaBufCh);
}

static void closeSpkCh() {
}

uint32_t initialSpkDiv(uint32_t sampleRate, uint32_t baseRate) {
  return ((baseRate + (sampleRate >> 1)) / sampleRate) + (1 << 4);
}

static void setSpkDiv(uint32_t clockDiv) {
  *pwmDivPtr = (clockDiv >> 4);
}

static usb_channel_settings_t spkChSettings = {
  .idStr = "Spk",
  .toUsb = false,
  .baseClock = PWM_CLOCK_RATE,
  .init = initSpkCh,
  .initialDiv = initialSpkDiv,
  .setDiv = setSpkDiv,
  .open = openSpkCh,
  .close = closeSpkCh
};

void createSpkChannel() {
  // Dedicate one GPIO pin to assist debugging
  // this way we can look at time critical sequences
  // using an oscilloscope
  gpio_set_function(DBG_PIN, GPIO_FUNC_SIO);
  gpio_set_dir(DBG_PIN, GPIO_OUT);
  pinState = 0;
  gpio_put(DBG_PIN, pinState);

  // Compute a sane initial value for the pwm clock divider
  uint16_t pwmDiv = initialSpkDiv(AUDIO_SAMPLE_RATE, PWM_CLOCK_RATE);

  // Set up PWM pin for audio out
  gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);    // Enable pwm on pin
  pwmSlice = pwm_gpio_to_slice_num(PWM_PIN);    // Find out which pwm slice controls the pin
  pwm_set_wrap(pwmSlice, 255);                  // For now we do 8-bit resolution only
  pwmDataPtr = &pwm_hw->slice[pwmSlice].cc;     // Address controlling our pwm pin
  *pwmDataPtr = 16;                             // Start at 50% duty cycle, basically zero amplitude
  pwmDivPtr = &pwm_hw->slice[pwmSlice].div;     // Address controlling our pwn clock
  *pwmDivPtr = pwmDiv >> 4;                     // Set the initial sane value
  pwm_set_enabled(pwmSlice, true);              // and finally enable output
  
  // Obtain two unused DMA channels
  pwmDmaCh =    dma_claim_unused_channel(true); // One to actually push data from buffer to pwm
  pwmDmaBufCh = dma_claim_unused_channel(true); // and one to switch the buffer address in the first channel

  // The whole point of this setup is to allow some delay in executing the interrupt handler for DMA
  // there is no hardware fifo in the pwm slice so new data has to be written before the sample
  // interval ends, for 48 kHz this means that once a buffer is tranferred, the next has to start
  // within 20 us. With only the data DMA channel running and buffer changes performed by an
  // interrupt handler this is difficult i view of other interrupt handlers also running, servicing
  // e.g. the USB hardware.
  // Having a second DMA channel update the buffer address of the data channel losens the real time
  // demands a lot. Basically the second DMA swaps the buffer and then interrupts the CPU. This means
  // that the only requirement on the interrupt is to update the buffer and re-arm the second DMA
  // channel before it completes sending all data in the buffer.

  // Configure the first channel, i.e. DMA to transfer actual sound data
  dma_channel_config c1 = dma_channel_get_default_config(pwmDmaCh);
                                                            // Get a default setup to modify
  channel_config_set_transfer_data_size(&c1, DMA_SIZE_16);  // Feed 16-bit data to the pwm slice
  channel_config_set_write_increment(&c1, false);           // to the fixed pwm slice data location
  channel_config_set_read_increment(&c1, true);             // while incrementing the buffer address
  channel_config_set_dreq(&c1, DREQ_PWM_WRAP1);     // repeating each time one pwm cycle completes
  channel_config_set_chain_to(&c1, pwmDmaBufCh);            // and finally trigger the second DMA channel

  // Call to actually set up the hardware registers controlling the DMA channel
  dma_channel_configure(
    pwmDmaCh,
    &c1,
    pwmDataPtr,     // Write to pwm data address
    spkBuf,         // Read from buffer (this will be updated by the second DMA)
    24,             // Number of transfers (just a sane initial value, set when sample rate is adjusted)
    false           // Don't start yet
  );

  // Set up DMA channel for feeding buffer addresses to the first channel
  dma_channel_config c2 = dma_channel_get_default_config(pwmDmaBufCh);
  channel_config_set_transfer_data_size(&c2, DMA_SIZE_32);  // Feed 32-bit memory adresses to the first DMA channel
  channel_config_set_write_increment(&c2, false);           // to the fixed DMA controller address
  channel_config_set_read_increment(&c2, false);            // from a fixed location
  
  dma_channel_configure(
    pwmDmaBufCh,
    &c2,
    &dma_channel_hw_addr(pwmDmaCh)->al3_read_addr_trig, // Write next buffer pointer into read address reg of the first DMA channel
    &nextBuffer,                                        // copying pointers from the list of buffer locations
    1,                                                  // transfer a single pointer
    false                                               // Don't start yet
  );

  // dma_channel_set_irq0_enabled(pwmDmaBufCh, false);
  dma_channel_set_irq1_enabled(pwmDmaBufCh, true);      // Use interrupt vector DMA_IRQ_1, the mic channel uses DMA_IRQ_0
  irq_set_exclusive_handler(DMA_IRQ_1, pwmDmaHandler);  // Connect the vector to our ISR
  irq_set_enabled(DMA_IRQ_1, true);                     // and enable interrupts on that vector

  spk_codec_handle = xTaskCreateStatic(
    spkCodecTask,
    "Spk codec",
    SPK_CODEC_STACK_SIZE,
    NULL,
    SPK_CODEC_TASK_PRIO,
    spk_codec_stack,
    &spk_codec_def
  );

  //vTaskCoreAffinitySet(spk_codec_handle, 1<<0);
  newChannelController(&spkChSettings, &spkChannel);
  //vTaskCoreAffinitySet(spk_ch_handle, 1<<0);

}

bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
//  pinState = !pinState;
//  gpio_put(DBG_PIN, pinState);
  if(controlMsg(&spkChannel, CH_DATA_RECEIVED, n_bytes_received >> 1) != pdPASS) {
    spkChannel.state.usbCtrlFails++;
  }
//   tu_fifo_t *ff = tud_audio_get_ep_in_ff();
//   if (tu_fifo_remaining(ff) < n_bytes_received) {
//       spkChannel.state.overRuns++;
//   }
  return true;
}

bool tud_audio_rx_done_post_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
  xTaskNotifyGive(spk_codec_handle);
  return true;
}

