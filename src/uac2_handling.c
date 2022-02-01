#include "FreeRTOS.h"
#include "task.h"

#include "bsp/board.h"
#include "tusb.h"

#include "program_config.h"
#include "uac2_handling.h"
#include "channel_controller.h"
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

static usb_channel_t spkChannel;
static usb_channel_t micChannel;

// static StackType_t  spk_ch_stack[SPK_CH_STACK_SIZE];
// static StaticTask_t spk_ch_taskdef;
// static TaskHandle_t spk_ch_handle;

// static StackType_t  mic_ch_stack[MIC_CH_STACK_SIZE];
// static StaticTask_t mic_ch_taskdef;
// static TaskHandle_t mic_ch_handle;

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

// Buffer for incoming sound data
static int16_t micBuf[2 * MAX_IO_CHUNK];    // Double buffer incoming data from ADC 
static int16_t zeroBuf[MAX_IO_CHUNK];

// Buffer for outgoing sound data
static int16_t spkBuf[2 * MAX_IO_CHUNK];

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

  usb_channel_state_t spkCh = spkChannel.state;
  usb_channel_state_t micCh = micChannel.state;

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
  // ch_ctrl_msg_t msg = {
  //   .cmd = CH_DATA_SENT,
  //   .count = spkChannel.state.ioChunk,
  //   .time = to_us_since_boot(get_absolute_time())
  // };
  uint32_t lastBase = base;
  if (base == 0) {
    base = spkChannel.state.ioChunk;
  } else {
    base = 0;
  }
  if (spkChannel.state.state != AUDIO_IDLE) {
    dma_channel_set_read_addr(pwmDmaCh, &spkBuf[base], true);
    if (controlMsgFromISR(&spkChannel, CH_DATA_SENT, spkChannel.state.ioChunk) != pdPASS) {
      spkChannel.state.isrCtrlFails++;
    }
    xTaskNotifyFromISR(uac2_out_handle, lastBase, eSetValueWithOverwrite, &taskWoken);
    portYIELD_FROM_ISR(taskWoken);
  }
  dma_hw->ints0 = 1u << pwmDmaCh;
}

static void uac2OutTask(void *pvParameters) {

  int16_t buf[MAX_IO_CHUNK];
  while(true) {
    uint32_t base = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uint32_t availableSamples = tud_audio_available() >> 1;
    if (availableSamples >= spkChannel.state.ioChunk) {
      tud_audio_read(buf, spkChannel.state.ioChunk << 1);
    } else {
      memset(buf, 0, spkChannel.state.ioChunk << 1);
      usbDebugInc(16);
    }
    for (uint32_t i=0; i<spkChannel.state.ioChunk; i++) {
      int16_t sample = buf[i];
      sample = sample>>8;           // -128 -- +127 in low byte, sign in high byte
      sample = (sample+128) & 0xff; //    0 --  255 in low byte, zero in high byte
      spkBuf[base + i] = sample;
    }
  }
}


static void initSpkCh() {
  pwm_set_enabled(pwmSlice, true);
}

static void openSpkCh() {
  dma_channel_set_trans_count(pwmDmaCh, spkChannel.state.ioChunk, false);
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
  // .state = &spkCh,
  // .queue = &spk_cmd_q,
  .idStr = "Spk",
  .toUsb = false,
  .debug = true,
  .baseClock = PWM_CLOCK_RATE,
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
  .debug = false,
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
    xTaskNotifyFromISR(uac2_in_handle, lastBase, eSetValueWithOverwrite, &taskWoken);
    portYIELD_FROM_ISR(taskWoken);
  }
  dma_hw->ints0 = 1u << adcDmaCh;
}

static void uac2InTask(void *pvParameters) {
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

void createUAC2Handler() {
  gpio_set_function(DBG_PIN, GPIO_FUNC_SIO);
  gpio_set_dir(DBG_PIN, GPIO_OUT);
  pinState = 0;
  gpio_put(DBG_PIN, pinState);

  //uint16_t pwmDiv = initialSpkDiv(audioSampleRate, spkChSettings.baseClock);

  // Set up PWM pin for audio out
  gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
  pwmSlice = pwm_gpio_to_slice_num(PWM_PIN);
  pwm_set_wrap(pwmSlice, 255);
  pwmDataPtr = &pwm_hw->slice[pwmSlice].cc;
  *pwmDataPtr = 16;
  pwmDivPtr = &pwm_hw->slice[pwmSlice].div;
  *pwmDivPtr = (((PWM_CLOCK_RATE + (AUDIO_SAMPLE_RATE >> 1)) / AUDIO_SAMPLE_RATE) + (1 << 4)) >> 4;
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
    spkBuf,           // Read address
    24,
    false             // Start
  );

  dma_channel_set_irq0_enabled(pwmDmaCh, false);
  dma_channel_set_irq1_enabled(pwmDmaCh, true);
  irq_set_exclusive_handler(DMA_IRQ_1, pwmDmaHandler);
  irq_set_enabled(DMA_IRQ_1, true);

  audioSampleRate = AUDIO_SAMPLE_RATE;
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

  uac2_out_handle = xTaskCreateStatic(
    uac2OutTask,
    "UAC2 Out",
    UAC2_OUT_STACK_SIZE,
    NULL,
    UAC2_OUT_TASK_PRIO,
    uac2_out_stack,
    &uac2_out_taskdef
  );

  //vTaskCoreAffinitySet(uac2_out_handle, 1<<0);

  uac2_in_handle = xTaskCreateStatic(
    uac2InTask,
    "UAC2 In",
    UAC2_IN_STACK_SIZE,
    NULL,
    UAC2_IN_TASK_PRIO,
    uac2_in_stack,
    &uac2_in_taskdef
  );

  //vTaskCoreAffinitySet(uac2_in_handle, 1<<0);

  // spk_cmd_q = xQueueCreateStatic(
  //   CH_CTRL_MSG_Q_LEN,
  //   CH_CTRL_MSG_SIZE,
  //   spk_cmd_q_buf,
  //   &spk_cmd_q_def
  // ); 

  // mic_cmd_q = xQueueCreateStatic(
  //   CH_CTRL_MSG_Q_LEN,
  //   CH_CTRL_MSG_SIZE,
  //   mic_cmd_q_buf,
  //   &mic_cmd_q_def
  // ); 

  // spk_ch_handle = xTaskCreateStatic(
  //   chController,
  //   "Spk Ch",
  //   SPK_CH_STACK_SIZE,
  //   &spkChSettings,
  //   SPK_CH_TASK_PRIO,
  //   spk_ch_stack,
  //   &spk_ch_taskdef
  // );

  newChannelController(&spkChSettings, &spkChannel);
  //vTaskCoreAffinitySet(spk_ch_handle, 1<<0);

  // mic_ch_handle = xTaskCreateStatic(
  //   chController,
  //   "Mic Ch",
  //   MIC_CH_STACK_SIZE,
  //   &micChSettings,
  //   MIC_CH_TASK_PRIO,
  //   mic_ch_stack,
  //   &mic_ch_taskdef
  // );

  newChannelController(&micChSettings, &micChannel);
  //vTaskCoreAffinitySet(mic_ch_handle, 1<<0);

}

bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
  pinState = !pinState;
  gpio_put(DBG_PIN, pinState);
  if(controlMsg(&spkChannel, CH_DATA_RECEIVED, n_bytes_received >> 1) != pdPASS) {
    spkChannel.state.usbCtrlFails++;
  }
  return true;
}

bool tud_audio_rx_done_post_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
  return true;
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
      controlMsg(&micChannel, CH_SET_RATE, audioSampleRate);
      controlMsg(&spkChannel, CH_SET_RATE, audioSampleRate);
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
        .subrange[0] = { .bMin = tu_htole16(0), tu_htole16(100), tu_htole16(1) }
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


