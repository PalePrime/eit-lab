#include "FreeRTOS.h"
#include "task.h"

#include "tusb.h"

#include "program_config.h"
#include "uac2_handling.h"
#include "spk_channel.h"
#include "mic_channel.h"
#include "program_state.h"
#include "pico/stdlib.h"

static volatile uint32_t audioSampleRate = BASE_SAMPLE_RATE;

typedef struct rateItem_t {
  uint32_t minR;
  uint32_t maxR;
  uint32_t step;
} rateItem_t;

static const rateItem_t allRates[] = {
  {.minR = 96000, .maxR = 96000, .step =     0},
  {.minR = 16000, .maxR = 48000, .step = 16000},
  {.minR =  4000, .maxR =  8000, .step =  4000},
  {.minR =  1000, .maxR =  2000, .step =   500}, 
};

#define rateCount (sizeof(allRates) / sizeof(rateItem_t))

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
      audio_control_range_4_n_t(rateCount) rangef =
      {
        .wNumSubRanges = tu_htole16(rateCount),
      };
      for (uint32_t r = 0; r < rateCount; r++) {
        rangef.subrange[r].bMin = tu_htole32(allRates[r].minR);
        rangef.subrange[r].bMax = tu_htole32(allRates[r].maxR);
        rangef.subrange[r].bRes = tu_htole32(allRates[r].step);
      };
      // sendMessageEventf("Clock get freq range (%d, %d, %d)", (int)rangef.subrange[0].bMin, (int)rangef.subrange[0].bMax, (int)rangef.subrange[0].bRes);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
    }
  }
  else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
           request->bRequest == AUDIO_CS_REQ_CUR)
  {
    audio_control_cur_1_t cur_valid = { .bCur = 1 };
    // sendMessageEventf("Clock get is valid %u", cur_valid.bCur);
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
      // sendMessageEventf("Get channel %u volume %d %%", request->bChannelNumber, cur_vol.bCur);
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
    sendMessageEventf("Set channel %d volume: %d %%", request->bChannelNumber, volume[request->bChannelNumber]);
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
  return true;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request) {
  uint8_t const itf = tu_u16_low(p_request->wIndex);
  return true;
}


