
#include "channel_controller.h"
#include "uac2_handling.h"

static void startChannel(usb_channel_state_t *state) {
  state->isrCtrlFails = 0;
  state->usbCtrlFails = 0;
  state->receiveCalls = 0;
  state->sendCalls = 0;
  state->queuedSamples = 0;
  state->overRuns = 0;
  state->underRuns = 0;
  state->maxQueuedSamples = -10000;
  state->minQueuedSamples =  10000;
  state->state = AUDIO_SYNC;
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

static void chController(void *pvParameters) {
  usb_channel_t *ch = (usb_channel_t *) pvParameters;
  usb_channel_settings_t *p = ch->settings;
  QueueHandle_t q = ch->cmd_q;
  usb_channel_state_t *state = &(ch->state);
  ch_ctrl_msg_t cmd;
  int64_t recTime = 0;
  int64_t sendTime = 0;
  int32_t sampleRate = AUDIO_SAMPLE_RATE;
  uint64_t recSamples = 0;
  uint64_t sendSamples = 0;
  
  state->clkDiv = (p->initialDiv)(AUDIO_SAMPLE_RATE, p->baseClock);
  (p->setDiv)(state->clkDiv);
  (p->init)();
  while (true) {
    if (xQueueReceive(q, &cmd, portMAX_DELAY) == pdTRUE) {
      switch (cmd.cmd) {
        case CH_SET_RATE:
          // printf("Ctrl %s w q-handle %x set rate %x\n", p->idStr, ch->cmd_q, cmd.count);
          state->clkDiv = (p->initialDiv)(cmd.count, p->baseClock);
          (p->setDiv)(state->clkDiv);
          sampleRate = cmd.count;
          (p->close)();
          state->state = AUDIO_IDLE;
          break;
        case CH_DATA_RECEIVED:
          if (!p->toUsb && state->state == AUDIO_IDLE) {
            // printf("Ctrl %s w q-handle %x recd %x\n", p->idStr, ch->cmd_q, cmd.count);
            sendTime = cmd.time;
            recSamples = 0;
            sendSamples = 0;
            startChannel(state);
            (p->open)();
          }
          state->receiveCalls++;
          recTime = cmd.time;
          recSamples += cmd.count;
          break;
        case CH_DATA_SENT:
          if (p->toUsb && state->state == AUDIO_IDLE) {
            // printf("Ctrl %s w q-handle %x sent %x\n", p->idStr, ch->cmd_q, cmd.count);
            recTime = cmd.time;
            recSamples = 0;
            sendSamples = 0;
            startChannel(state);
            (p->open)();
          }
          state->sendCalls++;
          sendTime = cmd.time;
          sendSamples += cmd.count;
          break;
        default:
          printf("Ctrl %s w q-handle %x bad cmd %x\n", p->idStr, ch->cmd_q, cmd.cmd);
          break;
      }
      if (state->state == AUDIO_SYNC && recTime > USB_SYNC_TIME && sendTime > USB_SYNC_TIME) {
        // Should now be in phase and ready for real data
        // printf("Ctrl %s w q-handle %x synced\n", p->idStr, ch->cmd_q);
        state->state = AUDIO_RUN;
      }
      if (state->state != AUDIO_IDLE) {
        int32_t currentLag = recSamples - sendSamples;
        int32_t estimatedTimeLag = (sampleRate * (sendTime - recTime)) >> 20;
        state->queuedSamples = currentLag + estimatedTimeLag;
        if (state->queuedSamples > state->maxQueuedSamples) {
          state->maxQueuedSamples = state->queuedSamples;
        }
        if (state->queuedSamples < state->minQueuedSamples) {
          state->minQueuedSamples = state->queuedSamples;
        }
        if (!p->toUsb) {
          if (state->queuedSamples < 100) {
            (p->setDiv)(state->clkDiv + (1 << 4));
          } else if (state->queuedSamples < 150) {
            (p->setDiv)(state->clkDiv);
          } else if (state->queuedSamples < 200) {
            (p->setDiv)(state->clkDiv - (1 << 4));
          } else {
            (p->setDiv)(state->clkDiv - (2 << 4));
          }
        }
        if (p->toUsb && recTime > sendTime + USB_SYNC_TIME ||
            !p->toUsb && sendTime > recTime + USB_SYNC_TIME) {
          // USB not active, stop channel
          // printf("Ctrl %s w q-handle %x closed\n", p->idStr, ch->cmd_q);
          (p->close)();
          state->state = AUDIO_IDLE;
        }
      }
    }
  }
}

void newChannelController(usb_channel_settings_t *channelSettings, usb_channel_t *channel) {

  channel->state.state = AUDIO_IDLE;
  channel->state.ioChunk = SAMPLES_PER_FRAME / 2;
  channel->settings = channelSettings;

  channel->cmd_q = xQueueCreateStatic(
    CH_CTRL_MSG_Q_LEN,
    CH_CTRL_MSG_SIZE,
    channel->cmd_q_buf,
    &channel->cmd_q_def
  ); 

  channel->handle = xTaskCreateStatic(
    chController,
    "Spk Ch",
    CH_STACK_SIZE,
    channel,
    CH_TASK_PRIO,
    channel->stack,
    &channel->taskdef
  );

}

