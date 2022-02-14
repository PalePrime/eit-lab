#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <stdio.h>
#include "task_tracing.h"
#include "report_handling.h"
#include "program_state.h"
#include "spk_channel.h"
#include "mic_channel.h"

static StackType_t  report_stack[REPORT_STACK_SIZE];
static StaticTask_t report_taskdef;
static TaskHandle_t report_handle;

static StaticTimer_t report_tickdef;
static TimerHandle_t report_ticker;

static char scratch[S_BUFFER_SIZE+1];

static uint8_t debugBuffer[DEBUG_BUFFER];

static void statTimerCallback(TimerHandle_t h) {
  xTaskNotifyGive(report_handle);
}

static void reportTop() {
  uint32_t taskCount = traceGetTaskCount();

  uint64_t totalTime = traceTotalTime();
  uint32_t deltaTime = traceDeltaTime();
  uint32_t adjTime = deltaTime/100;

  if (adjTime<100) return;

  printf("%-12s %15s %12s %5s %5s %3s\n\n", "Tasks name", "Acc time", "Time", "Core0", "Core1", "%");
  for(uint32_t task = 0; task < taskCount; task++) {
    TaskInfoRecord info = traceGetInfo(task);
    uint32_t percent = info.deltaTime / adjTime;
    printf("%-12s %15llu %12lu %5lu %5lu %3lu\n",
              info.name,
              info.accumulatedTime,
              info.deltaTime,
              info.core0Start,
              info.core1Start,
              percent);
  }
  printf("%-12s %15llu %12lu\n",
          "Totals",
          totalTime,
          deltaTime);


}

static void reportState() {
  uint32_t reg = 0;
  printf("State:\n");
  while (reg < LAST_REGISTER_MARKER)
  {
    if(reg && (reg >> 2)<<2 == reg) {
      printf("\n");
    }
    printf("%03lu: %10lu ", reg, getRegister(reg));
    reg++;
  }
  printf("\n");
  
}

static void reportBuffer() {
  char txt[17];
  for (int r=0; r>>4 < DEBUG_BUFFER; r++) {
    printf("%3lu: ", r>>4);
    for (int i=0; i<16; i++) {
      int idx = r>>4 + i;
      if (idx < DEBUG_BUFFER) {
        char c = debugBuffer[idx];
        txt[i] = c<32 ? '.' : c;
        printf("%02x", c);
      } else {
        txt[i] = 0;
        printf("  ");
      }
    }
    txt[r>>4] = 0;
    printf("   %s\n", txt);
  }

}

void reportUSB() {

  usb_channel_state_t spkCh; 
  usb_channel_state_t micCh; 

  cloneChannelState(&spkCh, &spkChannel.state);
  cloneChannelState(&micCh, &micChannel.state);

  printf("Channel        %15s %15s\n\n", spkChannel.settings->idStr, micChannel.settings->idStr);

  printf("Audio state    %15s %15s\n", stateString(spkCh.state), stateString(micCh.state));
  
  printf("Frames in      %15lu %15lu\n",  spkCh.receiveCalls,     micCh.receiveCalls);
  printf("Frames out     %15lu %15lu\n",  spkCh.sendCalls,        micCh.sendCalls);
  printf("Sample rate    %15lu %15lu\n",  spkCh.sampleRate,       micCh.sampleRate);
  printf("Oversampling   %15lu %15lu\n",  spkCh.oversampling,     micCh.oversampling);
  printf("USB Chunk      %15lu %15lu\n",  spkCh.usbChunk,         micCh.usbChunk);
  printf("I/O Chunk      %15lu %15lu\n",  spkCh.ioChunk,          micCh.ioChunk);
  printf("Offset         %15li %15li\n",  spkCh.offset,           micCh.offset);
  printf("Q-len          %15li %15li\n",  spkCh.queuedSamples,    micCh.queuedSamples);
  printf("Q-len min      %15li %15li\n",  spkCh.minQueuedSamples, micCh.minQueuedSamples);
  printf("Q-len max      %15li %15li\n",  spkCh.maxQueuedSamples, micCh.maxQueuedSamples);
  printf("ClkDiv         %15lu %15lu\n",  spkCh.clkDiv,           micCh.clkDiv);
  printf("Underrun       %15lu %15lu\n",  spkCh.underRuns,        micCh.underRuns);
  printf("Overrun        %15lu %15lu\n",  spkCh.overRuns,         micCh.overRuns);
  printf("C-fails usb    %15lu %15lu\n",  spkCh.usbCtrlFails,     micCh.usbCtrlFails);
  printf("C-fails dma    %15lu %15lu\n",  spkCh.isrCtrlFails,     micCh.isrCtrlFails);
}

static void reportMessage() {
  printf("\f");
  getMessage(scratch);
  printf("Message: %s\n\n", scratch);
}

static void reportTask(void *pvParameters) {
  uint32_t count;
  sendRegisterEvent(REPORT_MODE, SET, NO_REPORTING);
  while(true)
  {
    count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (count) {
      switch (getRegister(REPORT_MODE))
      {
      case TOP_REPORTING:
        reportMessage();
        reportTop();
        break;
      case STATE_REPORTING:
        reportMessage();
        reportState();
        break;
      case USB_REPORTING:
        reportMessage();
        reportUSB();
      default:
        break;
      }
    }
    traceDeltaTick();
  }

}

void createReportHandler() {
  report_handle = xTaskCreateStatic(reportTask,
    "Reporter",
    REPORT_STACK_SIZE,
    NULL,
    REPORT_TASK_PRIO,
    report_stack,
    &report_taskdef);
  report_ticker = xTimerCreateStatic("Report Tick", pdMS_TO_TICKS(REPORT_TICK_MS), pdTRUE, (void *) 0, statTimerCallback, &report_tickdef);
  xTimerStart(report_ticker, pdMS_TO_TICKS(REPORT_TICK_MS));

  //vTaskCoreAffinitySet(report_handle, 1<<1);

}

