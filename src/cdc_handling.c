#include "FreeRTOS.h"
#include "task.h"

#include "bsp/board.h"
#include "tusb.h"

#include <ctype.h>

#include "pico/stdio.h"
#include "pico/stdio/driver.h"

#include "program_config.h"
#include "program_state.h"
#include "cdc_handling.h"
#include "program_state.h"

static StackType_t  cdc_stack[CDC_STACK_SIZE];
static StaticTask_t cdc_taskdef;
static TaskHandle_t cdc_handle;

static char inputBuffer[CMD_BUFFER];
static uint32_t putIdx = 0;
static uint32_t getIdx = 0;

static volatile uint32_t cdcWrBlock;
static volatile uint32_t flowControl;
static volatile uint32_t flowControlCalls;

void stdio_cdc_write(const char *buf, int len) {
  int32_t actual = len;
  int32_t offset = 0;
  while (actual>0) {
    int32_t avail = tud_cdc_write_available();
    int32_t count = MIN(MAX(avail, 64), actual);
    if (count > 0) {
      int32_t chunk = tud_cdc_write(buf+offset, count);
      actual -= chunk;
      offset += chunk;
    } else {
      cdcWrBlock++;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  // if (avail >= actual && actual > 0) {
  //   tud_cdc_write(buf, actual);
  // } else {
  //   cdcWrBlock++;
  // }
}

stdio_driver_t stdio_cdc = {
  .out_chars = stdio_cdc_write,
  .out_flush = (void (*)(void)) tud_cdc_write_flush,
  .in_chars  = (int (*)(char *, int))  tud_cdc_read,
  .crlf_enabled = true
};

void reportUSB_CDC() {
  printf("Stdio flow ctl:       %15lu %15lu\n", flowControl, flowControlCalls);
  printf("Stdio blocked:        %15lu\n",  cdcWrBlock);

}

static void cdcTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(20));
#if CFG_TUSB_DEBUG < 2
  stdio_set_driver_enabled(&stdio_cdc, true);
#endif
  while(true)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (tud_cdc_available())
    {
      uint8_t buf[64];
      uint32_t count = tud_cdc_read(buf, sizeof(buf));
      bool cmdComplete = false;
      uint32_t idx = 0;
      while (idx<count) {
        char c = buf[idx++];
        if (c>31 && c<128) {
          uint32_t putNext = putIdx;
          if (++putNext>=CMD_BUFFER) putNext=0;
          if (putNext==getIdx) break;
          inputBuffer[putIdx] = c;
          tud_cdc_write_char(c);
          putIdx = putNext;
        } else if (c==13) {
          tud_cdc_write_str("\r\nCommand was: ");
          uint32_t i = 0;
          char c2;
          while (getIdx!=putIdx) {
            c2 = inputBuffer[getIdx];
            if (i==0) {
              switch (toupper(c2))
              {
              case 'T':
                sendRegisterEvent(REPORT_MODE, SET, TOP_REPORTING);
                break;
              case 'U':
                sendRegisterEvent(REPORT_MODE, SET, USB_REPORTING);
                break;
              case 'S':
                sendRegisterEvent(REPORT_MODE, SET, STATE_REPORTING);
                break;
              case 'N':
                sendRegisterEvent(REPORT_MODE, SET, NO_REPORTING);
                break;
              default:
                break;
              }
            }
            i++;
            tud_cdc_write_char(c2);
            if (++getIdx>=CMD_BUFFER) getIdx=0;
          }
          tud_cdc_write_str("\r\n");
        }
      }
      tud_cdc_write_flush();
    }
  }
}

void createCDCHandler() {
  cdc_handle = xTaskCreateStatic(cdcTask,
    "USB CDC",
    CDC_STACK_SIZE,
    NULL,
    CDC_TASK_PRIO,
    cdc_stack,
    &cdc_taskdef);
}

void tud_cdc_rx_cb(uint8_t itf)
{
  xTaskNotifyGive(cdc_handle);
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
  flowControlCalls++;
  if (dtr) {
    flowControl |= 0x1;
  } else {
    flowControl &= 0xE;
  }
  if (rts) {
    flowControl |= 0x2;
  } else {
    flowControl &= 0xD;
  }

}
