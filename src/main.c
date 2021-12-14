
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "bsp/board.h"
#include "tusb.h"

#include "hardware/resets.h"

#include "program_state.h"
#include "usb_handling.h"
#include "cdc_handling.h"
#include "uac2_handling.h"
#include "report_handling.h"
#include "display_handling.h"
#include "task_tracing.h"

int main(void) {
  sleep_ms(100);
  reset_block(RESETS_RESET_USBCTRL_BITS);
  sleep_ms(100);
  unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

  board_init();
  stdout_uart_init();

  createEventHandler();
  createCDCHandler();
  createUAC2Handler();
  createUSBHandler();
  createReportHandler();
  createDisplayHandler();

  traceInit();

  vTaskStartScheduler();
  /* should never reach here */
  panic_unsupported();  
}

