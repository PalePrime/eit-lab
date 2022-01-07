
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/irq.h"
//#include "pico/multicore.h"

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

void fix_irq_priorities() {
  irq_set_priority(USBCTRL_IRQ, 255);  
}

int main(void) {
  fix_irq_priorities();
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

