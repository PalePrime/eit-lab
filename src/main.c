
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"

#include "bsp/board.h"
#include "tusb.h"

#include "hardware/resets.h"

#include "program_config.h"
#include "program_state.h"
#include "poll_blink_handling.h"
#include "usb_handling.h"
#include "cdc_handling.h"
#include "spk_channel.h"
#include "mic_channel.h"
#include "report_handling.h"
#include "display_handling.h"
#include "task_tracing.h"

int main(void) {

  //set_sys_clock_khz(SYS_CLOCK_RATE / 1000, true);

  // When working with a debug probe connected to core 0 this
  // is needed to take core 1 to reset state, not required
  // after hardware reset but won't hurt
  multicore_reset_core1();

  // Make sure we get printf over serial port if USB fails
  stdout_uart_init();

  // Set upp all FreeRTOS tasks
  createEventHandler();
  createPollHandler();
  createCDCHandler();
  createSpkChannel();
  createMicChannel();
  createUSBHandler();
  createReportHandler();
  createDisplayHandler();

  // Initiate time metering on tasks
  traceInit();

  // Get FreeRTOS scheduler going
  vTaskStartScheduler();
  
  // Scheduler only returns on fatal errors
  panic("RTOS died");  
}
