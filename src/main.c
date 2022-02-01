
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

#include "program_state.h"
#include "usb_handling.h"
#include "cdc_handling.h"
#include "uac2_handling.h"
#include "report_handling.h"
#include "display_handling.h"
#include "task_tracing.h"

#define PWR_SAVE_PIN 23

int main(void) {

  // Get USB into reset asap
  reset_block(RESETS_RESET_USBCTRL_BITS);
  // Delay to ensure USB host has noticed reset
  sleep_ms(1000);
  // Get USB out of reset
  unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

  // Turn off power save mode to improve ADC performance
  gpio_set_function(PWR_SAVE_PIN, GPIO_FUNC_SIO);
  gpio_set_dir(PWR_SAVE_PIN, GPIO_OUT);
  gpio_put(PWR_SAVE_PIN, true);

  // Need this for the debug probe to work as it connects
  // via SWD/JTAG to a single core (usually Core 0) and will
  // reset only that core
  multicore_reset_core1();
  
  // Make sure we get printf over serial port if USB fails
  stdout_uart_init();

  // Set upp all FreeRTOS tasks
  createEventHandler();
  createCDCHandler();
  createUAC2Handler();
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
