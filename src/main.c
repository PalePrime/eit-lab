
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
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

#define PWR_SAVE_PIN 23

int main(void) {

  // Turn off power save mode to improve ADC performance
  gpio_set_function(PWR_SAVE_PIN, GPIO_FUNC_SIO);
  gpio_set_dir(PWR_SAVE_PIN, GPIO_OUT);
  gpio_put(PWR_SAVE_PIN, true);

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

