#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

#include "bsp/board.h"
#include "tusb.h"

#include "program_config.h"
#include "program_state.h"
#include "uac2_handling.h"
#include "cdc_handling.h"

static StackType_t  usb_stack[USB_STACK_SIZE];
static StaticTask_t usb_taskdef;
static TaskHandle_t usb_handle;

static volatile uint64_t usbTaskCalls;

void reportUSB() {
  printf("USB:\n");
  printf("USB Task calls:       %15llu\n", usbTaskCalls);
  printf("\n");

  reportUSB_UAC2();
  reportUSB_CDC();
}

static void usbTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(10));
  tusb_init();
  while(true)
  {
    usbTaskCalls++;
    tud_task();
    vTaskDelay(1);
  }
}

void createUSBHandler() {
  usb_handle = xTaskCreateStatic(usbTask,
    "USB Main",
    USB_STACK_SIZE,
    NULL,
    USB_TASK_PRIO,
    usb_stack,
    &usb_taskdef);
}

//--------------------------------------------------------------------+
// USB callbacks, called in the usbTask context
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  assert(xTaskGetCurrentTaskHandle()==usb_handle);
  sendRegisterEvent(USB_PORT_STATE, SET, USB_MOUNTED);
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  sendRegisterEvent(USB_PORT_STATE, SET, USB_NOT_MOUNTED);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  sendRegisterEvent(USB_PORT_STATE, SET, USB_SUSPENDED);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  sendRegisterEvent(USB_PORT_STATE, SET, USB_MOUNTED);
}

