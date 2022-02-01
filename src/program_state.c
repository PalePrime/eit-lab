#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "pico/stdlib.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "program_state.h"

static StaticQueue_t event_queuedef;
static uint8_t       event_queuebuffer[EVENT_QUEUE_LENGTH*EVENT_MSG_SIZE];
static QueueHandle_t event_queue;

#define EVENT_STACK_SIZE configMINIMAL_STACK_SIZE

static StackType_t  event_stack[EVENT_STACK_SIZE];
static StaticTask_t event_taskdef;
static TaskHandle_t event_handle;

static StaticSemaphore_t state_lock_buffer;
static SemaphoreHandle_t state_lock;
static uint32_t state_lock_timeout;

static uint32_t registers[LAST_REGISTER_MARKER];

static uint8_t message[S_BUFFER_SIZE+1];

inline uint32_t getTime() {
  return to_ms_since_boot(get_absolute_time());
}

// Accessors w/o synchronization for singe 32-bit items, will be accessed atomically anyway

inline uint32_t getRegister(state_register_t reg) {
  assert(reg >= 0 && reg < LAST_REGISTER_MARKER);
  return registers[reg];
}

// Accessors that require a mutex

uint8_t getMessage(char *buf) {
  uint8_t success = false;
  if(xSemaphoreTake(state_lock, pdMS_TO_TICKS(1))) {
    strcpy(buf, message);
    xSemaphoreGive(state_lock);
    success = true;
  }
  buf[0] = 0;
  return success;
}


// Internal code below

// Muliple accesses, so needs locking
void setMessage(s_buffer_t buf) {
  assert(buf.size<=S_BUFFER_SIZE);
  if(xSemaphoreTake(state_lock, pdMS_TO_TICKS(1))) {
    memcpy(message, buf.buffer, buf.size);
    message[buf.size] = 0;
    xSemaphoreGive(state_lock);
  } else {
    state_lock_timeout++;
  }
}

// Single writer (this procedure) and atomic operation, no locking required
void updateRegister(state_operation_t operation) {
  uint32_t reg = operation.reg;
  assert(reg >= 0 && reg < LAST_REGISTER_MARKER);
  switch (operation.op_code)
  {
  case SET:
    registers[reg] = operation.value;
    break;
  case ADD:
    registers[reg] = registers[reg] + operation.value;
    break;
  default:
    break;
  }
}

void handleButton(button_info_t button) {
  switch (button.txt)
  {
  case 'A':
    registers[REPORT_MODE] = TOP_REPORTING;
    break;
  case 'B':
    registers[REPORT_MODE] = USB_REPORTING;
    break;
  default:
    break;
  }
}

static void eventTask(void *pvParameters) {
  BaseType_t status;
  uint32_t count;
  event_msg_t msg;
  uint8_t str[S_BUFFER_SIZE+1];

  while(true)
  {
    status = xQueueReceive(event_queue, &msg, portMAX_DELAY);
    if (status == pdTRUE) {
      registers[STATE_UPDATES]++;
      switch (msg.msg_type)
      {
      case REGISTER:
        updateRegister(msg.data.operation);
        break;
      case BUTTON:
        snprintf(str, sizeof(str), "Btn %c %s", msg.data.button_info.txt, msg.data.button_info.down ? "down" : "up");
        sendMessageEvent(str);
        handleButton(msg.data.button_info);
        break;
      case MESSAGE:
        setMessage(msg.data.s_buffer);
        if (getRegister(REPORT_MODE)==NO_REPORTING) {
          printf("Message: %s\n", message);
        }
      default:
        break;
      }
    }
  }
}

void createEventHandler() {
  event_queue = xQueueCreateStatic(
    EVENT_QUEUE_LENGTH,
    EVENT_MSG_SIZE,
    event_queuebuffer,
    &event_queuedef
  ); 
  event_handle = xTaskCreateStatic(eventTask,
    "Events",
    EVENT_STACK_SIZE,
    NULL,
    tskIDLE_PRIORITY + 2,
    event_stack,
    &event_taskdef
  );

  //vTaskCoreAffinitySet(event_handle, 1<<1);

  state_lock = xSemaphoreCreateMutexStatic(&state_lock_buffer);
}

// TODO: Queue raw data and perform the formatting in the event task
void sendMessageEventvf(const char *format, va_list va) {
  uint8_t str[S_BUFFER_SIZE+1];
  vsnprintf(str, S_BUFFER_SIZE, format, va);
  sendMessageEvent(str);
}

void sendMessageEventf(const char *format, ...) {
    va_list va;
    va_start(va, format);
    sendMessageEventvf(format, va);
    va_end(va);
}

void sendMessageEvent(char *str) {
  uint32_t len = strlen(str);
  if (len>S_BUFFER_SIZE) {
    len = S_BUFFER_SIZE;
  }
  event_msg_t msg;
  msg.msg_type = MESSAGE;
  msg.data.s_buffer.size=len;
  memcpy(msg.data.s_buffer.buffer, str, len);
  xQueueSend(event_queue, &msg, 0);
}

void sendRegisterEvent(state_register_t reg, state_op_code_t op, uint32_t value) {
  assert(reg >= 0 && reg < LAST_REGISTER_MARKER);
  event_msg_t msg;
  msg.msg_type = REGISTER;
  msg.data.operation.reg = reg;
  msg.data.operation.op_code = op;
  msg.data.operation.value = value;
  xQueueSend(event_queue, &msg, 0);
}

void sendButtonEvent(char txt, bool down) {
  event_msg_t msg;
  msg.msg_type = BUTTON;
  msg.data.button_info.txt = txt;
  msg.data.button_info.down = down;  
  xQueueSend(event_queue, &msg, 0);
}