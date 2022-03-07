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

static menu_item_t top_menu[] = {
  {.kind = BINARY_ITEM, .text = "Led on",  .reg = LED_ON_STATE},
  {.kind = BINARY_ITEM, .text = "Display", .reg = UPDATE_DISPLAY}
};

#define menuSize(m) (sizeof(m) / sizeof(menu_item_t))

static menu_item_t menu_root = {.kind = SUB_MENU_ITEM, .text = "Menu", .parent = &menu_root, .sub_menu = &top_menu[0]};

inline uint32_t getTime() {
  return to_ms_since_boot(get_absolute_time());
}

// Accessor w/o synchronization for singe 32-bit items, will be accessed atomically anyway
inline uint32_t getRegister(state_register_t reg) {
  assert(reg >= 0 && reg < LAST_REGISTER_MARKER);
  return registers[reg];
}

// Single writer (eventTask) and atomic operation, no locking required
static inline void setRegister(uint32_t reg, uint32_t value) {
  assert(reg >= 0 && reg < LAST_REGISTER_MARKER);
  registers[reg] = value;
}

static void updateRegister(state_operation_t operation) {
  uint32_t reg = operation.reg;
  switch (operation.op_code)
  {
  case SET:
    setRegister(reg, operation.value);
    break;
  case ADD:
    setRegister(reg, registers[reg] + operation.value);
    break;
  default:
    break;
  }
}

menu_item_t *getMenuItem() {
  return (menu_item_t*) registers[MENU_ITEM];
}

menu_state_t getMenuState() {
  return (menu_state_t) registers[MENU_STATE];
}

// Accessors that require a mutex

uint8_t getMessage(char *buf) {
  uint8_t success = false;
  if(xSemaphoreTake(state_lock, pdMS_TO_TICKS(1))) {
    strcpy(buf, message);
    xSemaphoreGive(state_lock);
    success = true;
  }
  return success;
}

static inline void setMenuState(menu_state_t state) {
  registers[MENU_STATE] = state;
}

static inline void setMenuItem(menu_item_t *menu) {
  registers[MENU_ITEM] = (uint32_t) menu;
  setMenuState(MENU_NAV_STATE);
}

static void initMenu(menu_item_t menu[], uint32_t size, menu_item_t *parent) {
  for (uint32_t i=0; i<size; i++) {
    menu[i].parent = parent;
    if (i==0) {
      menu[i].prev = &menu[size-1];
    } else {
      menu[i].prev = &menu[i-1];
    }
    if (i==size-1) {
      menu[i].next = &menu[0];
    } else {
      menu[i].next = &menu[i+1];
    }
  }
}

static void enterMenu() {
  menu_item_t *menu = getMenuItem();
  menu_state_t state = getMenuState();
  if (state == MENU_NAV_STATE) {
    if (menu->kind == SUB_MENU_ITEM) {
      if (menu->sub_menu != NULL) {
        setMenuItem(menu->sub_menu);
      }
    } else {
      setMenuState(MENU_ENTRY_STATE);
    }
  }
}

static void backMenu() {
  menu_item_t *menu = getMenuItem();
  menu_state_t state = getMenuState();
  if (state == MENU_NAV_STATE) {
    if (menu->parent != NULL) {
      setMenuItem(menu->parent);
    }
  } else {
    setMenuState(MENU_NAV_STATE);
  }
}

static void nextPrevValue(menu_item_t *menu, bool next) {
  uint32_t value = getRegister(menu->reg);
  switch (menu->kind) {
    case BINARY_ITEM:
      setRegister(menu->reg, !value);
      break;
    
    default:
      break;
  }
}

static void nextPrevMenu(bool next) {
  menu_item_t *menu = getMenuItem();
  menu_state_t state = getMenuState();
  if (state == MENU_NAV_STATE) {
    if (next && menu->next != NULL) {
      setMenuItem(menu->next);
    } else if (!next && menu->prev != NULL) {
      setMenuItem(menu->prev);
    }
  } else {
    nextPrevValue(menu, next);
  }
}

// Muliple accesses, so needs locking
static void setMessage(s_buffer_t buf) {
  assert(buf.size<=S_BUFFER_SIZE);
  if(xSemaphoreTake(state_lock, pdMS_TO_TICKS(1))) {
    memcpy(message, buf.buffer, buf.size);
    message[buf.size] = 0;
    xSemaphoreGive(state_lock);
  } else {
    state_lock_timeout++;
  }
}

static void handleButton(button_info_t button) {
  if (button.down) {
    switch (button.txt) {
      case 'A':
        // registers[REPORT_MODE] = TOP_REPORTING;
        enterMenu();
        break;
      case 'B':
        // registers[REPORT_MODE] = USB_REPORTING;
        backMenu();
        break;
      case 'X':
        nextPrevMenu(false);
        break;
      case 'Y':
        nextPrevMenu(true);
        break;
      default:
        break;
    }
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

  setMenuItem(&menu_root);
  initMenu(top_menu, menuSize(top_menu), &menu_root);

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

  vTaskCoreAffinitySet(event_handle, 1<<0);

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