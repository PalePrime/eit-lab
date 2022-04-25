#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "pico/stdlib.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "program_config.h"
#include "program_state.h"
#include "hardware/clocks.h"
#include "filters.h"

#define PWR_SAVE_PIN 23

static StaticQueue_t event_queuedef;
static uint8_t       event_queuebuffer[EVENT_QUEUE_LENGTH*EVENT_MSG_SIZE];
static QueueHandle_t event_queue;

static StackType_t  event_stack[EVENT_STACK_SIZE];
static StaticTask_t event_taskdef;
static TaskHandle_t event_handle;

static StaticSemaphore_t state_lock_buffer;
static SemaphoreHandle_t state_lock;
static uint32_t state_lock_timeout;

static uint32_t registers[LAST_REGISTER_MARKER];

static uint8_t  message[S_BUFFER_SIZE+1];
static uint64_t message_time;

static void update_pwr_smooth(uint32_t smooth) {
  gpio_put(PWR_SAVE_PIN, smooth);
}

extern void vPortSetupTimerInterrupt( void );

static void update_sys_clock(uint32_t freq) {
  set_sys_clock_khz(freq, true);
  vPortSetupTimerInterrupt();
}

static menu_item_value_t boolean_choices[] = {{.label = "On", .value = 1}, {.label = "Off", .value = 0}};
static menu_item_info_t boolean_info = {
  .choice_info = {
    .count = 2,
    .choices = boolean_choices
  }
};

static menu_item_value_t led_choices[] = {{.label = "Auto",  .value = LED_AUTO}, {.label = "Off", .value = LED_OFF},
                                          {.label = "On",    .value = LED_ON},   {.label = "Blink", .value = LED_BLINK}};
static menu_item_info_t led_info = {
  .choice_info = {
    .count = 4,
    .choices = led_choices
  }
};

static menu_item_value_t display_choices[] = {{.label = "On",   .value = DISPLAY_ON},  {.label = "Off", .value = DISPLAY_OFF},
                                              {.label = "Auto", .value = DISPLAY_AUTO}};
static menu_item_info_t display_info = {
  .choice_info = {
    .count = 3,
    .choices = display_choices
  }
};

static menu_item_value_t clock_choices[] = {{.label = " 96 MHz", .value =  96000}, {.label = "125 MHz", .value = 125000},
                                            {.label = "144 MHz", .value = 144000}, {.label = "192 MHz", .value = 192000}};
static menu_item_info_t clock_info = {
  .choice_info = {
    .count = 4,
    .choices = clock_choices
  }
};

static menu_item_value_t oversample_choices[] = {{.label = "x 1", .value = 0}, {.label = "x 2", .value = 1},
                                                 {.label = "x 4", .value = 2}};
static menu_item_info_t oversample_info = {
  .choice_info = {
    .count = 3,
    .choices = oversample_choices
  }
};

static menu_item_value_t filter_choices[] = {{.label = "None", .value = 0}, {.label = "LP8", .value = (uint32_t)&low_pass8}};
static menu_item_info_t filter_info = {
  .choice_info = {
    .count = 2,
    .choices = filter_choices
  }
};

static menu_item_value_t mask_choices[] = {{.label = "12 bits", .value = 0xffff}, {.label = " 8 bits", .value = 0xfff0},
                                           {.label = " 6 bits", .value = 0xffc0}, {.label = " 4 bits", .value = 0xff00},
                                           {.label = " 3 bits", .value = 0xfe00}, {.label = " 2 bits", .value = 0xfc00}};
static menu_item_info_t mask_info = {
  .choice_info = {
    .count = 6,
    .choices = mask_choices
  }
};

static menu_item_t general_menu[] = {
  {.kind = CHOICE_ITEM, .info = &led_info,     .text = "Led state",  .reg = LED_MAIN_STATE},
  {.kind = CHOICE_ITEM, .info = &boolean_info, .text = "Pwr smooth", .reg = PWR_SMOOTH_STATE, .update = &update_pwr_smooth},
  {.kind = CHOICE_ITEM, .info = &clock_info,   .text = "Clk freq",   .reg = SYS_CLK_FREQ,     .update = &update_sys_clock},
  {.kind = CHOICE_ITEM, .info = &display_info, .text = "Display",    .reg = DISPLAY_STATE}
};

static menu_item_t mic_menu[] = {
  {.kind = CHOICE_ITEM, .info = &oversample_info, .text = "Oversample",  .reg = MIC_OVERSAMPLE},
  {.kind = CHOICE_ITEM, .info = &mask_info,       .text = "AD Bits",     .reg = MIC_BITMASK},
  {.kind = CHOICE_ITEM, .info = &filter_info,     .text = "Filter",      .reg = MIC_FILTER},
  {.kind = CHOICE_ITEM, .info = &oversample_info, .text = "Pre-amplify", .reg = MIC_AMPLIFY},
  {.kind = CHOICE_ITEM, .info = &boolean_info,    .text = "Auto zero",   .reg = MIC_AUTOZERO}
};

static menu_item_t spk_menu[] = {
  {.kind = CHOICE_ITEM, .info = &oversample_info, .text = "Oversample",  .reg = SPK_OVERSAMPLE},
  {.kind = CHOICE_ITEM, .info = &mask_info,       .text = "DA Bits",     .reg = SPK_BITMASK}
};

static menu_item_t top_menu[] = {
  {.kind = SUB_MENU_ITEM, .text = "General", .sub_menu = &general_menu[0]},
  {.kind = SUB_MENU_ITEM, .text = "Mic",     .sub_menu = &mic_menu[0]},
  {.kind = SUB_MENU_ITEM, .text = "Speaker", .sub_menu = &spk_menu[0]},
};

#define menuSize(m) (sizeof(m) / sizeof(menu_item_t))

static menu_item_t menu_root = {.kind = SUB_MENU_ITEM, .text = "Menu", .parent = NULL, .sub_menu = &top_menu[0]};

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

menu_item_t *getMenuItemN(uint32_t n) {
  menu_item_t *item = (menu_item_t*) registers[MENU_ITEM];
  while (n>0 && item != NULL) {
    n--;
    item = item->parent;
  }
  return item;
}

uint32_t getMenuDepth() {
  uint32_t n = 0;
  menu_item_t *item = (menu_item_t*) registers[MENU_ITEM];
  while (item->parent != NULL) {
    n++;
    item = item->parent;
  }
  return n;
}

menu_state_t getMenuState() {
  return (menu_state_t) registers[MENU_STATE];
}

// Accessors that require a mutex

uint64_t getMessage(char *buf) {
  uint64_t msgTime = 0;
  if(xSemaphoreTake(state_lock, pdMS_TO_TICKS(1))) {
    msgTime = message_time;
    strcpy(buf, message);
    xSemaphoreGive(state_lock);
  }
  return msgTime;
}

static void setMenu(menu_item_t *menu, menu_state_t state) {
  if (menu == &menu_root || menu == NULL || state == MENU_ROOT_STATE) {
    registers[MENU_ITEM]    = (uint32_t) &menu_root;
    registers[MENU_STATE]   = MENU_ROOT_STATE;
    registers[MENU_COUNT]   = 0;
    registers[MENU_SELECT]  = 0;
    registers[MENU_CURRENT] = 0;
  } else {
    registers[MENU_ITEM]   = (uint32_t) menu;
    registers[MENU_STATE]  = state;
    if (state == MENU_NAV_STATE) {

    } else if (state == MENU_ENTRY_STATE) {
      assert(menu->kind != SUB_MENU_ITEM);
      uint32_t value = getRegister(menu->reg);
      switch (menu->kind) {
        case CHOICE_ITEM:
          registers[MENU_COUNT] = menu->info->choice_info.count;
          registers[MENU_SELECT]  = 0;
          registers[MENU_CURRENT] = 0;
          for (uint32_t i = 0; i < menu->info->choice_info.count; i++) {
            if (value == menu->info->choice_info.choices[i].value) {
              registers[MENU_SELECT]  = i;
              registers[MENU_CURRENT] = i;
            }
          }
          break;
        default:
          registers[MENU_COUNT]   = 0;
          registers[MENU_SELECT]  = 0;
          registers[MENU_CURRENT] = 0;
          break;
      }
    }
  }
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
  if (state == MENU_ROOT_STATE) {
    setMenu(menu->sub_menu, MENU_NAV_STATE);
  } else if (state == MENU_NAV_STATE) {
    if (menu->kind == SUB_MENU_ITEM) {
      setMenu(menu->sub_menu, MENU_NAV_STATE);
    } else {
      setMenu(menu, MENU_ENTRY_STATE);
    }
  } else {
    uint32_t value = registers[MENU_SELECT];
    switch (menu->kind) {
      case CHOICE_ITEM:
        value = menu->info->choice_info.choices[value].value;
        break;
      default:
        break;
    }
    setRegister(menu->reg, value);
    if (menu->update != NULL) {
      menu->update(value);
    }
    setMenu(menu, MENU_ENTRY_STATE);
  }
}

static void backMenu() {
  menu_item_t *menu = getMenuItem();
  menu_state_t state = getMenuState();
  if (state == MENU_NAV_STATE) {
    setMenu(menu->parent, MENU_NAV_STATE);
  } else if (state == MENU_ENTRY_STATE) {
    setMenu(menu, MENU_NAV_STATE);
  }
}

static void nextPrevValue(menu_item_t *menu, bool next) {
  uint32_t value = getRegister(menu->reg);
  uint32_t selected = registers[MENU_SELECT];
  uint32_t count = registers[MENU_COUNT];
  switch (menu->kind) {
    case CHOICE_ITEM:
      if (next) {
        selected++;
      } else {
        selected--;
      }
      if (selected >= count) {
        selected = 0;
      }
      registers[MENU_SELECT] = selected;
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
      setMenu(menu->next, MENU_NAV_STATE);
    } else if (!next && menu->prev != NULL) {
      setMenu(menu->prev, MENU_NAV_STATE);
    }
  } else if (state == MENU_ENTRY_STATE) {
    nextPrevValue(menu, next);
  }
}

// Muliple accesses, so needs locking
static void setMessage(s_buffer_t buf) {
  assert(buf.size<=S_BUFFER_SIZE);
  if(xSemaphoreTake(state_lock, pdMS_TO_TICKS(1))) {
    message_time = to_us_since_boot(get_absolute_time());
    memcpy(message, buf.buffer, buf.size);
    message[buf.size] = 0;
    xSemaphoreGive(state_lock);
  } else {
    state_lock_timeout++;
  }
}

static void handleButton(button_info_t button) {
  if (button.down) {
    if (getRegister(DISPLAY_STATE) == DISPLAY_OFF) {
      setRegister(DISPLAY_STATE, DISPLAY_ON);
      setMenu(getMenuItem(), getMenuState());
    } else if (getRegister(MENU_STATE) == MENU_ROOT_STATE) {
      enterMenu();
    } else {
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
}

static void eventTask(void *pvParameters) {
  BaseType_t status;
  uint32_t count;
  event_msg_t msg;
  uint8_t str[S_BUFFER_SIZE+1];

  while(true) {
    status = xQueueReceive(event_queue, &msg, portMAX_DELAY);
    if (status == pdTRUE) {
      registers[STATE_UPDATES]++;
      switch (msg.msg_type) {
        case REGISTER:
          updateRegister(msg.data.operation);
          break;
        case BUTTON:
          // snprintf(str, sizeof(str), "Btn %c %s", msg.data.button_info.txt, msg.data.button_info.down ? "down" : "up");
          // sendMessageEvent(str);
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

  setMenu(&menu_root, MENU_ROOT_STATE);
  initMenu(top_menu, menuSize(top_menu), &menu_root);
  initMenu(general_menu, menuSize(general_menu), &top_menu[0]);
  initMenu(mic_menu, menuSize(mic_menu), &top_menu[1]);
  initMenu(spk_menu, menuSize(spk_menu), &top_menu[2]);

  // Prepare to control power save mode to improve ADC performance
  gpio_set_function(PWR_SAVE_PIN, GPIO_FUNC_SIO);
  gpio_set_dir(PWR_SAVE_PIN, GPIO_OUT);

  setRegister(PWR_SMOOTH_STATE, 1);
  update_pwr_smooth(true);

  uint32_t sysClock = clock_get_hz(clk_sys);
  setRegister(SYS_CLK_FREQ, sysClock / 1000);

  setRegister(MIC_BITMASK, 0xffff);
  setRegister(SPK_BITMASK, 0xffff);

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
    EVENT_TASK_PRIO,
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