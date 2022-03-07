#ifndef PROG_STATE_H
#define PROG_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    USB_NOT_MOUNTED = 0,
    USB_MOUNTED,
    USB_SUSPENDED
} usb_port_state_t;

typedef enum {
    NO_REPORTING,
    STATE_REPORTING,
    TOP_REPORTING,
    USB_REPORTING
} report_mode_t;

typedef enum {
  AUDIO_IDLE,
  AUDIO_SYNC,
  AUDIO_RUN,
  AUDIO_ERROR
} audio_state_t;

inline const char* stateString(audio_state_t state) {
  switch (state) {
  case AUDIO_IDLE:
    return "Idle";
    break;
  case AUDIO_SYNC:
    return "Sync";
    break;
  case AUDIO_RUN:
    return "Run";
    break;
  default:
    return "Error";
    break;
  }
}

typedef enum {
    REPORT_MODE,
    USB_PORT_STATE,
    STATE_LOCK_MISS,
    STATE_UPDATES,

    MIC_AUDIO_STATE,
    SPK_AUDIO_STATE,
    MENU_STATE,
    MENU_ITEM,

    UPDATE_DISPLAY,
    LED_ON_STATE,
    LED_DIM_STATE,
    LED_RATE_STATE,

    LAST_REGISTER_MARKER // Keep at end, used to allocate register storage
} state_register_t;

typedef enum {
    SET,
    ADD,
    
} state_op_code_t;

typedef enum {
    REGISTER,
    BUTTON,
    MESSAGE,
    MESSAGE_F
} event_msg_type_t;

#define S_BUFFER_SIZE 64

typedef struct {
    uint8_t size;
    uint8_t partial;
    uint8_t buffer[S_BUFFER_SIZE];
} s_buffer_t;

typedef struct {
    state_register_t reg;
    state_op_code_t  op_code;
    uint32_t value;
} state_operation_t;

typedef struct {
    char txt;
    bool down;
} button_info_t;

typedef union {
    state_operation_t operation;
    button_info_t button_info;
    s_buffer_t s_buffer;
} event_msg_data_t;

typedef struct {
    event_msg_type_t msg_type;
    event_msg_data_t data;
} event_msg_t;

typedef enum menu_state_t {
  MENU_NAV_STATE,
  MENU_ENTRY_STATE
} menu_state_t;

typedef enum menu_item_kind_t {
  SUB_MENU_ITEM,
  BINARY_ITEM,
  INTEGER_ITEM
} menu_item_kind_t;

typedef struct menu_item_t {
  menu_item_kind_t kind;
  const char *text;
  state_register_t reg;
  struct menu_item_t *parent;
  struct menu_item_t *next;
  struct menu_item_t *prev;
  struct menu_item_t *sub_menu;
} menu_item_t;

#define EVENT_QUEUE_LENGTH 20
#define EVENT_MSG_SIZE     sizeof(event_msg_t)

uint32_t getTime();
uint32_t getRegister(state_register_t reg);
uint8_t  getMessage(char *buf);

menu_item_t *getMenuItem();
menu_state_t getMenuState();

void sendRegisterEvent(state_register_t reg, state_op_code_t op, uint32_t value);
void sendButtonEvent(char txt, bool down);
void sendMessageEvent(char *str);
void sendMessageEventf(const char *format, ...);

void createEventHandler();

#ifdef __cplusplus
}
#endif

#endif /*PROG_STATE_H*/
