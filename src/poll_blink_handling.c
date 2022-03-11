#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "hardware/gpio.h"

#include "program_config.h"
#include "poll_blink_handling.h"
#include "program_state.h"

static StackType_t  poll_stack[EVENT_STACK_SIZE];
static StaticTask_t poll_taskdef;
static TaskHandle_t poll_handle;

static StaticTimer_t poll_tickdef;
static TimerHandle_t poll_ticker;

#define A_BUTTON 12U
#define B_BUTTON 13U
#define X_BUTTON 14U
#define Y_BUTTON 15U

#define INT_LED  25U
#define EXT_LED  03U

typedef struct button_t {
  uint8_t pin;
  char    txt;
  bool    state;
} button_t;

button_t buttons[] = {
  {
    .pin = A_BUTTON,
    .txt = 'A',
    .state = false
  },
  {
    .pin = B_BUTTON,
    .txt = 'B',
    .state = false
  },
  {
    .pin = X_BUTTON,
    .txt = 'X',
    .state = false
  },
  {
    .pin = Y_BUTTON,
    .txt = 'Y',
    .state = false
  }
};

static bool is_pressed(uint8_t button) {
  return !gpio_get(button);
}

static void pollTimerCallback(TimerHandle_t h) {
  xTaskNotifyGive(poll_handle);
}

static void pollTask(void *pvParameters) {
  uint32_t count;

  xTimerStart(poll_ticker, pdMS_TO_TICKS(POLL_TICK_MS));

  while(true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    count++;
    // Check buttons
    for(uint32_t i=0; i<4; i++) {
      button_t *button = &buttons[i];
      bool next = is_pressed(button->pin);
      if (next != button->state) {
        sendButtonEvent(button->txt, next);
        button->state = next;
      }
    }
    // Update LED
    uint32_t ledState = getRegister(LED_MAIN_STATE);
    bool ledOn = false;
    switch (ledState) {
      case LED_AUTO:
        ledOn = getRegister(MIC_AUDIO_STATE) != AUDIO_IDLE;
        break;
      case LED_ON:
        ledOn = true;
        break;
      case LED_BLINK:
        ledOn = (count >> 3) & 0x1;
        break;
      default:
        break;
    }
    gpio_put(INT_LED, ledOn);
  }
}

void createPollHandler() {

  gpio_set_function(A_BUTTON, GPIO_FUNC_SIO); gpio_set_dir(A_BUTTON, GPIO_IN); gpio_pull_up(A_BUTTON);
  gpio_set_function(B_BUTTON, GPIO_FUNC_SIO); gpio_set_dir(B_BUTTON, GPIO_IN); gpio_pull_up(B_BUTTON);
  gpio_set_function(X_BUTTON, GPIO_FUNC_SIO); gpio_set_dir(X_BUTTON, GPIO_IN); gpio_pull_up(X_BUTTON);
  gpio_set_function(Y_BUTTON, GPIO_FUNC_SIO); gpio_set_dir(Y_BUTTON, GPIO_IN); gpio_pull_up(Y_BUTTON);

  gpio_set_function(INT_LED, GPIO_FUNC_SIO); gpio_set_dir(INT_LED, GPIO_OUT); gpio_put(INT_LED, 0);

  poll_handle = xTaskCreateStatic(pollTask,
    "Poll",
    POLL_STACK_SIZE,
    NULL,
    POLL_TASK_PRIO,
    poll_stack,
    &poll_taskdef
  );

  vTaskCoreAffinitySet(poll_handle, 1<<0);

  poll_ticker = xTimerCreateStatic("Poll Tick", pdMS_TO_TICKS(POLL_TICK_MS), pdTRUE, (void *) 0, pollTimerCallback, &poll_tickdef);

}

