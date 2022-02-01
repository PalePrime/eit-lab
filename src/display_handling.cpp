#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <string.h>
#include <math.h>
#include <cstdlib>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

#include "libraries/pico_explorer/pico_explorer.hpp"

#include "program_config.h"
#include "display_handling.h"
#include "program_state.h"

using namespace pimoroni;

uint16_t buffer[PicoExplorer::WIDTH * PicoExplorer::HEIGHT];
PicoGraphics graphics(PicoExplorer::WIDTH, PicoExplorer::HEIGHT, buffer);

ST7789 screen = ST7789(PicoExplorer::WIDTH, PicoExplorer::HEIGHT, buffer, PICO_EXPLORER_ONBOARD);

spi_inst_t* spi;
uint cs;
uint dc;
uint32_t dma_channel;

const uint8_t MOTOR1N = 8;
const uint8_t MOTOR1P = 9;
const uint8_t MOTOR2N = 10;
const uint8_t MOTOR2P = 11;

struct pt {
    float      x;
    float      y;
    uint8_t    r;
    float     dx;
    float     dy;
    uint16_t pen;
};

pt shapes[SHAPE_CNT];

struct button {
  uint8_t pin;
  char    txt;
  bool    state;
};

button buttons[4];
char scratch[S_BUFFER_SIZE+1];

static bool is_pressed(uint8_t button) {
  return !gpio_get(button);
}

// static float get_adc(uint8_t channel) {
//   adc_select_input(channel);
//   // scale raw 12-bit adc value to 0 .. 1 float
//   float result = float(adc_read()) / (1 << 12);
//   // clamp result to 0 .. 1
//   result = std::min(1.0f, std::max(0.0f, result));
//   return result;
// }


void display_init() {

  buttons[0] = {
    .pin = PicoExplorer::A,
    .txt = 'A',
    .state = false
  };

  buttons[1] = {
    .pin = PicoExplorer::B,
    .txt = 'B',
    .state = false
  };

  buttons[2] = {
    .pin = PicoExplorer::X,
    .txt = 'X',
    .state = false
  };

  buttons[3] = {
    .pin = PicoExplorer::Y,
    .txt = 'Y',
    .state = false
  };

  screen.init();

  spi = screen.get_spi();
  cs  = screen.get_cs();
  dc  = screen.get_dc();

  // initialise dma channel for transmitting pixel data to screen
  dma_channel = dma_claim_unused_channel(true);
  dma_channel_config config = dma_channel_get_default_config(dma_channel);
  channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
  channel_config_set_dreq(&config, spi_get_index(spi) ? DREQ_SPI1_TX : DREQ_SPI0_TX);
  dma_channel_configure(
    dma_channel, &config, &spi_get_hw(spi)->dr, buffer, sizeof(buffer), false);
  dma_channel_set_irq0_enabled(dma_channel, false);
  dma_channel_set_irq1_enabled(dma_channel, false);


  gpio_set_function(PicoExplorer::A, GPIO_FUNC_SIO); gpio_set_dir(PicoExplorer::A, GPIO_IN); gpio_pull_up(PicoExplorer::A);
  gpio_set_function(PicoExplorer::B, GPIO_FUNC_SIO); gpio_set_dir(PicoExplorer::B, GPIO_IN); gpio_pull_up(PicoExplorer::B);
  gpio_set_function(PicoExplorer::X, GPIO_FUNC_SIO); gpio_set_dir(PicoExplorer::X, GPIO_IN); gpio_pull_up(PicoExplorer::X);
  gpio_set_function(PicoExplorer::Y, GPIO_FUNC_SIO); gpio_set_dir(PicoExplorer::Y, GPIO_IN); gpio_pull_up(PicoExplorer::Y);

  // setup ADC channels
  // adc_init();
  // const uint8_t ADC_BASE_PIN = 26;
  // adc_gpio_init(PicoExplorer::ADC0 + ADC_BASE_PIN);
  // adc_gpio_init(PicoExplorer::ADC1 + ADC_BASE_PIN);
  // adc_gpio_init(PicoExplorer::ADC2 + ADC_BASE_PIN);

  // setup motor pins
  pwm_config motor_pwm_cfg = pwm_get_default_config();
  pwm_config_set_wrap(&motor_pwm_cfg, 255);

  pwm_init(pwm_gpio_to_slice_num(MOTOR1N), &motor_pwm_cfg, true);
  gpio_set_function(MOTOR1N, GPIO_FUNC_PWM);

  pwm_init(pwm_gpio_to_slice_num(MOTOR1P), &motor_pwm_cfg, true);
  gpio_set_function(MOTOR1P, GPIO_FUNC_PWM);

  pwm_init(pwm_gpio_to_slice_num(MOTOR2N), &motor_pwm_cfg, true);
  gpio_set_function(MOTOR2N, GPIO_FUNC_PWM);

  pwm_init(pwm_gpio_to_slice_num(MOTOR2P), &motor_pwm_cfg, true);
  gpio_set_function(MOTOR2P, GPIO_FUNC_PWM);


  for(int i = 0; i < SHAPE_CNT; i++) {
    pt shape;
    shape.x = rand() % 240;
    shape.y = rand() % 240;
    shape.r = (rand() % 10) + 3;
    shape.dx = float(rand() % 255) / 128.0f;
    shape.dy = float(rand() % 255) / 128.0f;
    shape.pen = graphics.create_pen(rand() % 255, rand() % 255, rand() % 255);
    shapes[i] = shape;
  }

}

void screenUpdate() {

  if(!dma_channel_is_busy(dma_channel) && spi_is_writable(spi)) {
    uint8_t r = 0x2C;
    gpio_put(cs, 0);
    gpio_put(dc, 0); // command mode
    spi_write_blocking(spi, &r, 1);
    gpio_put(dc, 1); // data mode
    dma_channel_set_read_addr(dma_channel, buffer, true);
  }

}

void display_loop() {
  switch (getRegister(USB_PORT_STATE))
  {
  case USB_MOUNTED:
    graphics.set_pen(0, 127, 0);
    break;
  case USB_NOT_MOUNTED:
    graphics.set_pen(127, 0, 0);
    break;
  default:
    graphics.set_pen(0, 0, 127);
    break;
  }
  graphics.rectangle(graphics.bounds);

  for(auto &shape : shapes) {
    shape.x += shape.dx;
    shape.y += shape.dy;
    if(shape.x < 0) shape.dx *= -1;
    if(shape.x >= graphics.bounds.w) shape.dx *= -1;
    if(shape.y < 0) shape.dy *= -1;
    if(shape.y >= graphics.bounds.h) shape.dy *= -1;

    graphics.set_pen(shape.pen);
    graphics.circle(Point(shape.x, shape.y), shape.r);
  }

  for(auto &button : buttons) {
    bool next = is_pressed(button.pin);
    if (next != button.state) {
      sendButtonEvent(button.txt, next);
      button.state = next;
    }
  }

  graphics.set_pen(255, 255, 255);
  // graphics.text("Time:",    Point(10, 180), 80); graphics.text(std::to_string(getTime()), Point(100, 180), 130);
  getMessage(scratch);
  graphics.text("Message:", Point(10, 200), 80); graphics.text(std::string(scratch),      Point(100, 200), 130);
  // sprintf(scratch, "%f", get_adc(0));
  // graphics.text("ADC0:", Point(10, 220), 80); graphics.text(std::string(scratch),      Point(100, 220), 130);

  // update screen
  // screen.update();
  // Use our own dma based transfer procedure
  screenUpdate();

}

static StackType_t  display_stack[DISPLAY_STACK_SIZE];
static StaticTask_t display_taskdef;
static TaskHandle_t display_handle;

static StaticTimer_t display_tickdef;
static TimerHandle_t display_ticker;

static void displayTimerCallback(TimerHandle_t h) {
  xTaskNotifyGive(display_handle);
}

static void displayTask(void *pvParameters) {
  uint32_t count;

  printf("Display start, core %d - thread '%s'\n", get_core_num(), pcTaskGetName(xTaskGetCurrentTaskHandle()));
  display_init();
  xTimerStart(display_ticker, pdMS_TO_TICKS(2 * DISPLAY_TICK_MS));

  while(true)
  {
    count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (count) {
        display_loop();
    }
  }
}

void createDisplayHandler() {
  display_handle = xTaskCreateStatic(displayTask,
    "Display",
    DISPLAY_STACK_SIZE,
    NULL,
    DISPLAY_TASK_PRIO,
    display_stack,
    &display_taskdef
  );

  //vTaskCoreAffinitySet(display_handle, 1<<1);
  printf("Display create, core %d\n", get_core_num());

  display_ticker = xTimerCreateStatic("Disp Tick", pdMS_TO_TICKS(DISPLAY_TICK_MS), pdTRUE, (void *) 0, displayTimerCallback, &display_tickdef);
}
