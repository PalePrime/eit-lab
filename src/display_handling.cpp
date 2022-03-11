#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <string.h>
#include <math.h>
#include <cstdlib>

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

char scratch[S_BUFFER_SIZE+1];

void display_init() {

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

  for(int i = 0; i < SHAPE_CNT; i++) {
    pt shape;
    shape.x = rand() % 240;
    shape.y = rand() % 240;
    shape.r = (rand() % 10) + 3;
    shape.dx = float(rand() % 255) / 128.0f;
    shape.dy = float(rand() % 255) / 128.0f;
    shape.pen = graphics.create_pen((rand() % 127) + 64, (rand() % 127) + 64, (rand() % 127) + 64);
    shapes[i] = shape;
  }

}

void screenUpdate() {
  bool holdOff = getRegister(MIC_AUDIO_STATE) != AUDIO_IDLE;
  if (!holdOff && !dma_channel_is_busy(dma_channel) && spi_is_writable(spi)) {
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

  graphics.set_pen(255, 255, 255);

  menu_state_t state = getMenuState();

  if (state == MENU_ROOT_STATE) {
    graphics.text("Menu",   Point(5,  60),  40);
    getMessage(scratch);
    graphics.text(std::string(scratch), Point(20, 200), 200);
  } else {
    graphics.text("Ent",    Point(  5,  62),  40);
    graphics.text("Bck",    Point(  5, 182),  40);
    graphics.text("Up",     Point(200,  62),  40);
    graphics.text("Dwn",    Point(200, 182),  40);

    menu_item_t *menu  = getMenuItem();
    if (state == MENU_NAV_STATE) {
      graphics.text(std::string(menu->parent->text), Point(65,  40), 160);
      menu_item_t *first = menu->parent->sub_menu;
      menu_item_t *item  = first;
      uint32_t count = 0;
      while (item != NULL) {
        if (item == menu) {
          graphics.text(">",    Point(65,  60 + count * 20), 10);
        }
        graphics.text(std::string(item->text), Point(85,  60 + count * 20), 160);
        count++;
        item = item->next;
        if (item == first) {
          item = NULL;
        }
      }
    } else {
      graphics.text(std::string(menu->text), Point(65,  40), 160);
      menu_item_info_t *info = menu->info;
      uint32_t value = getRegister(menu->reg);
      for (uint32_t i = 0; i < getRegister(MENU_COUNT); i++) {
        if (i == getRegister(MENU_CURRENT)) {
          graphics.text("*",    Point(65,  60 + i * 20), 10);
        } else if (i == getRegister(MENU_SELECT)) {
          graphics.text(">",    Point(65,  60 + i * 20), 10);
        }
        graphics.text(std::string(info->choice_info.choices[i].label), Point(85,  60 + i * 20), 160);
      }
    }
  }

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

  display_ticker = xTimerCreateStatic("Disp Tick", pdMS_TO_TICKS(DISPLAY_TICK_MS), pdTRUE, (void *) 0, displayTimerCallback, &display_tickdef);
}
