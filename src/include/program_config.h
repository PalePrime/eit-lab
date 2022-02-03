#ifndef PROGRAM_CONFIG_H
#define PROGRAM_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define PROGRAM_TASK_CNT 15
#define DEBUG_BUFFER 1024
#define DEBUG_INFO_COUNT 20

#define REPORT_STACK_SIZE configMINIMAL_STACK_SIZE
#define REPORT_TICK_MS 2000
#define REPORT_TASK_PRIO tskIDLE_PRIORITY + 2

#define USB_STACK_SIZE configMINIMAL_STACK_SIZE
#define USB_TASK_PRIO tskIDLE_PRIORITY + 5

#define SPK_CODEC_STACK_SIZE configMINIMAL_STACK_SIZE
#define SPK_CODEC_TASK_PRIO tskIDLE_PRIORITY + 7

#define MIC_CODEC_STACK_SIZE configMINIMAL_STACK_SIZE
#define MIC_CODEC_TASK_PRIO tskIDLE_PRIORITY + 7

#define CH_STACK_SIZE configMINIMAL_STACK_SIZE
#define CH_TASK_PRIO tskIDLE_PRIORITY + 6

#define CDC_STACK_SIZE configMINIMAL_STACK_SIZE
#define CDC_TASK_PRIO tskIDLE_PRIORITY + 3

#define DISPLAY_STACK_SIZE     (3*configMINIMAL_STACK_SIZE)
#define DISPLAY_TASK_PRIO tskIDLE_PRIORITY + 1
#define DISPLAY_TICK_MS  100

// void usbDebugInc(uint32_t index);
// void usbDebugSet(uint32_t index, int64_t value);
// void usbDebugMax(uint32_t index, int64_t value);

#ifdef __cplusplus
}
#endif

#endif /*PROGRAM_CONFIG_H*/
