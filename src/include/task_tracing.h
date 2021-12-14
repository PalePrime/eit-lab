#ifndef TASK_TRACING_H
#define TASK_TRACING_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t running;
  char * name;
  uint32_t deltaTime;
  uint64_t accumulatedTime;
} TaskInfoRecord;

void traceInit();

uint32_t traceGetTaskCount();
TaskInfoRecord traceGetInfo(uint32_t index);
void traceDeltaTick();
uint32_t traceDeltaTime();
uint64_t traceTotalTime();

// The argument is really a TaskHandle_t but include file order makes type unavailable here
void traceCreateTask(void * tcb);
void traceSwitchIn();
void traceSwitchOut();

#define traceTASK_CREATE(xTask) traceCreateTask(xTask)
#define traceTASK_SWITCHED_IN() traceSwitchIn()
#define traceTASK_SWITCHED_OUT() traceSwitchOut()

#ifdef __cplusplus
}
#endif

#endif /*TASK_TRACING_H*/
