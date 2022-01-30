#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "program_config.h"
#include "task_tracing.h"
#include "program_state.h"

static uint32_t taskCount = 0;

typedef struct {
  TaskInfoRecord info;
  uint64_t lastStart;
} TaskRecord;

static TaskRecord taskRecords[PROGRAM_TASK_CNT];
static uint64_t firstTime;
static uint64_t deltaTime;

void traceInit() {
  firstTime = to_us_since_boot(get_absolute_time());
  deltaTime = firstTime;
}

uint32_t traceGetTaskCount() {
  return taskCount;
}

TaskInfoRecord traceGetInfo(uint32_t index) {
  assert(index<=taskCount);
  return taskRecords[index].info;
}

void traceDeltaTick() {
  deltaTime = to_us_since_boot(get_absolute_time());
  for (uint32_t index = 0; index<taskCount; index++) {
    taskRecords[index].info.deltaTime = 0;
    taskRecords[index].info.core0Start = 0;
    taskRecords[index].info.core1Start = 0;
  }
}

uint32_t traceDeltaTime() {
  uint64_t now = to_us_since_boot(get_absolute_time());
  return now - deltaTime;
}

uint64_t traceTotalTime() {
  uint64_t now = to_us_since_boot(get_absolute_time());
  return now - firstTime;
}

void traceCreateTask(void * _handle) {
  assert(taskCount<PROGRAM_TASK_CNT);
  taskRecords[taskCount].info.name = pcTaskGetName(_handle);
  vTaskSetApplicationTaskTag(_handle, (void *) taskCount);
  taskCount++;
}

void traceSwitchIn() {
  uint32_t index = (uint32_t) xTaskGetApplicationTaskTag(NULL);
  // assert(index<taskCount);
  uint64_t now = to_us_since_boot(get_absolute_time());
  if (get_core_num()) {
    taskRecords[index].info.core1Start++;
  } else {
    taskRecords[index].info.core0Start++;
  }
  taskRecords[index].lastStart = now;
  taskRecords[index].info.running = true;
};

void traceSwitchOut() {
  uint32_t index = (uint32_t) xTaskGetApplicationTaskTag(NULL);
  // assert(index<taskCount);
  uint64_t now = to_us_since_boot(get_absolute_time());
  uint64_t timeSlice = now - taskRecords[index].lastStart;
  taskRecords[index].info.accumulatedTime += timeSlice;
  taskRecords[index].info.deltaTime += timeSlice;
  taskRecords[index].info.running = false;
};

