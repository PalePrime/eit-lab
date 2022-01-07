cmake_minimum_required(VERSION 3.12)

# Pull in PICO SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)
include(pimoroni_pico_import.cmake)

include(external/FreeRTOS/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)
message("FREERTOS_KERNEL_PATH is ${FREERTOS_KERNEL_PATH}")

project(pico_pa C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(external/tinyusb/tinyusb/hw/bsp/rp2040/family.cmake)
message("PICO_TINYUSB_PATH is ${PICO_TINYUSB_PATH}")

# Initialize the Pico SDK
pico_sdk_init()

add_subdirectory(${PIMORONI_PICO_PATH}/common pimoroni_common)
add_subdirectory(${PIMORONI_PICO_PATH}/drivers pimoroni_drivers)
add_subdirectory(${PIMORONI_PICO_PATH}/libraries pimoroni_libraries)

function(add_resource target file)
  get_filename_component(NAME ${ARGV1} NAME_WE)
  set(FILENAME ${ARGV1})

  add_custom_command(
    OUTPUT ${NAME}.o

    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}
            ${CMAKE_CURRENT_BINARY_DIR}

    COMMAND arm-none-eabi-ld -r -b binary -o ${NAME}.o ${FILENAME}
    DEPENDS ${FILENAME}
  )

  target_sources(${ARGV0} PRIVATE ${NAME}.o)
endfunction(add_resource)

add_subdirectory(src)