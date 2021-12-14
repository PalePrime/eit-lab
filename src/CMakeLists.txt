add_executable(pa_audio
        main.c
        # sdkRTOSfix.c
        # msc_disk.c
        FreeRTOSHooks.c
        usb_descriptors.c
        display_handling.cpp
        program_state.c
        usb_handling.c
        cdc_handling.c
        report_handling.c
        uac2_handling.c
        task_tracing.c
        )

# add_resource(pa_audio fox.tga)

target_include_directories(pa_audio PRIVATE
        include/
        ${FREERTOS_KERNEL_PATH}/include
        ${FREERTOS_KERNEL_PATH}/${FREERTOS_KERNEL_RP2040_RELATIVE_PATH}/include)

#add_library(my_stuff INTERFACE)

#target_link_libraries(my_stuff INTERFACE
#        FreeRTOS-Kernel
#        FreeRTOS-Kernel-Static
#        tinyusb_device
#        tinyusb_board
#        pico_stdlib
#        pico_multicore)

target_link_libraries(pa_audio
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Static
        tinyusb_device
        tinyusb_board
        pico_stdlib
        pico_multicore
        hardware_spi
        pico_explorer
        hardware_structs)

target_compile_definitions(pa_audio PRIVATE

        AUDIO_FREQ_MAX=48000

        # ours are zero based, so say so
        PICO_USBDEV_USE_ZERO_BASED_INTERFACES=1

        # need large descriptor
        PICO_USBDEV_MAX_DESCRIPTOR_SIZE=256


        PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE=1
        PICO_USBDEV_ENABLE_DEBUG_TRACE

        PICO_AUDIO_I2S_MONO_OUTPUT=0
        PICO_AUDIO_I2S_MONO_INPUT=0

        # PICO_STDOUT_MUTEX=0
)

pico_add_extra_outputs(pa_audio)
