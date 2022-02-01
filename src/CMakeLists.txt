add_executable(pa_audio
        main.c
        FreeRTOSHooks.c
        usb_descriptors.c
        display_handling.cpp
        program_state.c
        usb_handling.c
        cdc_handling.c
        report_handling.c
        uac2_handling.c
        task_tracing.c
        channel_controller.c
        mic_channel.c
        spk_channel.c
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

        # This is critical for FreeRTOS, TinyUSB and Pico SDK to play nicely
        # The FreeRTOS port sets priority levels for timer and SVC interrupts to 255 (lowest level)
        # The TinyUSB port installs the USB interrupt handler at Pico SDK default priority
        # The Pico SDK sets default interrupt priority to 128 (middle level) unless adjusted as below
        # Without this setting the USB interrupt handler violates FreeRTOS assumptions by preempting its handlers
        # effectively creating a spurious real time bug that really haunted me...
        PICO_DEFAULT_IRQ_PRIORITY=255

)

pico_add_extra_outputs(pa_audio)
