SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.7)

####################################################################################################
set(VERSION_MAJOR 0)
set(VERSION_MINOR 0)
set(VERSION_PATCH 1)

set(VERSION_FW "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")
add_definitions(-DMINOR=\"${VERSION_MAJOR}\")
add_definitions(-DMAJOR=\"${VERSION_MINOR}\")
add_definitions(-DPATCH=\"${VERSION_PATCH}\")
####################################################################################################

SET(CMAKE_C_COMPILER arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)

SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F405RGTx_FLASH.ld)

#Uncomment for hardware floating point
SET(FPU_FLAGS "-mfloat-abi=hard -mfpu=fpv4-sp-d16")
add_definitions(-DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING)

#Uncomment for software floating point
#SET(FPU_FLAGS "-mfloat-abi=soft")

SET(COMMON_FLAGS
    "-mcpu=cortex-m4 ${FPU_FLAGS} -mthumb -mthumb-interwork -ffunction-sections -fdata-sections \
    -g -fno-common -fmessage-length=0 -specs=nosys.specs -specs=nano.specs")

SET(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} -std=c++11")
SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")



PROJECT(MCU_Module C CXX ASM)
SET(CMAKE_CXX_STANDARD 11)


add_definitions(-D__weak=__attribute__\(\(weak\)\)
        -D__packed=__attribute__\(\(__packed__\)\)
        -DUSE_HAL_DRIVER -DSTM32F405xx)

####################################################################################################
# STM HAL Library

SET(STM32_HAL_INC
        Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Include
        Drivers/CMSIS/Device/ST/STM32F4xx/Include )

SET(STM32_HAL_SRC
        startup/startup_stm32f405xx.s
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c

        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
        Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
        Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
        Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c)

# FAT_FS Library
SET(FAT_FS_SRC
        Middlewares/Third_Party/FatFs/src/option/syscall.c
        Middlewares/Third_Party/FatFs/src/diskio.c
        Middlewares/Third_Party/FatFs/src/ff.c
        Middlewares/Third_Party/FatFs/src/ff_gen_drv.c )

SET(FAT_FS_INC Middlewares/Third_Party/FatFs/src)

####################################################################################################
# GROUND_STATION

SET(GROUND_STATION_SRC
        src/ground_station/main.c
        src/ground_station/device/bsp_driver_sd.c
        src/ground_station/device/fatfs.c
        src/ground_station/device/sd_diskio.c
        src/ground_station/device/stm32f4xx_hal_msp.c
        src/ground_station/device/stm32f4xx_it.c
        src/ground_station/device/system_stm32f4xx.c
        src/ground_station/device/usb_device.c
        src/ground_station/device/usbd_cdc_if.c
        src/ground_station/device/usbd_conf.c
        src/ground_station/device/usbd_desc.c)

SET(GROUND_STATION_INC
        src/ground_station
        src/ground_station/device)

SET(Ground_Station GROUND_STATION)

add_executable (${Ground_Station} ${STM32_HAL_SRC} ${FAT_FS_SRC} ${GROUND_STATION_SRC} ${LINKER_SCRIPT})
target_include_directories(${Ground_Station} PRIVATE ${STM32_HAL_INC} ${FAT_FS_INC} ${GROUND_STATION_INC})


SET(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${Ground_Station}.map ${FPU_FLAGS}")

SET(HEX_FILE_GROUND_STATION ${PROJECT_BINARY_DIR}/${Ground_Station}.hex)
SET(BIN_FILE_GROUND_STATION ${PROJECT_BINARY_DIR}/${Ground_Station}.bin)

add_custom_command(TARGET ${Ground_Station} POST_BUILD
        COMMAND ${SIZE} --format=berkeley ${Ground_Station}
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${Ground_Station}> ${HEX_FILE_GROUND_STATION}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${Ground_Station}> ${BIN_FILE_GROUND_STATION}
        COMMENT "Building ${HEX_FILE_GROUND_STATION}
Building ${BIN_FILE_GROUND_STATION}")

####################################################################################################
# MCU_MODULE

SET(MCU_MODULE MCU_MODULE)

SET(MCU_MODULE_SRC
        src/mcu_module/main.c
        src/mcu_module/device/bsp_driver_sd.c
        src/mcu_module/device/fatfs.c
        src/mcu_module/device/sd_diskio.c
        src/mcu_module/device/stm32f4xx_hal_msp.c
        src/mcu_module/device/stm32f4xx_it.c
        src/mcu_module/device/system_stm32f4xx.c
        src/mcu_module/device/usb_device.c
        src/mcu_module/device/usbd_cdc_if.c
        src/mcu_module/device/usbd_conf.c
        src/mcu_module/device/usbd_desc.c)

SET(MCU_MODULE_INC
        src/mcu_module
        src/mcu_module/device)

add_executable(${MCU_MODULE} ${STM32_HAL_SRC} ${FAT_FS_SRC} ${MCU_MODULE_SRC} ${LINKER_SCRIPT})
target_include_directories(${MCU_MODULE} PRIVATE ${STM32_HAL_INC} ${FAT_FS_INC} ${MCU_MODULE_INC})


SET(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${MCU_MODULE}.map ${FPU_FLAGS}")

SET(HEX_FILE_MCU_MODULE ${PROJECT_BINARY_DIR}/${MCU_MODULE}.hex)
SET(BIN_FILE_MCU_MODULE ${PROJECT_BINARY_DIR}/${MCU_MODULE}.bin)

add_custom_command(TARGET ${MCU_MODULE} POST_BUILD
        COMMAND ${SIZE} --format=berkeley ${MCU_MODULE}
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${MCU_MODULE}> ${HEX_FILE_MCU_MODULE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${MCU_MODULE}> ${BIN_FILE_MCU_MODULE}
        COMMENT "Building ${HEX_FILE_MCU_MODULE}
Building ${BIN_FILE_MCU_MODULE}")

####################################################################################################
# SERVO_MODULE

SET(SERVO_MODULE SERVO_MODULE)

SET(SERVO_MODULE_SRC
        src/servo_module/main.c
        src/servo_module/device/stm32f4xx_hal_msp.c
        src/servo_module/device/stm32f4xx_it.c
        src/servo_module/device/system_stm32f4xx.c
        src/servo_module/device/usb_device.c
        src/servo_module/device/usbd_cdc_if.c
        src/servo_module/device/usbd_conf.c
        src/servo_module/device/usbd_desc.c)

SET(SERVO_MODULE_INC
        src/servo_module
        src/servo_module/device)

add_executable(${SERVO_MODULE} ${STM32_HAL_SRC} ${SERVO_MODULE_SRC} ${LINKER_SCRIPT})
target_include_directories(${SERVO_MODULE} PRIVATE ${STM32_HAL_INC} ${SERVO_MODULE_INC})


SET(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${SERVO_MODULE}.map ${FPU_FLAGS}")

SET(HEX_FILE_SERVO_MODULE ${PROJECT_BINARY_DIR}/${SERVO_MODULE}.hex)
SET(BIN_FILE_SERVO_MODULE ${PROJECT_BINARY_DIR}/${SERVO_MODULE}.bin)

add_custom_command(TARGET ${SERVO_MODULE} POST_BUILD
        COMMAND ${SIZE} --format=berkeley ${SERVO_MODULE}
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${SERVO_MODULE}> ${HEX_FILE_SERVO_MODULE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${SERVO_MODULE}> ${BIN_FILE_SERVO_MODULE}
        COMMENT "Building ${HEX_FILE_SERVO_MODULE}
Building ${BIN_FILE_SERVO_MODULE}")
