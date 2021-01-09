####################################################################################################
# This file is part of "Telemetry system" project.
#
# "Telemetry system" are free software. You can redistribute
# this software and/or modify this software under the terms of the
# GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# "Telemetry system" are distributed in the hope that they
# will be useful, but WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software.
#
# If not, see <http://www.gnu.org/licenses/>.
####################################################################################################
SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.7)

####################################################################################################
# Software version

set(VERSION_MAJOR 0)
set(VERSION_MINOR 0)
set(VERSION_PATCH 1)

set(VERSION_FW "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")
add_definitions(-DMINOR=\"${VERSION_MAJOR}\")
add_definitions(-DMAJOR=\"${VERSION_MINOR}\")
add_definitions(-DPATCH=\"${VERSION_PATCH}\")

####################################################################################################
# Get git hash ang git branch name (please use definition in project GIT_HASH & GIT_BRANCH)

# Git branch name
execute_process(COMMAND git rev-parse --abbrev-ref HEAD COMMAND cut -c1-10 OUTPUT_VARIABLE GIT_BRANCH)
string(STRIP ${GIT_BRANCH} GIT_BRANCH)
message(STATUS "Git branch - ${GIT_BRANCH}")
add_definitions(-DGIT_BRANCH=\"${GIT_BRANCH}\")

# Git hash
execute_process(COMMAND git rev-parse --short=8 HEAD OUTPUT_VARIABLE GIT_HASH)
string(STRIP ${GIT_HASH} GIT_HASH)
message(STATUS "Git hash - ${GIT_HASH}")
add_definitions(-DGIT_HASH=\"${GIT_HASH}\")

####################################################################################################
# Set compiler todo: need refactor and set specific version of the compiler,
# it is better to make CMake download and install it itself, in _toolchain folder in the project root

SET(CMAKE_C_COMPILER arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)

####################################################################################################
# Set linker script for linker

SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F405RGTx_FLASH.ld)


####################################################################################################
# Float point: Software or Hardware

# Hardware floating point
SET(FPU_FLAGS "-mfloat-abi=hard -mfpu=fpv4-sp-d16")
add_definitions(-DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING)

# Software floating point
#Uncomment for software floating point
#SET(FPU_FLAGS "-mfloat-abi=soft")


####################################################################################################
# Specificity flag definition for ARM Cortex M4 MCU

SET(COMMON_FLAGS
    "-mcpu=cortex-m4 ${FPU_FLAGS} -mthumb -mthumb-interwork -ffunction-sections -fdata-sections \
    -O3 -g3 -fno-common -fmessage-length=0 -specs=nosys.specs -specs=nano.specs")

####################################################################################################
# Specificity flag definition for ARM Cortex M4 MCU

SET(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} -std=c++11")
SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")


PROJECT(Telemetry_System C CXX ASM)
SET(CMAKE_CXX_STANDARD 11)


add_definitions(-D__weak=__attribute__\(\(weak\)\)
        -D__packed=__attribute__\(\(__packed__\)\)
        -DUSE_HAL_DRIVER -DSTM32F405xx -DMCU=\"STM32F405RG\")


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

# FreeRTOS source
SET(FreeRTOS_SRC
        src/3rd-partry/FreeRTOS/portable/GCC/ARM_CM4F/port.c
        src/3rd-partry/FreeRTOS/portable/MemMang/heap_4.c
        src/3rd-partry/FreeRTOS/croutine.c
        src/3rd-partry/FreeRTOS/event_groups.c
        src/3rd-partry/FreeRTOS/list.c
        src/3rd-partry/FreeRTOS/queue.c
        src/3rd-partry/FreeRTOS/tasks.c
        src/3rd-partry/FreeRTOS/timers.c
        src/3rd-partry/FreeRTOS/CMSIS_RTOS/cmsis_os.c)

# FreeRTOS header
SET(FreeRTOS_INC
        src/3rd-partry/FreeRTOS
        src/3rd-partry/FreeRTOS/include
        src/3rd-partry/FreeRTOS/portable/GCC/ARM_CM4F
        src/3rd-partry/FreeRTOS/CMSIS_RTOS)

####################################################################################################
# Select build: SERVO_MODULE, MCU_MODULE or GROUND_STATION, please add to CMake config -DBUILD=<name>

set(SELECT_BUILD GROUND_STATION MCU_MODULE SERVO_MODULE)
SET(BUILD_NAME BUILD_NAME)
foreach(TYPE ${SELECT_BUILD})
#####################################################################################################
## SERVO_MODULE
    if (BUILD STREQUAL SERVO_MODULE)
        message(SERVO_MODULE)
        SET(BUILD_NAME SERVO_MODULE)

        SET(SRC_FILE
                src/servo_module/main.c
                src/servo_module/device/stm32f4xx_hal_msp.c
                src/servo_module/device/stm32f4xx_it.c
                src/servo_module/device/system_stm32f4xx.c
                src/servo_module/device/usb_device.c
                src/servo_module/device/usbd_cdc_if.c
                src/servo_module/device/usbd_conf.c
                src/servo_module/device/usbd_desc.c)

        SET(INC_FILE
                src/servo_module
                src/servo_module/device)


#####################################################################################################
## MCU_MODULE
    elseif(BUILD STREQUAL MCU_MODULE)
        message(MCU_MODULE)
        SET(BUILD_NAME MCU_MODULE)
        add_definitions(-DBUILD_NAME=\"MCU\ MODULE\t\")

        SET(SRC_FILE
                Middlewares/Third_Party/FatFs/src/option/syscall.c
                Middlewares/Third_Party/FatFs/src/diskio.c
                Middlewares/Third_Party/FatFs/src/ff.c
                Middlewares/Third_Party/FatFs/src/ff_gen_drv.c

                ${FreeRTOS_SRC}

                src/mcu_module/main.c
                src/mcu_module/device/hardware_init.c
                src/mcu_module/device_global_irq.c


                src/mcu_module/device/bsp_driver_sd.c
                src/mcu_module/device/fatfs.c
                src/mcu_module/device/sd_diskio.c
                src/mcu_module/device/stm32f4xx_hal_msp.c
                src/mcu_module/device/stm32f4xx_it.c
                src/mcu_module/device/system_stm32f4xx.c
                src/mcu_module/device/usb_device.c
                src/mcu_module/device/usbd_cdc_if.c
                src/mcu_module/device/usbd_conf.c
                src/mcu_module/device/usbd_desc.c

                src/3rd-partry/helper-library/tinyprintf/tinyprintf.c
                src/3rd-partry/helper-library/tinystring/tinystring.c

                src/components/cli/lib/cli_input.c
                src/components/cli/lib/cli_log.c
                src/components/cli/lib/cli_queue.c
                src/components/cli/lib/cli_time.c
                src/components/cli/cli_io.c
                src/components/cli/cli.c
                src/components/cli/ulog/ulog.c

                src/components/sx1276/sx1276.c
                src/mcu_module/services/E19_lora_modem/sx1276-board.c

                src/mcu_module/services/range_test/range_test.c
                src/components/minmea/minmea.c
                src/mcu_module/services/gps/gps_service.c)

        SET(INC_FILE
                Middlewares/Third_Party/FatFs/src

                ${FreeRTOS_INC}

                src/3rd-partry/helper-library/tinyprintf
                src/3rd-partry/helper-library/tinystring

                src/components/cli
                src/components/cli/lib
                src/components/cli/ulog

                src/mcu_module
                src/mcu_module/device

                src/components/sx1276
                src/mcu_module/services/E19_lora_modem
                src/mcu_module/services/range_test
                src/mcu_module/services

                src/components/minmea
                src/mcu_module/services/gps)

#####################################################################################################
## GROUND_STATION
    else() # This is default build target
        message(GROUND_STATION)
        SET(BUILD_NAME GROUND_STATION)
        add_definitions(-DBUILD_NAME=\"GROUND\ STATION\")

        SET(SRC_FILE
                Middlewares/Third_Party/FatFs/src/option/syscall.c
                Middlewares/Third_Party/FatFs/src/diskio.c
                Middlewares/Third_Party/FatFs/src/ff.c
                Middlewares/Third_Party/FatFs/src/ff_gen_drv.c

                ${FreeRTOS_SRC}

                src/ground_station/main.c
                src/ground_station/device/hardware_init.c
                src/ground_station/device_global_irq.c

                src/ground_station/device/bsp_driver_sd.c
                src/ground_station/device/fatfs.c
                src/ground_station/device/sd_diskio.c
                src/ground_station/device/stm32f4xx_hal_msp.c
                src/ground_station/device/stm32f4xx_it.c
                src/ground_station/device/system_stm32f4xx.c
                src/ground_station/device/usb_device.c
                src/ground_station/device/usbd_cdc_if.c
                src/ground_station/device/usbd_conf.c
                src/ground_station/device/usbd_desc.c

                ####################################################################################################
                # Device source codes

                src/3rd-partry/helper-library/tinyprintf/tinyprintf.c
                src/3rd-partry/helper-library/tinystring/tinystring.c

                src/components/cli/lib/cli_input.c
                src/components/cli/lib/cli_log.c
                src/components/cli/lib/cli_queue.c
                src/components/cli/lib/cli_time.c
                src/components/cli/cli_io.c
                src/components/cli/cli.c
                src/components/cli/ulog/ulog.c

                src/components/ssd1306/ssd1306.c
                src/components/ssd1306/Fonts/Font_8x10.c
                src/components/bq25895/bq2589x_charger.c

                src/components/sx1276/sx1276.c
                src/ground_station/sx1276-board.c

                src/ground_station/pcb_test.c
                src/ground_station/range_test.c)
                ####################################################################################################

        SET(INC_FILE
                Middlewares/Third_Party/FatFs/src

                ${FreeRTOS_INC}
                src/ground_station
                src/ground_station/device

                ####################################################################################################
                # Device include directories

                src/3rd-partry/helper-library/tinyprintf
                src/3rd-partry/helper-library/tinystring

                src/components/cli
                src/components/cli/lib
                src/components/cli/ulog

                src/components/ssd1306/
                src/components/ssd1306/Fonts
                src/components/ssd1306/Image
                src/components/bq25895

                src/components/sx1276)
    endif()
endforeach()
#####################################################################################################

#_WHERE_BUILD
if (_WHERE_BUILD STREQUAL Cloud)
    add_definitions(-D_WHERE_BUILD=\"Cloud\ build\")
else()
    add_definitions(-D_WHERE_BUILD=\"Local\ build\")
    add_definitions(-DDEBUG=1)
    add_definitions(-DDEBUG_TIMESTAMP=1)
endif()

add_executable (${BUILD_NAME} ${STM32_HAL_SRC} ${SRC_FILE} ${LINKER_SCRIPT})
target_include_directories(${BUILD_NAME} PRIVATE ${STM32_HAL_INC} ${INC_FILE})


SET(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-u,_printf_float,-Map=${PROJECT_BINARY_DIR}/${BUILD_NAME}-V${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}-${GIT_HASH}.map ${FPU_FLAGS}")

SET(HEX_FILE ${PROJECT_BINARY_DIR}/${BUILD_NAME}-V${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}-${GIT_HASH}.hex)
SET(BIN_FILE ${PROJECT_BINARY_DIR}/${BUILD_NAME}-V${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}-${GIT_HASH}.bin)

add_custom_command(TARGET ${BUILD_NAME} POST_BUILD
        COMMAND arm-none-eabi-size --format=berkeley ${BUILD_NAME}
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${BUILD_NAME}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${BUILD_NAME}> ${BIN_FILE}
        COMMENT "Building done, output files:
${HEX_FILE}
${BIN_FILE}
File details, format berkeley:")
#####################################################################################################