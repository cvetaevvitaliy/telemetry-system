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
        stm32_lib/STM32_USB_Device_Library/Core/Inc
        stm32_lib/STM32_USB_Device_Library/Class/CDC/Inc
        stm32_lib/STM32F4xx_HAL_Driver/Inc
        stm32_lib/STM32F4xx_HAL_Driver/Inc/Legacy
        stm32_lib/CMSIS/Include
        stm32_lib/CMSIS/Device/ST/STM32F4xx/Include )

SET(STM32_HAL_SRC
        stm32_lib/startup/startup_stm32f405xx.s
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c
        stm32_lib/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c

        stm32_lib/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
        stm32_lib/STM32_USB_Device_Library/Core/Src/usbd_core.c
        stm32_lib/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
        stm32_lib/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c)

# FAT_FS Library
SET(FAT_FS_SRC
        src/3rd-partry/FatFs/src/option/syscall.c
        src/3rd-partry/FatFs/src/diskio.c
        src/3rd-partry/FatFs/src/ff.c
        src/3rd-partry/FatFs/src/ff_gen_drv.c )

SET(FAT_FS_INC src/3rd-partry/FatFs/src)

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
# Select build: SERVO_MODULE, TX_MODULE or GROUND_STATION, please add to CMake config -DBUILD=<name>

set(SELECT_BUILD GROUND_STATION TX_MODULE SERVO_MODULE)
SET(BUILD_NAME BUILD_NAME)
foreach(TYPE ${SELECT_BUILD})
#####################################################################################################
## SERVO_MODULE
    if (BUILD STREQUAL SERVO_MODULE)
        message(SERVO_MODULE)
        SET(BUILD_NAME SERVO_MODULE)

        SET(SRC_FILE
                src/servo_module/main.c
                src/servo_module/stm32_hal_config/stm32f4xx_hal_msp.c
                src/servo_module/stm32_hal_config/stm32f4xx_it.c
                src/servo_module/stm32_hal_config/system_stm32f4xx.c
                src/servo_module/stm32_hal_config/usb_device.c
                src/servo_module/stm32_hal_config/usbd_cdc_if.c
                src/servo_module/stm32_hal_config/usbd_conf.c
                src/servo_module/stm32_hal_config/usbd_desc.c)

        SET(INC_FILE
                src/servo_module
                src/servo_module/stm32_hal_config)


#####################################################################################################
## TX_MODULE
    elseif(BUILD STREQUAL TX_MODULE)
        message(TX_MODULE)
        SET(BUILD_NAME TX_MODULE)
        add_definitions(-DBUILD_NAME=\"TX\ MODULE\t\")

        SET(SRC_FILE
                src/3rd-partry/FatFs/src/option/syscall.c
                src/3rd-partry/FatFs/src/diskio.c
                src/3rd-partry/FatFs/src/ff.c
                src/3rd-partry/FatFs/src/ff_gen_drv.c

                ${FreeRTOS_SRC}

                src/tx_module/main.c
                src/tx_module/stm32_hal_config/hardware_init.c
                src/tx_module/device_global_irq.c


                src/tx_module/stm32_hal_config/bsp_driver_sd.c
                src/tx_module/stm32_hal_config/fatfs.c
                src/tx_module/stm32_hal_config/sd_diskio.c
                src/tx_module/stm32_hal_config/stm32f4xx_hal_msp.c
                src/tx_module/stm32_hal_config/stm32f4xx_it.c
                src/tx_module/stm32_hal_config/system_stm32f4xx.c
                src/tx_module/stm32_hal_config/usb_device.c
                src/tx_module/stm32_hal_config/usbd_cdc_if.c
                src/tx_module/stm32_hal_config/usbd_conf.c
                src/tx_module/stm32_hal_config/usbd_desc.c

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
                src/tx_module/services/E19_lora_modem/sx1276-board.c

                src/tx_module/services/range_test/range_test.c
                src/components/minmea/minmea.c
                src/tx_module/services/gps/gps_service.c
                src/components/dfu/dfu.c

                src/tx_module/services/sd_card/sd_card.c
                src/tx_module/services/sd_card/sd_cli_cmd.c)

        SET(INC_FILE
                src/3rd-partry/FatFs/src

                ${FreeRTOS_INC}

                src/3rd-partry/helper-library/tinyprintf
                src/3rd-partry/helper-library/tinystring

                src/components/cli
                src/components/cli/lib
                src/components/cli/ulog

                src/tx_module
                src/tx_module/stm32_hal_config

                src/components/sx1276
                src/tx_module/services/E19_lora_modem
                src/tx_module/services/range_test
                src/tx_module/services

                src/components/minmea
                src/tx_module/services/gps
                src/components/dfu

                src/tx_module/services/sd_card)

#####################################################################################################
## GROUND_STATION
    else() # This is default build target
        message(GROUND_STATION)
        SET(BUILD_NAME GROUND_STATION)
        add_definitions(-DBUILD_NAME=\"GROUND\ STATION\")

        SET(SRC_FILE
                src/3rd-partry/FatFs/src/option/syscall.c
                src/3rd-partry/FatFs/src/diskio.c
                src/3rd-partry/FatFs/src/ff.c
                src/3rd-partry/FatFs/src/ff_gen_drv.c

                ${FreeRTOS_SRC}

                src/ground_station/main.c
                src/ground_station/stm32_hal_config/hardware_init.c
                src/ground_station/device_global_irq.c

                src/ground_station/stm32_hal_config/bsp_driver_sd.c
                src/ground_station/stm32_hal_config/fatfs.c
                src/ground_station/stm32_hal_config/sd_diskio.c
                src/ground_station/stm32_hal_config/stm32f4xx_hal_msp.c
                src/ground_station/stm32_hal_config/stm32f4xx_it.c
                src/ground_station/stm32_hal_config/system_stm32f4xx.c
                src/ground_station/stm32_hal_config/usb_device.c
                src/ground_station/stm32_hal_config/usbd_cdc_if.c
                src/ground_station/stm32_hal_config/usbd_conf.c
                src/ground_station/stm32_hal_config/usbd_desc.c

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
                src/3rd-partry/FatFs/src

                ${FreeRTOS_INC}
                src/ground_station
                src/ground_station/stm32_hal_config

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

add_custom_command(TARGET ${BUILD_NAME}
        COMMENT "For flash via DFU use helper string: dfu-util -a 0 -s 0x08000000:leave -D ${BIN_FILE} -w
Or use: make flash")

add_custom_target(flash
        COMMAND dfu-util -a 0 -s 0x08000000:leave -D ${BIN_FILE} -w)
