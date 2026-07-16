cmake_minimum_required(VERSION 3.20)
project(stm32_automation_board C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)

set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(MCU_DEFINES "STM32F407xx" "USE_HAL_DRIVER")

set(CMAKE_C_FLAGS "${MCU_FLAGS} -Wall -Wextra -Wno-unused-parameter -Os -ffunction-sections -fdata-sections")
set(CMAKE_C_FLAGS_DEBUG   "-g -O0")
set(CMAKE_C_FLAGS_RELEASE "-Os -DNDEBUG")

set(CMAKE_ASM_FLAGS "${MCU_FLAGS}")


foreach(def ${MCU_DEFINES})
    add_compile_definitions(${def})
endforeach()

set(HAL_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4_hal")

if(EXISTS "${HAL_SRC_DIR}")
    set(HAL_SOURCES
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_cortex.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_rcc.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_gpio.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_uart.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_adc.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_dac.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_eth.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_pwr.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_flash.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_dma.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_tim.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_rcc_ex.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_flash_ex.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_gpio.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_dma_ex.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_pwr_ex.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_cortex.c"
    )
    set(STARTUP_SRC
        "${HAL_SRC_DIR}/Src/system_stm32f4xx.c"
    )
else()
    set(HAL_SOURCES "")
    set(STARTUP_SRC "")
    message(WARNING "ST HAL library not found at ${HAL_SRC_DIR}. Build will continue without HAL sources.")
endif()

set(APP_SOURCES
    "src/main.c"
    "src/digital_io.c"
    "src/analog_io.c"
    "src/relay.c"
    "src/rs485.c"
    "src/ethernet.c"
    "src/modbus.c"
    "src/modbus_master.c"
    "src/modbus_master_rtu.c"
    "src/modbus_tcp.c"
)

set(STARTUP_ASM
    "startup/startup_stm32f407xx.s"
)

add_executable(${PROJECT_NAME}
    ${APP_SOURCES}
    ${HAL_SOURCES}
    ${STARTUP_SRC}
    ${STARTUP_ASM}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    "inc"
    "${HAL_SRC_DIR}/Inc"
    "${HAL_SRC_DIR}/Inc/CMSIS_Core"
)

target_link_options(${PROJECT_NAME} PRIVATE
    "SHELL:-T ${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F407VGTx_FLASH.ld"
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    "SHELL:-Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map"
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -nostartfiles
    --specs=nosys.specs
    --specs=nano.specs
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Generating hex and bin files"
)
