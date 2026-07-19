cmake_minimum_required(VERSION 3.20)
project(stm32_automation_board C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)

set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(MCU_DEFINES "STM32F407xx" "USE_HAL_DRIVER" "USE_FREERTOS")

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
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_iwdg.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_flash.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_dma.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_tim.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_rcc_ex.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_flash_ex.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_dma_ex.c"
        "${HAL_SRC_DIR}/Src/stm32f4xx_hal_pwr_ex.c"
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
    "src/rs232.c"
    "src/ethernet.c"
    "src/ethernetif.c"
    "src/net_init.c"
    "src/modbus.c"
    "src/modbus_diag.c"
    "src/modbus_master.c"
    "src/modbus_master_rtu.c"
    "src/modbus_tcp.c"
    "src/modbus_tcp_server.c"
    "src/arch/sys_arch.c"
)

set(STARTUP_ASM
    "startup/startup_stm32f407xx.s"
)

set(FREERTOS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS")

if(EXISTS "${FREERTOS_DIR}")
    set(FREERTOS_SOURCES
        "${FREERTOS_DIR}/Source/croutine.c"
        "${FREERTOS_DIR}/Source/event_groups.c"
        "${FREERTOS_DIR}/Source/list.c"
        "${FREERTOS_DIR}/Source/queue.c"
        "${FREERTOS_DIR}/Source/stream_buffer.c"
        "${FREERTOS_DIR}/Source/tasks.c"
        "${FREERTOS_DIR}/Source/timers.c"
        "${FREERTOS_DIR}/Source/portable/GCC/ARM_CM4F/port.c"
        "${FREERTOS_DIR}/Source/portable/MemMang/heap_4.c"
    )
    set(FREERTOS_INCLUDES
        "${FREERTOS_DIR}/Source/include"
        "${FREERTOS_DIR}/Source/portable/GCC/ARM_CM4F"
    )
else()
    set(FREERTOS_SOURCES "")
    set(FREERTOS_INCLUDES "")
    message(WARNING "FreeRTOS not found at ${FREERTOS_DIR}. Build will continue without FreeRTOS.")
endif()

set(LWIP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip")

if(EXISTS "${LWIP_DIR}/src/core/init.c")
    set(LWIP_SOURCES
        "${LWIP_DIR}/src/core/init.c"
        "${LWIP_DIR}/src/core/def.c"
        "${LWIP_DIR}/src/core/dns.c"
        "${LWIP_DIR}/src/core/inet_chksum.c"
        "${LWIP_DIR}/src/core/ip.c"
        "${LWIP_DIR}/src/core/mem.c"
        "${LWIP_DIR}/src/core/memp.c"
        "${LWIP_DIR}/src/core/netif.c"
        "${LWIP_DIR}/src/core/pbuf.c"
        "${LWIP_DIR}/src/core/raw.c"
        "${LWIP_DIR}/src/core/stats.c"
        "${LWIP_DIR}/src/core/sys.c"
        "${LWIP_DIR}/src/core/tcp.c"
        "${LWIP_DIR}/src/core/tcp_in.c"
        "${LWIP_DIR}/src/core/tcp_out.c"
        "${LWIP_DIR}/src/core/timeouts.c"
        "${LWIP_DIR}/src/core/udp.c"
        "${LWIP_DIR}/src/core/ipv4/autoip.c"
        "${LWIP_DIR}/src/core/ipv4/dhcp.c"
        "${LWIP_DIR}/src/core/ipv4/etharp.c"
        "${LWIP_DIR}/src/core/ipv4/icmp.c"
        "${LWIP_DIR}/src/core/ipv4/igmp.c"
        "${LWIP_DIR}/src/core/ipv4/ip4.c"
        "${LWIP_DIR}/src/core/ipv4/ip4_addr.c"
        "${LWIP_DIR}/src/core/ipv4/ip4_frag.c"
        "${LWIP_DIR}/src/api/api_lib.c"
        "${LWIP_DIR}/src/api/api_msg.c"
        "${LWIP_DIR}/src/api/err.c"
        "${LWIP_DIR}/src/api/if_api.c"
        "${LWIP_DIR}/src/api/netbuf.c"
        "${LWIP_DIR}/src/api/netdb.c"
        "${LWIP_DIR}/src/api/netifapi.c"
        "${LWIP_DIR}/src/api/sockets.c"
        "${LWIP_DIR}/src/api/tcpip.c"
        "${LWIP_DIR}/src/netif/ethernet.c"
    )
    set(LWIP_INCLUDES
        "${LWIP_DIR}/src/include"
    )
else()
    set(LWIP_SOURCES "")
    set(LWIP_INCLUDES "")
    message(WARNING "lwIP not found at ${LWIP_DIR}. Run scripts/fetch_lwip.sh or CI download step.")
endif()

add_executable(${PROJECT_NAME}
    ${APP_SOURCES}
    ${HAL_SOURCES}
    ${FREERTOS_SOURCES}
    ${LWIP_SOURCES}
    ${STARTUP_SRC}
    ${STARTUP_ASM}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    "inc"
    "inc/arch"
    "${HAL_SRC_DIR}/Inc"
    "${HAL_SRC_DIR}/Inc/CMSIS_Core"
    ${FREERTOS_INCLUDES}
    ${LWIP_INCLUDES}
)

# lwIP looks for lwipopts.h and arch/cc.h on the include path
target_compile_definitions(${PROJECT_NAME} PRIVATE
    LWIP_DONT_PROVIDE_BYTEORDER_FUNCTIONS
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
