
cmake_minimum_required(VERSION 3.15)

project(M4A7CommApp VERSION 1.0.0 LANGUAGES CXX C DESCRIPTION "M4 A7 General Comm")


include(GNUInstallDirs)

option(USE_SEMIHOSTING "Use semi-hosting" OFF)
option(USE_STTERM "Use st-term" OFF)
option(USE_DBGUART "Use debug UART" OFF)
option(USE_GDB "Enable GDB build for debugging" OFF)
option(USE_OVERCLOCKING "Enable overclocking to 128MHz" OFF)


# CMAKE toolchain for the gcc arm-none-eabi
#
set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_VERSION   1)
set(CMAKE_SYSTEM_PROCESSOR arm-none-eabi)

# set arm-none-eabi toolchain paths
if (NOT TOOLCHAIN_DIR)
set(TOOLCHAIN_DIR /home/hemant/toolchain/gcc-arm-none-eabi)
endif()

set(TOOL_CHAIN_PREFIX arm-none-eabi)
set(TOOLCHAIN_BIN_DIR ${TOOLCHAIN_DIR}/bin)
set(TOOLCHAIN_LIB_DIR ${TOOLCHAIN_DIR}/lib)

# which compilers to use for C and C++
#
SET(CMAKE_AR               ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-gcc-ar${EXE})
SET(CMAKE_RANLIB           ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-gcc-ranlib${EXE})
SET(CMAKE_LD               ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-ld${EXE})
set(CMAKE_C_COMPILER       ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-gcc${EXE})
set(CMAKE_CXX_COMPILER     ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-g++${EXE})
set(CMAKE_ASM_COMPILER     ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-as${EXE})
set(CMAKE_OBJCOPY     	   ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-objcopy${EXE} CACHE INTERNAL "objcopy command")
set(CMAKE_OBJDUMP     	   ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-objdump${EXE} CACHE INTERNAL "objdump command")
set(CMAKE_GDB              ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-gdb${EXE})
set(CMAKE_SIZE             ${TOOLCHAIN_BIN_DIR}/${TOOL_CHAIN_PREFIX}-size${EXE})

set(CMAKE_C_ARCHIVE_CREATE "${CMAKE_AR} qc <TARGET> <OBJECTS>")
set(CMAKE_C_ARCHIVE_FINISH "<CMAKE_RANLIB> <TARGET>")

set(COMPILER_OPTIONS "-mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -ffast-math -mlittle-endian -mthumb-interwork ")

SET(CMAKE_C_FLAGS "${COMPILER_OPTIONS} -lm -lc -lnosys --specs=nosys.specs " CACHE INTERNAL "c compiler flags")
SET(CMAKE_CXX_FLAGS "${COMPILER_OPTIONS} -lm -lc -lnosys --specs=nosys.specs " CACHE INTERNAL "cxx compiler flags")
SET(CMAKE_ASM_FLAGS "${COMPILER_OPTIONS}" CACHE INTERNAL "asm compiler flags")
SET(CMAKE_EXE_LINKER_FLAGS "${COMPILER_OPTIONS} -Wl,-Map=linker.map -Wl,-cref -Wl,--gc-sections" CACHE INTERNAL "exe link flags")
SET(CMAKE_ASM_COMPILE_OBJECT "<CMAKE_ASM_COMPILER> <FLAGS> -o <OBJECT> <SOURCE>")


# Set STM32 SoC specific variables
set(STM32_DEFINES " \
    -DIVECT_TAB_OFFSET=0x0 \
    -DUSE_HAL_DRIVER \
    -DCORE_CM4 \
    -DSTM32MP157Cxx \
    -DMETAL_INTERNAL \
    -DMETAL_MAX_DEVICE_REGIONS=2 \
    -DVIRTIO_SLAVE_ONLY \
    -D__LOG_UART_IO_ \
    -DNO_ATOMIC_64_SUPPORT \
    ")


if (USE_OVERCLOCKING)
    set(STM32_DEFINES "${STM32_DEFINES} -DUSE_OVERCLOCKING")
endif()

# set compiler optimisations
set(COMPILER_OPTIMISATION "-g -O1 -ffunction-sections -fdata-sections -fno-builtin")

# Only build for arm-none-eabi
if (NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "arm-none-eabi")
    message(FATAL_ERROR "Invalid CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
endif()



# enable asm for stm startup.s file
enable_language(ASM)

# add startup and linker file
set(STARTUP_ASM_FILE "${CMAKE_SOURCE_DIR}/sw4stm32/startup_stm32mp15xx.s")
set_property(SOURCE ${STARTUP_ASM_FILE} PROPERTY LANGUAGE ASM)
set(LINKER_FILE "${CMAKE_SOURCE_DIR}/sw4stm32/FreeRTOS/stm32mp15xx_m4.ld")

# Custom STM32 options for assembler and gcc
set(STM32_ASM_OPTIONS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
                        -mthumb -mthumb-interwork")

set(STM32_COMPILER_OPTIONS "${STM32_ASM_OPTIONS} \
                        ${COMPILER_OPTIMISATION} \
                        -Wall -Wl,-Map=output.map -Wl,--gc-sections -lm -lc ")

# Set project options
SET(CMAKE_C_FLAGS "${STM32_COMPILER_OPTIONS} ${COMPILER_OPTIMISATION} -std=c11" CACHE INTERNAL "c compiler flags")
SET(CMAKE_CXX_FLAGS "${STM32_COMPILER_OPTIONS} ${COMPILER_OPTIMISATION} \
                    -std=c++14 -fno-exceptions -fno-unwind-tables -fno-rtti \
                    -mabi=aapcs -fno-unroll-loops -ftree-vectorize "
                    CACHE INTERNAL "cxx compiler flags")
SET(CMAKE_ASM_FLAGS "${STM32_ASM_OPTIONS}" CACHE INTERNAL "asm compiler flags")

set(EXTRA_LINKER_FLAGS "-lm -lc --specs=nano.specs -u _printf_float")
if (USE_SEMIHOSTING)
    set(EXTRA_LINKER_FLAGS "${EXTRA_LINKER_FLAGS} --specs=nosys.specs --specs=rdimon.specs -lrdimon")
endif()
SET(CMAKE_EXE_LINKER_FLAGS "${STM32_COMPILER_OPTIONS} -Wl,-Map=linker.map -Wl,-cref " CACHE INTERNAL "exe link flags")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_LINKER_FLAGS} -T${LINKER_FILE}")

message(STATUS "System Processor : ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS
    "BUILD FLAGS:\n"
    "   STM32_DEFINES   : ${STM32_DEFINES}\n"
    "   c flags         : ${CMAKE_C_FLAGS}\n"
    "   c++ flags       : ${CMAKE_CXX_FLAGS}\n"
    "   asm flags       : ${CMAKE_ASM_FLAGS}\n"
    "   ld flags        : ${CMAKE_EXE_LINKER_FLAGS}\n"
    "   optimizations   : ${COMPILER_OPTIMISATION}\n"
    "   Overclocking    : ${USE_OVERCLOCKING}\n"
)


set(INSTALL_BIN_DIR /usr/local/)
set(LINKER_SCRIPTS_DIR ${CMAKE_SOURCE_DIR}/config/LinkerScripts)


add_subdirectory(drivers)
add_subdirectory(middlewares/Third_Party/FreeRTOS)
add_subdirectory(middlewares/Third_Party/OpenAMP)


include_directories(
    ${CMAKE_SOURCE_DIR}/inc
)

set(${PROJECT_NAME}_Src
    ${CMAKE_SOURCE_DIR}/src/app_freertos.c
    ${CMAKE_SOURCE_DIR}/src/copro_sync.c
    ${CMAKE_SOURCE_DIR}/src/lock_resource.c
    ${CMAKE_SOURCE_DIR}/src/mbox_ipcc.c
    ${CMAKE_SOURCE_DIR}/src/openamp_log.c
    ${CMAKE_SOURCE_DIR}/src/openamp.c
    ${CMAKE_SOURCE_DIR}/src/rsc_table.c
    ${CMAKE_SOURCE_DIR}/src/stm32mp1xx_hal_msp.c
    ${CMAKE_SOURCE_DIR}/src/stm32mp1xx_hal_timebase_tim.c
    ${CMAKE_SOURCE_DIR}/src/stm32mp1xx_it.c
    ${CMAKE_SOURCE_DIR}/src/syscalls.c
    ${CMAKE_SOURCE_DIR}/src/sysmem.c
    ${CMAKE_SOURCE_DIR}/src/system_stm32mp1xx.c
    ${CMAKE_SOURCE_DIR}/src/main.c
)

set_source_files_properties(${${PROJECT_NAME}_Src}
    PROPERTIES COMPILE_FLAGS ${STM32_DEFINES} ${CMAKE_C_FLAGS} ${CMAKE_ASM_FLAGS}
)

add_executable(${PROJECT_NAME}.elf ${${PROJECT_NAME}_Src} ${STARTUP_ASM_FILE})

set_target_properties(${PROJECT_NAME}.elf PROPERTIES LINKER_LANGUAGE C LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS})

target_link_libraries(${PROJECT_NAME}.elf
    PRIVATE
        bsp_drivers
        freertos
        open-amp
    PUBLIC
)


add_custom_target ( ${PROJECT_NAME}.hex ALL
    DEPENDS ${PROJECT_NAME}.elf
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Generating ${PROJECT_NAME}.hex"
)
add_custom_target ( ${PROJECT_NAME}.bin ALL
    DEPENDS ${PROJECT_NAME}.elf
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMENT "Generating ${PROJECT_NAME}.bin"
)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} --format=berkeley "${PROJECT_NAME}.elf")


install(TARGETS ${PROJECT_NAME}.elf DESTINATION ${INSTALL_BIN_DIR})
