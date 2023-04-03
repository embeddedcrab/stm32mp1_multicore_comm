
# FreeRTOS Lib CMake

cmake_minimum_required(VERSION 3.15)


set(FreeRTOS_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/CMSIS_RTOS/cmsis_os.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/CMSIS_RTOS_V2/cmsis_os2.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/portable/Common/mpu_wrappers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/portable/GCC/ARM_CM4F/port.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/portable/MemMang/heap_4.c
    #${CMAKE_CURRENT_SOURCE_DIR}/Source/croutine.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/event_groups.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/stream_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/timers.c
)

set_source_files_properties(${FreeRTOS_Src}
    PROPERTIES COMPILE_FLAGS ${STM32_DEFINES}
)

add_library(freertos STATIC ${FreeRTOS_Src})

target_include_directories(freertos
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Source/CMSIS_RTOS
        ${CMAKE_CURRENT_SOURCE_DIR}/Source/CMSIS_RTOS_V2
        ${CMAKE_CURRENT_SOURCE_DIR}/Source/include
        ${CMAKE_CURRENT_SOURCE_DIR}/Source/portable/GCC/ARM_CM4F
        ${PROJECT_SOURCE_DIR}/inc
        ${PROJECT_SOURCE_DIR}/drivers/CMSIS/Core/Include
    PUBLIC
        $<BUILD_INTERFACE:
        ${CMAKE_CURRENT_SOURCE_DIR}/Source/CMSIS_RTOS
        ${CMAKE_CURRENT_SOURCE_DIR}/Source/CMSIS_RTOS_V2
        ${CMAKE_CURRENT_SOURCE_DIR}/Source/include
        ${CMAKE_CURRENT_SOURCE_DIR}/Source/portable/GCC/ARM_CM4F/>
        # where external projects will look for the library's public headers
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

set_target_properties(freertos PROPERTIES LINKER_LANGUAGE C)


# Create package of drivers
install(TARGETS freertos DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Install headers
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
  PATTERN *.h
  PATTERN *.inl
  PATTERN *.hpp
  PATTERN *.ih
)
