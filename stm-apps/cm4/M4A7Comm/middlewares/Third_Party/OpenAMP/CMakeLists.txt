
# OpemAMP Lib CMake

cmake_minimum_required(VERSION 3.15)


set(OpenAMP_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/system/freertos/condition.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/system/freertos/device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/system/freertos/init.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/system/freertos/io.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/system/freertos/irq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/system/freertos/shmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/system/freertos/time.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/system/freertos/stm32mp157c-dk2/sys.c

    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/init.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/io.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/irq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/shmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/softirq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/version.c

    ${CMAKE_CURRENT_SOURCE_DIR}/open-amp/lib/proxy/rpmsg_retarget.c
    ${CMAKE_CURRENT_SOURCE_DIR}/open-amp/lib/remoteproc/elf_loader.c
    ${CMAKE_CURRENT_SOURCE_DIR}/open-amp/lib/remoteproc/remoteproc_virtio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/open-amp/lib/remoteproc/remoteproc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/open-amp/lib/remoteproc/rsc_table_parser.c
    ${CMAKE_CURRENT_SOURCE_DIR}/open-amp/lib/rpmsg/rpmsg_virtio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/open-amp/lib/rpmsg/rpmsg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/open-amp/lib/virtio/virtio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/open-amp/lib/virtio/virtqueue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/virtual_driver/virt_uart.c
)

add_library(open-amp STATIC ${OpenAMP_Src} )

target_include_directories(open-amp
    PRIVATE
        #${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/compiler/gcc
        ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/processor/arm
        ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/system/freertos
        ${CMAKE_CURRENT_SOURCE_DIR}/open-amp/lib/include
        ${CMAKE_CURRENT_SOURCE_DIR}/open-amp/lib/rpmsg
        ${CMAKE_CURRENT_SOURCE_DIR}/virtual_driver

        ${PROJECT_SOURCE_DIR}/inc
    PUBLIC
    $<BUILD_INTERFACE:
        ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/compiler/gcc
        ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/processor/arm
        ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/system/freertos
        ${CMAKE_CURRENT_SOURCE_DIR}/libmetal/lib/include
        ${CMAKE_CURRENT_SOURCE_DIR}/open-amp/lib/include
        ${CMAKE_CURRENT_SOURCE_DIR}/open-amp/lib/rpmsg
        ${CMAKE_CURRENT_SOURCE_DIR}/virtual_driver
    >
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

set_target_properties(open-amp PROPERTIES LINKER_LANGUAGE C)

target_link_libraries(open-amp PRIVATE freertos)

# Create package of drivers
install(TARGETS open-amp DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Install headers
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
  PATTERN *.h
  PATTERN *.inl
  PATTERN *.hpp
  PATTERN *.ih
)