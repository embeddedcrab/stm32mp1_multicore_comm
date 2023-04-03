
cmake_minimum_required(VERSION 3.15)

set(Shm_Src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/shared_memory.cpp
)

add_library(shmlib STATIC ${Shm_Src})

target_link_libraries(shmlib
  PUBLIC
    pmrlib
    semaphorelib
  PRIVATE
    rt
    Threads::Threads
  INTERFACE
)

target_include_directories(shmlib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../pmr/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)


install(TARGETS shmlib EXPORT ${CMAKE_PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.inl"
  PATTERN "*.hpp"
  PATTERN "*.ih"
)