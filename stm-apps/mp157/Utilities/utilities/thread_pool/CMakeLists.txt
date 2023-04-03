
cmake_minimum_required(VERSION 3.15)

set(ThreadPool_Src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/thread_pool.cpp
)

add_library(threadPoollib STATIC ${ThreadPool_Src})

target_link_libraries(threadPoollib
  PUBLIC
    pmrlib
    Threads::Threads
  PRIVATE
  INTERFACE
)

target_include_directories(threadPoollib
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../pmr/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(TARGETS threadPoollib EXPORT ${CMAKE_PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.inl"
  PATTERN "*.hpp"
  PATTERN "*.ih"
)