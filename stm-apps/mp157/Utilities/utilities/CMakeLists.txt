
cmake_minimum_required(VERSION 3.15)


set(UtilsHeaders
  ${CMAKE_CURRENT_SOURCE_DIR}/pmr/inc/pmr_headers.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/pmr/inc/polymorphic_allocator.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/pmr/inc/monotonic_buffer_resource.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sysutils/inc/reset_handler.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sysutils/inc/signal_handler.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/semaphore/inc/semaphore_interface.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/semaphore/inc/semaphore.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/shared_memory/inc/shared_memory_interface.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/shared_memory/inc/shared_memory.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/thread_pool/inc/thread_pool_interface.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/thread_pool/inc/thread_pool.hpp
)

# Add library
add_library(utilities
  ${UTILITIES_RESOURCE}
  ${CMAKE_CURRENT_SOURCE_DIR}/pmr/src/polymorphic_allocator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/pmr/src/monotonic_buffer_resource.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/semaphore/src/semaphore_.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/shared_memory/src/shared_memory.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/thread_pool/src/thread_pool.cpp
)

if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(utilities PUBLIC "-pthread -rt")
endif()

target_link_libraries(utilities
  PUBLIC
  PRIVATE
    rt
    Threads::Threads
  INTERFACE
)


target_include_directories(utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stl/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/pmr/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/sysutils/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/semaphore/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/shared_memory/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/thread_pool/inc
    ${CMAKE_CURRENT_LIST_DIR}
)

set_property(TARGET utilities PROPERTY POSITION_INDEPENDENT_CODE ON)

# Install library + targets
set_target_properties(utilities
  PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR})

install(TARGETS utilities
  EXPORT ${PROJECT_NAME}_Exports
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    NAMELINK_ONLY
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(EXPORT ${PROJECT_NAME}_Exports 
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}-${PROJECT_VERSION}
  NAMESPACE utils::
)

install(TARGETS utilities
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    NAMELINK_ONLY
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Install directory and headers
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.inl"
  PATTERN "*.hpp"
  PATTERN "*.ih"
)

install(FILES ${UtilsHeaders}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
