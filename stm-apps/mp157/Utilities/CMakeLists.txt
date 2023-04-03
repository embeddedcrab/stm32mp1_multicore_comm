
cmake_minimum_required(VERSION 3.15)

project(Utilities VERSION 1.0.0 LANGUAGES CXX C DESCRIPTION "Utilities Library C++")


include(GNUInstallDirs)

find_package(Threads REQUIRED)

configure_file(
  ${PROJECT_SOURCE_DIR}/utilities/version.h.in
  ${PROJECT_BINARY_DIR}/utilities/version.h
  @ONLY
)
set(UTILITIES_RESOURCE ${PROJECT_SOURCE_DIR}/utilities/utilities.rc)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)


set(CMAKE_C_FLAGS "-O1 -lpthread -lrt ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-g -std=c++14 -O1 -pthread -Wall ${CMAKE_CXX_FLAGS}")


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

# Add source directory
add_subdirectory(utilities)


# Config find_package() files
include(CMakePackageConfigHelpers)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
INSTALL_DESTINATION
  ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}-${PROJECT_VERSION}
)

write_basic_package_version_file(
  "${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
DESTINATION
  ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}-${PROJECT_VERSION}
)

include(CPack)
