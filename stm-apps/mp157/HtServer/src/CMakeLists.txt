
cmake_minimum_required(VERSION 3.15)

# Set Source Files
set(Src
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

# Add Executables
add_executable(${PROJECT_NAME}
  ${Src}
)

# Target Link Libraries
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    utilities
    rt
    Threads::Threads
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/init
    ${CMAKE_CURRENT_SOURCE_DIR}/comm
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_LIST_DIR}
)

# Install Output binary
install(TARGETS ${PROJECT_NAME} DESTINATION ${INSTALL_BIN_DIR})