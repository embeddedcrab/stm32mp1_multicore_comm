
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

# Install Output binary
install(TARGETS ${PROJECT_NAME} DESTINATION ${INSTALL_BIN_DIR})