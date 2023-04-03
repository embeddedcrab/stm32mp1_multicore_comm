cmake_minimum_required(VERSION 3.15)


set(SOURCE_FILES main.cpp)


add_executable(hello-world ${SOURCE_FILES})

target_link_libraries(hello-world pthread)


install(TARGETS hello-world DESTINATION ${INSTALL_BIN_DIR})
