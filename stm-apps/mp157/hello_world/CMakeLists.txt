cmake_minimum_required(VERSION 3.15)

project(HelloWorld CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O1")

set(INSTALL_BIN_DIR /usr/bin)

add_subdirectory(src)
#add_subdirectory(test)

