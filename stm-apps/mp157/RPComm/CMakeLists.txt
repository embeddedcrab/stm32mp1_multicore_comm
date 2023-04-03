cmake_minimum_required(VERSION 3.15)

project(RPComm VERSION 1.0.0 LANGUAGES CXX C DESCRIPTION "RP Communication C++ (Test)")

include(GNUInstallDirs)


find_package(Threads REQUIRED)
find_package(Utilities REQUIRED)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O1 ${CMAKE_CXX_FLAGS}")

set(INSTALL_BIN_DIR /usr/bin)

add_subdirectory(src)