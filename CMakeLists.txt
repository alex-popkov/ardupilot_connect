cmake_minimum_required(VERSION 3.16)
project(mav_listener CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Шлях до header-only бібліотеки MAVLink (git clone https://github.com/mavlink/c_library_v2.git)
set(MAVLINK_DIR "$ENV{HOME}/www/cpp_embedded/libs/mavlink/c_library_v2" CACHE PATH "Path to mavlink c_library_v2")

if(NOT EXISTS "${MAVLINK_DIR}/ardupilotmega/mavlink.h")
  message(FATAL_ERROR "MAVLink headers not found in ${MAVLINK_DIR}. "
                      "Pass -DMAVLINK_DIR=/path/to/c_library_v2")
endif()

add_library(mav_listener_lib
    src/port_controllers/udp_server.cpp
)

add_executable(mav_listener main.cpp)

# SYSTEM — щоб попередження з C-заголовків MAVLink не засмічували вивід
target_include_directories(mav_listener SYSTEM PRIVATE ${MAVLINK_DIR})
target_link_libraries(mav_listener PRIVATE mav_listener_lib)
target_compile_options(mav_listener PRIVATE -Wall -Wextra -pedantic)
target_compile_options(mav_listener_lib PRIVATE -Wall -Wextra -pedantic)