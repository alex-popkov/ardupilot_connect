cmake_minimum_required(VERSION 3.16)
project(mav_listener CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Шлях до header-only бібліотеки MAVLink (git clone https://github.com/mavlink/c_library_v2.git)
set(MAVLINK_DIR "$ENV{HOME}/www/cpp_embedded/libs/mavlink/c_library_v2" CACHE PATH "Path to mavlink c_library_v2")

if(NOT EXISTS "${MAVLINK_DIR}/ardupilotmega/mavlink.h")
  message(FATAL_ERROR "MAVLink headers not found in ${MAVLINK_DIR}. "
                      "Pass -DMAVLINK_DIR=/path/to/c_library_v2")
endif()

add_library(mav_listener_lib
    src/transport/udp_server.cpp
    src/mavlink/mavlink_parser.cpp
    src/mavlink/mavlink_sender.cpp
    src/mavlink/mavlink_utils.cpp
    src/vehicle/telemetry_handler.cpp
    src/vehicle/flight_sequence.cpp
)

add_executable(mav_listener src/main.cpp)

target_include_directories(mav_listener_lib SYSTEM PUBLIC ${MAVLINK_DIR})
target_include_directories(mav_listener_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mav_listener PRIVATE mav_listener_lib)
target_compile_options(mav_listener PRIVATE -Wall -Wextra -pedantic)
target_compile_options(mav_listener_lib PRIVATE -Wall -Wextra -pedantic)