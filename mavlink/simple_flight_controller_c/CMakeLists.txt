cmake_minimum_required(VERSION 3.6)
project(simple_flight_controller_c C)

set(CMAKE_C_STANDARD 11)

add_executable(simple_flight_controller_c src/main.c src/mavlink/mavlink_com.c src/mavlink/mavlink_com.h src/util/common.h src/cmd_handler/cmd_handler.c src/cmd_handler/cmd_handler.h src/pid/pid.c src/pid/pid.h src/pid/calculate_pids.c src/pid/calculate_pids.h src/util/common.c src/mavlink/mavlink_msg_handler.c src/mavlink/mavlink_msg_handler.h src/drone/drone.c src/drone/drone.h)
include_directories(${CMAKE_SOURCE_DIR}/deps)
target_link_libraries(simple_flight_controller_c pthread)