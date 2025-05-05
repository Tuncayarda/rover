cmake_minimum_required(VERSION 3.12)

set(NAME Rover)

include(pico_sdk_import.cmake)

project(${NAME} C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

set(MICRO_ROS_PATH
    "${CMAKE_CURRENT_LIST_DIR}/uros_ws/firmware/build"
    CACHE STRING ""
)

include(micro_ros.cmake)

add_subdirectory(pico_src/uart_lib)
add_subdirectory(pico_src/servo_lib)
add_subdirectory(pico_src/pico_motor)
add_subdirectory(pico_src/pico_servo)

install(CODE
    "execute_process(
        COMMAND
            $ENV{HOME}/bin/picoDeploy.sh
            ${CMAKE_CURRENT_BINARY_DIR}/pico_src/${NAME}.elf
    )"
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/pico_src/${NAME}.uf2
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)