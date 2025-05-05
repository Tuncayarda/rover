cmake_minimum_required(VERSION 3.12)

set(NAME pico_servo)
project(pico_servo C CXX ASM)

add_executable(${NAME}
    app.cpp
)

target_compile_options(${NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-Wno-implicit-function-declaration -Wno-builtin-declaration-mismatch>
)

target_link_libraries(${NAME}
    pico_stdlib
    micro_ros
    hardware_pwm
)

file(GLOB_RECURSE ALL_PATHS LIST_DIRECTORIES true "${MICRO_ROS_PATH}/include/*")

set(ROS2_INCLUDE_DIRS "")
foreach(path ${ALL_PATHS})
    if(IS_DIRECTORY ${path})
        list(APPEND ROS2_INCLUDE_DIRS ${path})
    endif()
endforeach()

list(REMOVE_DUPLICATES ROS2_INCLUDE_DIRS)

target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${ROS2_INCLUDE_DIRS}
)

pico_add_extra_outputs(${NAME})

pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)