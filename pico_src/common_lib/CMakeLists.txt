cmake_minimum_required(VERSION 3.12)

set(NAME common_lib)
project(common_lib C CXX ASM)

add_library(common_lib STATIC
  pico_uart_transport.c
)

target_compile_options(${NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-Wno-implicit-function-declaration -Wno-builtin-declaration-mismatch>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-result>
)

target_include_directories(common_lib
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(common_lib
  PUBLIC
    micro_ros
    pico_stdlib
)

file(GLOB_RECURSE ALL_PATHS
  LIST_DIRECTORIES true
  "${MICRO_ROS_PATH}/include/*"
)

set(ROS2_INCLUDE_DIRS "")
foreach(path ${ALL_PATHS})
  if(IS_DIRECTORY ${path})
    list(APPEND ROS2_INCLUDE_DIRS ${path})
  endif()
endforeach()
list(REMOVE_DUPLICATES ROS2_INCLUDE_DIRS)

target_include_directories(common_lib
  PUBLIC
    ${ROS2_INCLUDE_DIRS}
)