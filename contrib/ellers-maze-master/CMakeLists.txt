cmake_minimum_required(VERSION 3.16.0)

PROJECT(ellersmaze VERSION 1.0.0 
)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(IS_ROOT_PROJECT ON)
else()
    set(IS_ROOT_PROJECT OFF)
endif()

set(CMAKE_CXX_STANDARD          20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ELLERSMAZE_EXAMPLE "Enable the example executable for ellers-maze" ${IS_ROOT_PROJECT})

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)


if(ELLERSMAZE_EXAMPLE)
    add_executable(maze_example ${CMAKE_CURRENT_SOURCE_DIR}/example/main.cpp)
    target_link_libraries(maze_example PRIVATE ${PROJECT_NAME})
endif()

if(IS_ROOT_PROJECT)
    # copy compile commands to the root dir for the clangd
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E create_hardlink "${CMAKE_BINARY_DIR}/compile_commands.json" "${CMAKE_SOURCE_DIR}/compile_commands.json"
        COMMAND_ECHO STDOUT
        )
endif()
