cmake_minimum_required(VERSION 3.15)
project(example)

add_executable(${PROJECT_NAME} "example.cpp")

target_link_libraries( ${PROJECT_NAME} mcp23017 max31855)
target_include_directories(${PROJECT_NAME} PUBLIC "../MCP23017_raspi/src")