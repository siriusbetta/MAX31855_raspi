cmake_minimum_required(VERSION 3.15)
project(max31855)

set(SPI_SRC "spi/spi.c")
add_library(spi STATIC ${SPI_SRC})

set(SRC "src/MAX31855.cpp" src/gpio.cpp src/gpio.h)

add_subdirectory(MCP23017_raspi)

add_library(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} "spi" "mcp23017")
target_include_directories(${PROJECT_NAME} PUBLIC "spi" "src" "MCP23017_raspi/src" )
set(EXAMPLE "OFF" CACHE STRING "On/Off example")
if(EXAMPLE)
    add_subdirectory(example)
endif()