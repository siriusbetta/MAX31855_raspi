cmake_minimum_required(VERSION 3.15)
project(max31855)

set(SPI_SRC "../spi/spi.c")
add_library(spi STATIC ${SPI_SRC})

set(SRC "src/MAX31855.cpp"
        )

add_library(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} "spi" "mcp23017")
target_include_directories(${PROJECT_NAME} PUBLIC "../spi" "src" "../MCP23017_raspi/src" )
