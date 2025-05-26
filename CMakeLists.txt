cmake_minimum_required(VERSION 3.15)
project(max31855)
set(SRC "MAX31855.cpp"
        "MAX31855soft.cpp")

add_library(${PROJECT_NAME} ${SRC})
