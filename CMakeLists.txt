cmake_minimum_required(VERSION 3.9)
project(Location)

set(CMAKE_CXX_STANDARD 11)

set(INC_DIR ./include/eigen3/)

include_directories(${INC_DIR})

add_library(lib ./models/AHRS.cpp ./sensor/Accelerometer.cpp ./sensor/Gyroscope.cpp ./sensor/Magnetometer.cpp ./math/Quaternions.cpp math/Optimizer.cpp math/Optimizer.h system/Parameters.cpp system/Parameters.h sensor/Sensor.cpp sensor/Sensor.h)
link_libraries(lib)

add_executable(Location main.cpp ${lib})

#target_link_libraries(Location lib)