cmake_minimum_required(VERSION 3.9)
project(Location)

set(CMAKE_CXX_STANDARD 11)

#set(googleTestDir ./include/googletest)
#add_subdirectory(${googleTestDir})
#set(LIBRARIES
#        gtest
#        pthread)

set(INC_DIR ./include/eigen3/)
include_directories(${INC_DIR})

add_library(lib ./models/AHRS.cpp ./sensor/Accelerometer.cpp ./sensor/Gyroscope.cpp ./sensor/GPS.cpp ./sensor/Magnetometer.cpp ./math/Quaternions.cpp math/Optimizer.cpp sensor/Sensor.cpp sensor/Sensor.h location/Location.cpp system/Status.cpp ./include/googletest/ test/utils/DataFormat.cpp test/utils/DataFormat.h test/TestLocation.cpp test/TestCalibration.cpp test/TestCalibration.h math/LPF.cpp math/LPF.h)
link_libraries(lib)
add_executable(Location main.cpp ${lib})
#target_link_libraries(Location gtest)
#target_link_libraries(GTest gmock gmock_main)
#target_link_libraries(Location lib)