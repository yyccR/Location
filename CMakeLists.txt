cmake_minimum_required(VERSION 3.9)
project(Location)

set(CMAKE_CXX_STANDARD 11)

set(INC_DIR D:/cpp/eigen/eigen3)

include_directories(${INC_DIR})

#link_libraries(eigen3)

add_executable(Location main.cpp common/kalmanFilter.cpp common/kalmanFilter.h common/KalmanFilter.cpp common/KalmanFilter.h rtk/RtkLocation.cpp rtk/RtkLocation.h)

#target_link_libraries(Location eigen3)