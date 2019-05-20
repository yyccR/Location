cmake_minimum_required(VERSION 3.9)
project(Location)
set(CMAKE_CXX_STANDARD 11)

set(INC_DIR ./include/eigen3/)
include_directories(${INC_DIR})

include_directories(${PROJECT_SOURCE_DIR}/math)
add_subdirectory(math)

include_directories(${PROJECT_SOURCE_DIR}/models)
add_subdirectory(models)

include_directories(${PROJECT_SOURCE_DIR}/location)
add_subdirectory(location)

include_directories(${PROJECT_SOURCE_DIR}/sensor)
add_subdirectory(sensor)

include_directories(${PROJECT_SOURCE_DIR}/system)
add_subdirectory(system)

include_directories(${PROJECT_SOURCE_DIR}/test)
add_subdirectory(test)

#add_library(lib ./math/KalmanFilter.h ./math/KalmanFilter.cpp ./models/AHRS.cpp ./sensor/Accelerometer.cpp ./sensor/Gyroscope.cpp ./sensor/GPS.cpp ./sensor/Magnetometer.cpp ./math/Quaternions.cpp math/Optimizer.cpp sensor/Sensor.cpp sensor/Sensor.h location/Location.cpp system/Status.cpp ./include/googletest/ test/utils/DataFormat.cpp test/utils/DataFormat.h test/TestLocation.cpp test/TestCalibration.cpp test/TestCalibration.h math/LPF.cpp math/LPF.h models/StrapdownAHRS.cpp config/Config.cpp config/Config.h sensor/Gravity.cpp sensor/Gravity.h sensor/Compass.cpp sensor/Compass.h)
#link_libraries(lib)
#add_executable(Location main.cpp ${lib})
add_executable(Location main.cpp)

target_link_libraries(Location Location_math)
target_link_libraries(Location Location_models)
target_link_libraries(Location Location_location)
target_link_libraries(Location Location_sensor)
target_link_libraries(Location Location_system)
target_link_libraries(Location Location_test)