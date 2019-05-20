aux_source_directory(. sensor_src_lists)
add_library(Location_sensor ${sensor_src_lists})
target_link_libraries(Location_sensor Location_math)