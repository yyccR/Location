aux_source_directory(. location_src_lists)
add_library(Location_location ${location_src_lists})

target_link_libraries(
        Location_location
        Location_sensor
        Location_system
        Location_models
)