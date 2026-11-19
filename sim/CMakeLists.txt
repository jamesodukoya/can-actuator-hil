add_library(sbw_sim
  src/axle_plant.cpp
  src/sensor_model.cpp)
target_include_directories(sbw_sim PUBLIC include)
target_link_libraries(sbw_sim PUBLIC sbw_core)
sbw_set_strict_warnings(sbw_sim)
sbw_set_embedded_flags(sbw_sim)
