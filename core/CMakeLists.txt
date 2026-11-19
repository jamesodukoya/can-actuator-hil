add_library(sbw_core
  src/alive_counter.cpp
  src/can_protocol.cpp
  src/crc8.cpp
  src/current_loop.cpp
  src/disagreement_monitor.cpp
  src/jam_observer.cpp
  src/rate_estimator.cpp
  src/sensor_conversion.cpp
  src/steering_controller.cpp
  src/steering_ecu.cpp)
target_include_directories(sbw_core PUBLIC include)
sbw_set_strict_warnings(sbw_core)
sbw_set_embedded_flags(sbw_core)
