idf_component_register(
  SRCS "base.cpp" "leg/leg.cpp" "leg/leg_controller.cpp"
  INCLUDE_DIRS "include" "leg/include"
  REQUIRES util peripherals motor algorithms
)