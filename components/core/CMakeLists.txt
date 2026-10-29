idf_component_register(
  SRCS "base.cpp" "leg/leg.cpp" "leg/leg_controller.cpp" "stabilizer/stabilizer.cpp"
  INCLUDE_DIRS "include" "leg/include" "stabilizer/include"
  REQUIRES util peripherals motor
)