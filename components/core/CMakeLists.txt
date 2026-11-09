idf_component_register(
  SRCS
    "base.cpp"
    "leg/leg.cpp"
    "leg/leg_controller.cpp"
    "stabilizer/stabilizer.cpp"
    "motion/motion_subsystem.cpp"
    "motion/state_estimator.cpp"
  INCLUDE_DIRS
    "include"
    "leg/include"
    "stabilizer/include"
    "motion/include"
  REQUIRES util peripherals motor mpc algorithms
)