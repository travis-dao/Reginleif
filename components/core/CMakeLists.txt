idf_component_register(
  SRCS
    "base.cpp"
    "leg_controller.cpp"
    "imu_subsystem.cpp"
    "motion_subsystem.cpp"
  INCLUDE_DIRS
    "include"
  REQUIRES util peripherals motor algorithms rl
)