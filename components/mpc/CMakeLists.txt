idf_component_register(
  SRCS "mpc.cpp" "foothold_planner.cpp"
  INCLUDE_DIRS "include"
  REQUIRES util esp_timer
)