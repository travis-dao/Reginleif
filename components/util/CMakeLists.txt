idf_component_register(
  SRCS "types.cpp" "frame.cpp"
  INCLUDE_DIRS "include"
  REQUIRES motor esp_timer espressif__eigen core
)