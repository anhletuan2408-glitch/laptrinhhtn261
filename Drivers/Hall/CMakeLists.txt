idf_component_register(
    SRCS "hall.c" "port/hall_port_esp32.c"
    INCLUDE_DIRS "." "port"
    REQUIRES driver esp_timer freertos
)
