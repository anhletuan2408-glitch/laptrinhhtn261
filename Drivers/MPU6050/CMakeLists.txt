idf_component_register(
    SRCS "mpu6050.c" "port/mpu6050_port_esp32.c"
    INCLUDE_DIRS "." "port"
    REQUIRES driver freertos
)
