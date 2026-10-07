idf_component_register(
    SRCS "sensor_task.c"
    INCLUDE_DIRS "."
    REQUIRES MPU6050 Hall Speed EventDetection driver esp_timer freertos
)
