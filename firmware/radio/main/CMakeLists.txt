# main/CMakeLists.txt for CW Radio BLE Server

idf_component_register(SRCS "main.c"
                      INCLUDE_DIRS "."
                      REQUIRES nvs_flash driver esp_timer bt)
