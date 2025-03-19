idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES tasks roboticArm pca9548a stepper as5600 esp_timer log)