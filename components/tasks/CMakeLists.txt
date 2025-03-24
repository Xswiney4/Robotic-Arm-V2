idf_component_register(SRCS "communication_task.cpp"
                            "control_task.cpp"
                            "kinematics_task.cpp"
                            "motor_task.cpp"
                    INCLUDE_DIRS "./include"
                    REQUIRES as5600 pca9548a stepper tasks log roboticArm) # bt nvs_flash