idf_component_register(SRCS "motor_monitor_task.cpp" "communication_task.cpp"
                            "control_task.cpp"
                            "kinematics_task.cpp"
                            "motor_task.cpp"
                    INCLUDE_DIRS "./include"
                    REQUIRES as5600 pca9548a motorModule tasks log roboticArm) # bt nvs_flash