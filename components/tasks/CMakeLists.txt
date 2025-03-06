idf_component_register(SRCS "communication_task.cpp"
                            "control_task.cpp"
                            "kinematics_task.cpp"
                            "motor_task.cpp"
                    INCLUDE_DIRS "./include"
                    REQUIRES as5600 a4988)