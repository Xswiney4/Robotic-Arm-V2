idf_component_register(SRCS "robotFunctions.cpp" "roboticArm.cpp"
                    INCLUDE_DIRS "./include"
                    REQUIRES driver tasks log pca9548a as5600 stepper motorModule)