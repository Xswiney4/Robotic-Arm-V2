idf_component_register(SRCS "roboticArm.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES driver tasks log pca9548a as5600 stepper)