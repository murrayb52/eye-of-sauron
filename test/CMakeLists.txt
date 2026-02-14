idf_component_register(
    SRCS 
        "test_servoControl.c"
        "test_stepperControl.cpp"
        "stepperControl_stubs.c"
        "${CMAKE_SOURCE_DIR}/src/gimbalControlSvc/stepperControl.c"
    INCLUDE_DIRS 
        "."
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/src/gimbalControlSvc"
    PRIV_REQUIRES freertos
)