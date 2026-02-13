# Eye of Sauron main application component
FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS 
        "main.c"
        "gimbalControlSvc/gimbalControlSvc.c"
        "gimbalControlSvc/servoControl.c"
        "gimbalControlSvc/stepperControl.c"
        "commsSvc/commsSvc.c"
        "commsSvc/joystick.c"
    INCLUDE_DIRS 
        "."
        "gimbalControlSvc"
        "commsSvc"
    REQUIRES 
        driver 
        log 
        servo
)
