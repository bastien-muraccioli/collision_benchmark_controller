set(controller_SRC CircularOriController.cpp)

set(controller_HDR CircularOriController.h)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS
                                                 "-DCircularOriController_EXPORTS")
