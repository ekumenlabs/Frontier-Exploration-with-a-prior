##############################################################################
# NODELET
##############################################################################

add_library(${PROJECT_NAME}let ${PROJECT_NAME}let.cpp)
add_dependencies(${PROJECT_NAME}let ca_driver)
target_link_libraries(${PROJECT_NAME}let)

install(TARGETS ${PROJECT_NAME}let
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
