
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libcartesian_motion_control-<target>.so
# and installs in the directory lib/orocos/cartesian_motion_control/
#
orocos_component(kinematic_converter kinematic_converter.hpp kinematic_converter.cpp)
target_link_libraries(kinematic_converter ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})
orocos_component(CartesianControllerPos CartesianControllerPos.cpp CartesianControllerPos.hpp)
target_link_libraries(CartesianControllerPos ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})
orocos_component(CartesianControllerVel CartesianControllerVel.cpp CartesianControllerVel.hpp)
target_link_libraries(CartesianControllerVel ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})
orocos_component(CartesianControllerPosVel CartesianControllerPosVel.cpp CartesianControllerPosVel.hpp)
target_link_libraries(CartesianControllerPosVel ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})
orocos_component(CartesianGeneratorPos  CartesianGeneratorPos.cpp CartesianGeneratorPos.hpp)
target_link_libraries(CartesianGeneratorPos ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES} ${tf_conversions_INCLUDE_LIBRARIES})
orocos_component(CartesianImpedanceController  CartesianImpedanceController.cpp CartesianImpedanceController.hpp)
target_link_libraries(CartesianImpedanceController ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/cartesian_motion_control/ directory
#
# orocos_install_headers( CartesianControllerPos.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libcartesian_motion_control-plugin-<target>.so
# and installs in the directory lib/orocos/cartesian_motion_control/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
#orocos_plugin(cartesian_motion_control-plugin cartesian_motion_control-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libcartesian_motion_control-service-<target>.so
# and installs in the directory lib/orocos/cartesian_motion_control/plugins/
#
#orocos_service(cartesian_motion_control-service cartesian_motion_control-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libcartesian_motion_control-support-<target>.so and installs it in
# lib/
#
#orocos_library(cartesian_motion_control-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


