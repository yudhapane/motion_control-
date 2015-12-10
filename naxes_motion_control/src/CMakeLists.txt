
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
orocos_component(nAxesControllerPos nAxesControllerPos.cpp)
target_link_libraries(nAxesControllerPos ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})
orocos_component(nAxesGeneratorPos nAxesGeneratorPos.cpp)
target_link_libraries(nAxesGeneratorPos ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/cartesian_motion_control/ directory
#
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


