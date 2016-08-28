
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libherkulex_driver-<target>.so
# and installs in the directory lib/orocos/herkulex_driver/
#
orocos_component(herkulex_driver herkulex_driver.hpp herkulex_driver.cpp) # ...you may add multiple source files
orocos_component(herkulex_array herkulex_array.hpp herkulex_array.cpp herkulex_servo.hpp herkulex_servo.cpp herkulex_servo_drs101.hpp herkulex_servo_drs101.cpp) # ...you may add multiple source files
orocos_component(herkulex_sched herkulex_sched.hpp herkulex_sched.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/herkulex_driver/ directory
#
# orocos_install_headers(herkulex_driver.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

