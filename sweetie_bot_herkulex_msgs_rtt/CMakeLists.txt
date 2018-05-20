cmake_minimum_required(VERSION 2.8.3)
project(sweetie_bot_herkulex_msgs_rtt)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm sweetie_bot_herkulex_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

ros_generate_rtt_typekit(sweetie_bot_herkulex_msgs)
ros_generate_rtt_service_proxies(sweetie_bot_herkulex_msgs_rtt)

orocos_generate_package(
  DEPENDS sweetie_bot_herkulex_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs
)

if (DEFINED _pkgconfig_error)
if (${_pkgconfig_error})
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
else()
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
