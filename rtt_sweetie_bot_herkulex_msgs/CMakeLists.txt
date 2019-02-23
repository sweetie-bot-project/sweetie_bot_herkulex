cmake_minimum_required(VERSION 2.8.3)
project(rtt_sweetie_bot_herkulex_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm sweetie_bot_herkulex_msgs)
include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

ros_generate_rtt_typekit(sweetie_bot_herkulex_msgs)
ros_generate_rtt_service_proxies(rtt_sweetie_bot_herkulex_msgs)

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
