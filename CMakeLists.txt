cmake_minimum_required(VERSION 2.8.3)
project(asr_state_machine)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
endif()


find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  actionlib
  actionlib_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  asr_msgs
  asr_aruco_marker_recognition
  asr_fake_object_recognition
  asr_world_model
  asr_recognizer_prediction_ism
  asr_next_best_view
  move_base
  asr_flir_ptu_controller
  tf
  tf2_ros
  asr_visualization_server
)

set(EXT_INCLUDES ${catkin_INCLUDE_DIRS})

## Check whether optional package asr_descriptor_surface_based_recognition is available
find_package(asr_descriptor_surface_based_recognition QUIET)

if(asr_descriptor_surface_based_recognition_FOUND)
    list(APPEND EXT_INCLUDES ${asr_descriptor_surface_based_recognition_INCLUDE_DIRS})
    message(STATUS "${BoldGreen}Found asr_descriptor_surface_based_recognition${ColourReset}")
else()
    message(STATUS "${BoldYellow}Optional package asr_descriptor_surface_based_recognition could not be found${ColourReset}")
endif()


## Check whether optional package recognition_for_grasping is available
find_package(recognition_for_grasping QUIET)

if(recognition_for_grasping_FOUND)
    list(APPEND EXT_INCLUDES ${recognition_for_grasping_INCLUDE_DIRS})
    message(STATUS "${BoldGreen}Found recognition_for_grasping${ColourReset}")
else()
    message(STATUS "${BoldYellow}Optional package recognition_for_grasping could not be found${ColourReset}")
endif()


catkin_package(

)


include_directories(
  ${EXT_INCLUDES}
)

