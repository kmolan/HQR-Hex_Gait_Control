# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hqrhex_control: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ihqrhex_control:/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hqrhex_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg" NAME_WE)
add_custom_target(_hqrhex_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hqrhex_control" "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/PoseStamped:std_msgs/Header"
)

get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/internal_states_msg.msg" NAME_WE)
add_custom_target(_hqrhex_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hqrhex_control" "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/internal_states_msg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hqrhex_control
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hqrhex_control
)
_generate_msg_cpp(hqrhex_control
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/internal_states_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hqrhex_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(hqrhex_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hqrhex_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hqrhex_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hqrhex_control_generate_messages hqrhex_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg" NAME_WE)
add_dependencies(hqrhex_control_generate_messages_cpp _hqrhex_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/internal_states_msg.msg" NAME_WE)
add_dependencies(hqrhex_control_generate_messages_cpp _hqrhex_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hqrhex_control_gencpp)
add_dependencies(hqrhex_control_gencpp hqrhex_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hqrhex_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(hqrhex_control
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hqrhex_control
)
_generate_msg_eus(hqrhex_control
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/internal_states_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hqrhex_control
)

### Generating Services

### Generating Module File
_generate_module_eus(hqrhex_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hqrhex_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hqrhex_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hqrhex_control_generate_messages hqrhex_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg" NAME_WE)
add_dependencies(hqrhex_control_generate_messages_eus _hqrhex_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/internal_states_msg.msg" NAME_WE)
add_dependencies(hqrhex_control_generate_messages_eus _hqrhex_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hqrhex_control_geneus)
add_dependencies(hqrhex_control_geneus hqrhex_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hqrhex_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hqrhex_control
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hqrhex_control
)
_generate_msg_lisp(hqrhex_control
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/internal_states_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hqrhex_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(hqrhex_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hqrhex_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hqrhex_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hqrhex_control_generate_messages hqrhex_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg" NAME_WE)
add_dependencies(hqrhex_control_generate_messages_lisp _hqrhex_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/internal_states_msg.msg" NAME_WE)
add_dependencies(hqrhex_control_generate_messages_lisp _hqrhex_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hqrhex_control_genlisp)
add_dependencies(hqrhex_control_genlisp hqrhex_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hqrhex_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(hqrhex_control
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hqrhex_control
)
_generate_msg_nodejs(hqrhex_control
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/internal_states_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hqrhex_control
)

### Generating Services

### Generating Module File
_generate_module_nodejs(hqrhex_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hqrhex_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hqrhex_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hqrhex_control_generate_messages hqrhex_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg" NAME_WE)
add_dependencies(hqrhex_control_generate_messages_nodejs _hqrhex_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/internal_states_msg.msg" NAME_WE)
add_dependencies(hqrhex_control_generate_messages_nodejs _hqrhex_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hqrhex_control_gennodejs)
add_dependencies(hqrhex_control_gennodejs hqrhex_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hqrhex_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hqrhex_control
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hqrhex_control
)
_generate_msg_py(hqrhex_control
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/internal_states_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hqrhex_control
)

### Generating Services

### Generating Module File
_generate_module_py(hqrhex_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hqrhex_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hqrhex_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hqrhex_control_generate_messages hqrhex_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg" NAME_WE)
add_dependencies(hqrhex_control_generate_messages_py _hqrhex_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/internal_states_msg.msg" NAME_WE)
add_dependencies(hqrhex_control_generate_messages_py _hqrhex_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hqrhex_control_genpy)
add_dependencies(hqrhex_control_genpy hqrhex_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hqrhex_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hqrhex_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hqrhex_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(hqrhex_control_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(hqrhex_control_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hqrhex_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hqrhex_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(hqrhex_control_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(hqrhex_control_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hqrhex_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hqrhex_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(hqrhex_control_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(hqrhex_control_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hqrhex_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hqrhex_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(hqrhex_control_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(hqrhex_control_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hqrhex_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hqrhex_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hqrhex_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(hqrhex_control_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(hqrhex_control_generate_messages_py std_msgs_generate_messages_py)
endif()
