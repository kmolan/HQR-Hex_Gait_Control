# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "catkin_internal: 1 messages, 0 services")

set(MSG_I_FLAGS "-Icatkin_internal:/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(catkin_internal_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg" NAME_WE)
add_custom_target(_catkin_internal_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "catkin_internal" "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/PoseStamped:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(catkin_internal
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/catkin_internal
)

### Generating Services

### Generating Module File
_generate_module_cpp(catkin_internal
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/catkin_internal
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(catkin_internal_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(catkin_internal_generate_messages catkin_internal_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg" NAME_WE)
add_dependencies(catkin_internal_generate_messages_cpp _catkin_internal_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(catkin_internal_gencpp)
add_dependencies(catkin_internal_gencpp catkin_internal_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS catkin_internal_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(catkin_internal
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/catkin_internal
)

### Generating Services

### Generating Module File
_generate_module_eus(catkin_internal
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/catkin_internal
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(catkin_internal_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(catkin_internal_generate_messages catkin_internal_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg" NAME_WE)
add_dependencies(catkin_internal_generate_messages_eus _catkin_internal_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(catkin_internal_geneus)
add_dependencies(catkin_internal_geneus catkin_internal_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS catkin_internal_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(catkin_internal
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/catkin_internal
)

### Generating Services

### Generating Module File
_generate_module_lisp(catkin_internal
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/catkin_internal
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(catkin_internal_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(catkin_internal_generate_messages catkin_internal_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg" NAME_WE)
add_dependencies(catkin_internal_generate_messages_lisp _catkin_internal_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(catkin_internal_genlisp)
add_dependencies(catkin_internal_genlisp catkin_internal_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS catkin_internal_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(catkin_internal
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/catkin_internal
)

### Generating Services

### Generating Module File
_generate_module_nodejs(catkin_internal
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/catkin_internal
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(catkin_internal_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(catkin_internal_generate_messages catkin_internal_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg" NAME_WE)
add_dependencies(catkin_internal_generate_messages_nodejs _catkin_internal_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(catkin_internal_gennodejs)
add_dependencies(catkin_internal_gennodejs catkin_internal_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS catkin_internal_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(catkin_internal
  "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/catkin_internal
)

### Generating Services

### Generating Module File
_generate_module_py(catkin_internal
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/catkin_internal
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(catkin_internal_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(catkin_internal_generate_messages catkin_internal_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anmolk/kodlab/src/HQR-Hex_Gait_Control/msg/pose_info_msg.msg" NAME_WE)
add_dependencies(catkin_internal_generate_messages_py _catkin_internal_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(catkin_internal_genpy)
add_dependencies(catkin_internal_genpy catkin_internal_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS catkin_internal_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/catkin_internal)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/catkin_internal
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(catkin_internal_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(catkin_internal_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/catkin_internal)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/catkin_internal
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(catkin_internal_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(catkin_internal_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/catkin_internal)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/catkin_internal
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(catkin_internal_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(catkin_internal_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/catkin_internal)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/catkin_internal
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(catkin_internal_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(catkin_internal_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/catkin_internal)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/catkin_internal\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/catkin_internal
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(catkin_internal_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(catkin_internal_generate_messages_py std_msgs_generate_messages_py)
endif()
