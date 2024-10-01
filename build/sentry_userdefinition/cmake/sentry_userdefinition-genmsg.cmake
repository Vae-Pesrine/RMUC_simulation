# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "sentry_userdefinition: 1 messages, 3 services")

set(MSG_I_FLAGS "-Isentry_userdefinition:/home/jgy/RMUC_simulation/src/sentry_userdefinition/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sentry_userdefinition_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/msg/ScanMatchingStatus.msg" NAME_WE)
add_custom_target(_sentry_userdefinition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sentry_userdefinition" "/home/jgy/RMUC_simulation/src/sentry_userdefinition/msg/ScanMatchingStatus.msg" "std_msgs/Header:geometry_msgs/Transform:geometry_msgs/Quaternion:std_msgs/String:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalMap.srv" NAME_WE)
add_custom_target(_sentry_userdefinition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sentry_userdefinition" "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalMap.srv" "sensor_msgs/PointField:std_msgs/Header:sensor_msgs/PointCloud2"
)

get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/QueryGlobalLocalization.srv" NAME_WE)
add_custom_target(_sentry_userdefinition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sentry_userdefinition" "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/QueryGlobalLocalization.srv" "std_msgs/Header:geometry_msgs/Pose:geometry_msgs/Quaternion:sensor_msgs/PointCloud2:sensor_msgs/PointField:geometry_msgs/Point"
)

get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalLocalizationEngine.srv" NAME_WE)
add_custom_target(_sentry_userdefinition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sentry_userdefinition" "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalLocalizationEngine.srv" "std_msgs/String"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/msg/ScanMatchingStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sentry_userdefinition
)

### Generating Services
_generate_srv_cpp(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sentry_userdefinition
)
_generate_srv_cpp(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/QueryGlobalLocalization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sentry_userdefinition
)
_generate_srv_cpp(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalLocalizationEngine.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sentry_userdefinition
)

### Generating Module File
_generate_module_cpp(sentry_userdefinition
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sentry_userdefinition
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sentry_userdefinition_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sentry_userdefinition_generate_messages sentry_userdefinition_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/msg/ScanMatchingStatus.msg" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_cpp _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalMap.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_cpp _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/QueryGlobalLocalization.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_cpp _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalLocalizationEngine.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_cpp _sentry_userdefinition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sentry_userdefinition_gencpp)
add_dependencies(sentry_userdefinition_gencpp sentry_userdefinition_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sentry_userdefinition_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/msg/ScanMatchingStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sentry_userdefinition
)

### Generating Services
_generate_srv_eus(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sentry_userdefinition
)
_generate_srv_eus(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/QueryGlobalLocalization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sentry_userdefinition
)
_generate_srv_eus(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalLocalizationEngine.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sentry_userdefinition
)

### Generating Module File
_generate_module_eus(sentry_userdefinition
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sentry_userdefinition
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(sentry_userdefinition_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(sentry_userdefinition_generate_messages sentry_userdefinition_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/msg/ScanMatchingStatus.msg" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_eus _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalMap.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_eus _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/QueryGlobalLocalization.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_eus _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalLocalizationEngine.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_eus _sentry_userdefinition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sentry_userdefinition_geneus)
add_dependencies(sentry_userdefinition_geneus sentry_userdefinition_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sentry_userdefinition_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/msg/ScanMatchingStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sentry_userdefinition
)

### Generating Services
_generate_srv_lisp(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sentry_userdefinition
)
_generate_srv_lisp(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/QueryGlobalLocalization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sentry_userdefinition
)
_generate_srv_lisp(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalLocalizationEngine.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sentry_userdefinition
)

### Generating Module File
_generate_module_lisp(sentry_userdefinition
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sentry_userdefinition
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sentry_userdefinition_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sentry_userdefinition_generate_messages sentry_userdefinition_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/msg/ScanMatchingStatus.msg" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_lisp _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalMap.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_lisp _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/QueryGlobalLocalization.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_lisp _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalLocalizationEngine.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_lisp _sentry_userdefinition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sentry_userdefinition_genlisp)
add_dependencies(sentry_userdefinition_genlisp sentry_userdefinition_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sentry_userdefinition_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/msg/ScanMatchingStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sentry_userdefinition
)

### Generating Services
_generate_srv_nodejs(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sentry_userdefinition
)
_generate_srv_nodejs(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/QueryGlobalLocalization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sentry_userdefinition
)
_generate_srv_nodejs(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalLocalizationEngine.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sentry_userdefinition
)

### Generating Module File
_generate_module_nodejs(sentry_userdefinition
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sentry_userdefinition
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(sentry_userdefinition_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(sentry_userdefinition_generate_messages sentry_userdefinition_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/msg/ScanMatchingStatus.msg" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_nodejs _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalMap.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_nodejs _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/QueryGlobalLocalization.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_nodejs _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalLocalizationEngine.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_nodejs _sentry_userdefinition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sentry_userdefinition_gennodejs)
add_dependencies(sentry_userdefinition_gennodejs sentry_userdefinition_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sentry_userdefinition_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/msg/ScanMatchingStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentry_userdefinition
)

### Generating Services
_generate_srv_py(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentry_userdefinition
)
_generate_srv_py(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/QueryGlobalLocalization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentry_userdefinition
)
_generate_srv_py(sentry_userdefinition
  "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalLocalizationEngine.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentry_userdefinition
)

### Generating Module File
_generate_module_py(sentry_userdefinition
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentry_userdefinition
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sentry_userdefinition_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sentry_userdefinition_generate_messages sentry_userdefinition_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/msg/ScanMatchingStatus.msg" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_py _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalMap.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_py _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/QueryGlobalLocalization.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_py _sentry_userdefinition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jgy/RMUC_simulation/src/sentry_userdefinition/srv/SetGlobalLocalizationEngine.srv" NAME_WE)
add_dependencies(sentry_userdefinition_generate_messages_py _sentry_userdefinition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sentry_userdefinition_genpy)
add_dependencies(sentry_userdefinition_genpy sentry_userdefinition_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sentry_userdefinition_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sentry_userdefinition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sentry_userdefinition
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(sentry_userdefinition_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(sentry_userdefinition_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(sentry_userdefinition_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sentry_userdefinition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sentry_userdefinition
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(sentry_userdefinition_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(sentry_userdefinition_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(sentry_userdefinition_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sentry_userdefinition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sentry_userdefinition
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(sentry_userdefinition_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(sentry_userdefinition_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(sentry_userdefinition_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sentry_userdefinition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sentry_userdefinition
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(sentry_userdefinition_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(sentry_userdefinition_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(sentry_userdefinition_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentry_userdefinition)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentry_userdefinition\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentry_userdefinition
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(sentry_userdefinition_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(sentry_userdefinition_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(sentry_userdefinition_generate_messages_py sensor_msgs_generate_messages_py)
endif()
