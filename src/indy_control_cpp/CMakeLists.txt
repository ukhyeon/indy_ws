cmake_minimum_required(VERSION 3.8)
project(indy_control_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hrc_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# =========================
# IndyDCP3 library
# =========================

find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(pinocchio REQUIRED)


file(GLOB INDY_PROTO_SRCS
  proto/*.pb.cc
  proto/*.grpc.pb.cc
)

add_library(indydcp3
  src/indydcp3.cpp
  ${INDY_PROTO_SRCS}
)


target_include_directories(indydcp3
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/proto>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(indydcp3
  PUBLIC
    protobuf::libprotobuf
    gRPC::grpc++
    gRPC::grpc++_reflection
)

# =========================
# Header-only performance logger
# =========================
add_library(perf_logger INTERFACE)

target_include_directories(perf_logger
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

#Check;  connection check node
add_executable(indy_connection_check_node src/check/indy_connection_check_node.cpp)

ament_target_dependencies(indy_connection_check_node
  rclcpp
  std_msgs
)
target_link_libraries(indy_connection_check_node
  indydcp3
  perf_logger
)
install(TARGETS
  indy_connection_check_node
  DESTINATION lib/${PROJECT_NAME}
)

#Check; 유효잘량 체크
add_executable(indy_pinocchio_meff_check src/check/indy_pinocchio_meff_check.cpp)
target_link_libraries(indy_pinocchio_meff_check pinocchio::pinocchio)
install(TARGETS
  indy_pinocchio_meff_check
  DESTINATION lib/${PROJECT_NAME}
)

#Check; 유효질량 검증용 (매트랩 비교)
add_executable(indy_pinocchio_meff_with_gearing
  src/check/indy_pinocchio_meff_with_gearing.cpp
)
target_link_libraries(indy_pinocchio_meff_with_gearing
  pinocchio::pinocchio
)
install(TARGETS
  indy_pinocchio_meff_with_gearing
  DESTINATION lib/${PROJECT_NAME}
)

#Check ; indy speed check node
add_executable(indy_speed_check_node
  src/check/indy_speed_check_node.cpp
)
ament_target_dependencies(indy_speed_check_node
  rclcpp
)
target_link_libraries(indy_speed_check_node
  indydcp3
  pinocchio::pinocchio
  perf_logger
)
install(TARGETS
  indy_speed_check_node
  DESTINATION lib/${PROJECT_NAME}
)

# indy_speeed _node
add_executable(indy_speed_control_node src/indy_speed_control_node.cpp)

ament_target_dependencies(indy_speed_control_node
  rclcpp
  hrc_interfaces
)
target_link_libraries(indy_speed_control_node
  indydcp3
  perf_logger
)
install(TARGETS
  indy_speed_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# robot_dynamics
add_executable(robot_dynamics_node
  src/robot_dynamics_node.cpp)

target_link_libraries(robot_dynamics_node
  indydcp3
  pinocchio::pinocchio
  perf_logger
  )
install(TARGETS
  robot_dynamics_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_target_dependencies(robot_dynamics_node
  rclcpp
  std_msgs
  hrc_interfaces
)


#collision_assessment
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(collision_assessment_node
  src/collision_assessment_node.cpp
)
ament_target_dependencies(collision_assessment_node
  rclcpp
  std_msgs
  geometry_msgs
  hrc_interfaces
)
target_link_libraries(collision_assessment_node
  nlohmann_json::nlohmann_json
  Eigen3::Eigen
  perf_logger
)

# collision check 
add_executable(collision_assessment_check
  src/check/collision_assessment_check.cpp
)
ament_target_dependencies(collision_assessment_check
  rclcpp
  std_msgs
  geometry_msgs
  hrc_interfaces
)
target_link_libraries(collision_assessment_check
  nlohmann_json::nlohmann_json
  Eigen3::Eigen
)



install(TARGETS
  collision_assessment_node
  collision_assessment_check
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()
