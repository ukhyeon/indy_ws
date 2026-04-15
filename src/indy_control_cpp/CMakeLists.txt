cmake_minimum_required(VERSION 3.8)
project(indy_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

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
find_package(rclcpp REQUIRED)

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

# indy_speeed _node
add_executable(indy_speed_control_node src/indy_speed_control_node.cpp)

ament_target_dependencies(indy_speed_control_node
  rclcpp
  std_msgs
)
target_link_libraries(indy_speed_control_node
  indydcp3
)
install(TARGETS
  indy_speed_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# connection check node
add_executable(indy_connection_check_node src/indy_connection_check_node.cpp)

ament_target_dependencies(indy_connection_check_node
  rclcpp
  std_msgs
)
target_link_libraries(indy_connection_check_node
  indydcp3
)
install(TARGETS
  indy_connection_check_node
  DESTINATION lib/${PROJECT_NAME}
)

#피노키오
add_executable(indy_pinocchio_meff_check src/indy_pinocchio_meff_check.cpp)
target_link_libraries(indy_pinocchio_meff_check pinocchio::pinocchio)
install(TARGETS
  indy_pinocchio_meff_check
  DESTINATION lib/${PROJECT_NAME}
)

# 유효질량 검증용 (매트랩 비교)
add_executable(indy_pinocchio_meff_with_gearing
  src/indy_pinocchio_meff_with_gearing.cpp
)
target_link_libraries(indy_pinocchio_meff_with_gearing
  pinocchio::pinocchio
)
install(TARGETS
  indy_pinocchio_meff_with_gearing
  DESTINATION lib/${PROJECT_NAME}
)

# 실시간 데이터 기반 유효질량
add_executable(robot_dynamics_node
  src/robot_dynamics_node.cpp)

target_link_libraries(robot_dynamics_node
  indydcp3
  pinocchio::pinocchio)
install(TARGETS
  robot_dynamics_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_target_dependencies(robot_dynamics_node
  rclcpp
  std_msgs
)

#indy speed check node
add_executable(indy_speed_check_node
  src/indy_speed_check_node.cpp
)
ament_target_dependencies(indy_speed_check_node
  rclcpp
)
target_link_libraries(indy_speed_check_node
  indydcp3
  pinocchio::pinocchio
)
install(TARGETS
  indy_speed_check_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
