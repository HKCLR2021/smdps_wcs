cmake_minimum_required(VERSION 3.8)
project(wcs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(smdps_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(open62541pp REQUIRED)

include_directories(
  include 
)

add_executable(prod_line_ctrl 
  src/prod_line_ctrl.cpp
  src/http_svr_handler.cpp
  src/http_svr_utilities.cpp
  src/http_cli.cpp
)
target_link_libraries(prod_line_ctrl
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(prod_line_ctrl 
  rclcpp 
  rclcpp_action
  std_msgs
  std_srvs
  smdps_msgs
)

add_executable(dis_station_node 
  src/dis_station_node.cpp
  src/opcua_cli.cpp
  src/opcua_cb.cpp
  src/opcua_action.cpp
  src/opcua_utilities.cpp
)
target_link_libraries(dis_station_node
  open62541pp::open62541pp
)
ament_target_dependencies(dis_station_node 
  rclcpp 
  std_msgs
  std_srvs
  smdps_msgs
  open62541pp
)

add_executable(fake_printing_srv_node 
  src/fake_printing_srv.cpp
)
ament_target_dependencies(fake_printing_srv_node 
  rclcpp 
  smdps_msgs
)

add_executable(fake_new_order_cli_node 
  src/fake_new_order_cli.cpp
)
ament_target_dependencies(fake_new_order_cli_node 
  rclcpp 
  rclcpp_action
  std_msgs
  smdps_msgs
)

install(TARGETS
  prod_line_ctrl
  dis_station_node
  fake_printing_srv_node
  fake_new_order_cli_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

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

ament_package()
