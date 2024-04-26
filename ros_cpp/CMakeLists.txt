cmake_minimum_required(VERSION 3.8)
project(ros_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_tutorials_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED) #HW03
find_package(geometry_msgs REQUIRED) #HW03
find_package(rosidl_default_generators REQUIRED) #HW03
find_package(ros_cpp REQUIRED)                      

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#HM1 ------------------------------------
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
# ---------------------------------------

#HM2 -----------------------------------
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)

add_executable(minimal_param_node src/cpp_parameters_node.cpp)
ament_target_dependencies(minimal_param_node rclcpp)

add_library(action_server SHARED
  src/fibonacci_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "ros_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(action_client SHARED
  src/fibonacci_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "ros_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# -----------------------------------------


# START HW3 -------------------------------------

add_executable(sensors_sub src/sensors_subscriber.cpp)
ament_target_dependencies(sensors_sub sensor_msgs rclcpp)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

add_executable(custom_talker src/custom_messages_talker.cpp)
ament_target_dependencies(custom_talker rclcpp ros_cpp)    # CHANGE

add_executable(custom_listener src/custom_messages_listener.cpp)
ament_target_dependencies(custom_listener rclcpp ros_cpp)  # CHANGE

add_executable(custom_server src/custom_messages_server.cpp)
ament_target_dependencies(custom_server rclcpp ros_cpp)                      # CHANGE

add_executable(custom_client src/custom_messages_client.cpp)
ament_target_dependencies(custom_client rclcpp ros_cpp)                      # CHANGE

# END HW3 ---------------------------------------

# Behvaior Tree Testing...
#add_executable(behaviorTree_Tutorial src/behaviorTree_Tutorial.cpp)
#ament_target_dependencies(behaviorTree_Tutorial rclcpp ros_cpp)                          # CHANGE



install(TARGETS
  talker    #HW01
  listener  #HW01
  server    #HW02 - part1
  client    #HW02 - part1
  minimal_param_node
  sensors_sub #HW03
  custom_talker
  custom_listener
  custom_server
  custom_client

  #behaviorTree_Tutorial
  DESTINATION lib/${PROJECT_NAME})

# Install Launch File
install(DIRECTORY
    launch
    config
    models
    worlds
    rviz
    #behaviorTree
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
