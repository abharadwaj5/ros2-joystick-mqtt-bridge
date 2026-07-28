cmake_minimum_required(VERSION 3.8)
project(remote_robot_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fallback_logic test/test_fallback_logic.cpp)
  target_include_directories(test_fallback_logic PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
# find_package(my_robotthinker_interfaces REQUIRED)

add_executable(mqtt_bridge src/mqtt_bridge_node.cpp)
target_include_directories(mqtt_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mqtt_bridge rclcpp geometry_msgs)
target_link_libraries(mqtt_bridge mosquitto nlohmann_json::nlohmann_json)


install(TARGETS
  mqtt_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
