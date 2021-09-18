cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)
set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include) 
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)
#unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)
project(marvelmind_nav)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(localization_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
#find_package(rosidl_typesupport_cpp REQUIRED)


include_directories(include 
  ${rclcpp_INCLUDE_DIRS} 
  ${localization_interfaces} 
  ${visualization_msgs_INCLUDE_DIRS}
  ) 

add_executable(hedge_rcv_bin 
src/hedge_rcv_bin.cpp 
src/marvelmind_hedge.c
)

target_link_libraries(hedge_rcv_bin ${localization_interfaces_LIB})
#rosidl_target_interfaces(hedge_rcv_bin ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(hedge_rcv_bin rclcpp std_msgs localization_interfaces nav_msgs geometry_msgs tf2_msgs) 

add_executable(subscriber_test 
src/subscriber_test.cpp
)

ament_target_dependencies(subscriber_test rclcpp std_msgs visualization_msgs localization_interfaces nav_msgs)

install(TARGETS 
  hedge_rcv_bin 
  subscriber_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  src
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
