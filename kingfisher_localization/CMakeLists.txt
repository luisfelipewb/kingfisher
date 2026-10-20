cmake_minimum_required(VERSION 3.8)
project(kingfisher_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sbg_driver REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# Debian's libgeographiclib-dev ships a find module only
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

# sbg/* -> IMU, GPS fix, odom and odom -> base_link
add_executable(sbg_localization src/sbg_localization.cpp)
target_compile_features(sbg_localization PUBLIC cxx_std_17)
target_include_directories(sbg_localization PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(sbg_localization
  rclcpp::rclcpp
  ${sbg_driver_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  yaml-cpp::yaml-cpp
  ${GeographicLib_LIBRARIES})
install(TARGETS sbg_localization
  DESTINATION lib/${PROJECT_NAME})

# geo.py and geo_tf_publisher (Python)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/geo_tf_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
