cmake_minimum_required(VERSION 3.5)
project(romea_localisation_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_core_filtering REQUIRED)
find_package(romea_core_localisation REQUIRED)
find_package(romea_common_msgs REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(romea_localisation_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/conversions/observation_angular_speed_conversions.cpp
  src/conversions/observation_attitude_conversions.cpp
  src/conversions/observation_course_conversions.cpp
  src/conversions/observation_linear_speed_conversions.cpp
  src/conversions/observation_linear_speeds_conversions.cpp
  src/conversions/observation_pose_conversions.cpp
  src/conversions/observation_position_conversions.cpp
  src/conversions/observation_range_conversions.cpp
  src/conversions/observation_twist_conversions.cpp
  src/filter/parameters.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  romea_core_common
  romea_core_filtering
  romea_core_localisation
  std_msgs
  romea_common_msgs
  romea_common_utils
  romea_localisation_msgs)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(romea_core_common)
ament_export_dependencies(romea_core_filtering)
ament_export_dependencies(romea_core_localisation)
ament_export_dependencies(romea_common_msgs)
ament_export_dependencies(romea_common_utils)
ament_export_dependencies(romea_localisation_msgs)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
#ament_export_targets(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

option(BUILD_TESTING "BUILD WITH TESTS" ON)

if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
    ament_lint_auto_find_test_dependencies()
    add_subdirectory(test)

endif()

ament_package()
