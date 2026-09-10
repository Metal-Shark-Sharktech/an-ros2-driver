cmake_minimum_required(VERSION 3.5)
set(CMAKE_BUILD_TYPE release)
project(adnav_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Avoid Confusing max() and min() for Windows Macros. See rclcpp issue #1054
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(adnav_interfaces REQUIRED)

# Build
include_directories(
  include/adnav_driver
  ../an-ros-common/include
)

aux_source_directory(src src_MAIN)
aux_source_directory(../an-ros-common/src src_COMMON)

add_library(${PROJECT_NAME}_lib
  ${src_MAIN}
  ${src_COMMON}
)

# Modern CMake targets. ament_target_dependencies() was removed in the
# ament_cmake shipped with ROS 2 Lyrical, so link these directly.
set(DEPENDENCIES
  rclcpp::rclcpp
  tf2::tf2
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${adnav_interfaces_TARGETS}
)

set(EXECUTABLE_NAME "adnav_driver")

target_link_libraries(${PROJECT_NAME}_lib ${DEPENDENCIES})

add_executable(${EXECUTABLE_NAME} src/main.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib ${DEPENDENCIES})

# add_executable(srv_test src/srv_test.cpp)
# target_link_libraries(srv_test ${PROJECT_NAME}_lib)
# target_link_libraries(srv_test ${DEPENDENCIES})

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS srv_test
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  ament_cpplint()
endif()

ament_export_include_directories(include)
ament_package()