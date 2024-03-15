cmake_minimum_required(VERSION 3.8)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nusim REQUIRED)
find_package(nuturtle_description REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nuturtle_control REQUIRED)
find_package(Armadillo REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include ${ARMADILLO_INCLUDE_DIRS})

add_executable(slam src/slam.cpp)
add_executable(landmark src/landmark.cpp)
ament_target_dependencies(slam rclcpp
  geometry_msgs
  turtlelib
  nav_msgs
  tf2
  tf2_ros
  sensor_msgs
  nav_msgs
  nuturtle_control
  nuturtlebot_msgs
  visualization_msgs)
ament_target_dependencies(landmark
  rclcpp
  sensor_msgs
  turtlelib
  visualization_msgs)
target_link_libraries(slam turtlelib::turtlelib ${cpp_typesupport_target} ${ARMADILLO_LIBRARIES})
target_link_libraries(landmark turtlelib::turtlelib ${cpp_typesupport_target} ${ARMADILLO_LIBRARIES})




install(TARGETS
  slam
  landmark
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
config
  launch
  DESTINATION share/${PROJECT_NAME}/
)
if( NOT CMAKE_CROSSCOMPILING)
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
endif()

find_package(Doxygen)

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" ON)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()



ament_package()
