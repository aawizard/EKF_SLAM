cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

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

rosidl_generate_interfaces(${PROJECT_NAME}_srv
  "srv/InitConfig.srv"
  "srv/Control.srv"
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")


find_package(rosidl_default_generators REQUIRED)
add_executable(turtle_control src/turtle_control.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(circle src/circle.cpp)

ament_target_dependencies(turtle_control rclcpp geometry_msgs turtlelib nuturtlebot_msgs sensor_msgs)
ament_target_dependencies(odometry rclcpp geometry_msgs turtlelib  nav_msgs tf2 tf2_ros sensor_msgs nav_msgs)
ament_target_dependencies(circle rclcpp geometry_msgs turtlelib std_srvs sensor_msgs)


target_link_libraries(turtle_control turtlelib::turtlelib ${cpp_typesupport_target})
target_link_libraries(odometry turtlelib::turtlelib ${cpp_typesupport_target})
target_link_libraries(circle turtlelib::turtlelib ${cpp_typesupport_target})


install(TARGETS
  turtle_control
  odometry
  circle
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
config
  launch
  srv
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)

  find_package(catch_ros2 REQUIRED)
  include(CTest)
  add_executable(turtle_control_test test/turtle_control_test.cpp)
  add_executable(turtle_odom_test_node test/turtle_odom_test_node.cpp)
  add_executable(turtle_circle_test test/circle_test.cpp)

  target_link_libraries(turtle_control_test catch_ros2::catch_ros2_with_node_main)
  target_link_libraries(turtle_odom_test_node catch_ros2::catch_ros2_with_node_main ${cpp_typesupport_target})
  target_link_libraries(turtle_circle_test catch_ros2::catch_ros2_with_node_main )

  ament_target_dependencies(turtle_control_test rclcpp geometry_msgs turtlelib nuturtlebot_msgs sensor_msgs)
  ament_target_dependencies(turtle_odom_test_node rclcpp nuturtlebot_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros turtlelib std_srvs)
  ament_target_dependencies(turtle_circle_test rclcpp geometry_msgs)
  install(TARGETS
    turtle_control_test
    turtle_odom_test_node
    turtle_circle_test
    DESTINATION lib/${PROJECT_NAME})




  install(DIRECTORY
    srv
    DESTINATION share/${PROJECT_NAME})
  install(FILES
    test/turtle_control_test.launch.xml
    test/turtle_odom_test.launch.xml
    test/turtle_circle_test.launch.xml
    DESTINATION share/${PROJECT_NAME}/test)

  catch_ros2_add_integration_test(turtle_control_test
    LAUNCH_FILE turtle_control_test.launch.xml
  )
  catch_ros2_add_integration_test(turtle_odom_test
    LAUNCH_FILE turtle_odom_test.launch.xml
  )

  catch_ros2_add_integration_test(turtle_circle_test
    LAUNCH_FILE turtle_circle_test.launch.xml
  )
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
