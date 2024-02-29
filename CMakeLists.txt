
cmake_minimum_required(VERSION 3.8)
project(tutorial_action_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################################################################################################
#                                             Dependencies                                         #
####################################################################################################
find_package(ament_cmake REQUIRED)                                                                  # Needed to compile
find_package(rclcpp REQUIRED)                                                                       # ROS2 C++ libraries
find_package(rclcpp_action REQUIRED)                                                                # ROS2 action libraries
find_package(rclcpp_components REQUIRED)                                                            # ROS2 node libraries
find_package(tutorial_action_definition REQUIRED)                                                   # Previously built package

if(BUILD_TESTING)
     find_package(ament_lint_auto REQUIRED)
     set(ament_cmake_copyright_FOUND TRUE)
     set(ament_cmake_cpplint_FOUND TRUE)
     ament_lint_auto_find_test_dependencies()
endif()

####################################################################################################
#                                            Executables                                           #
####################################################################################################
add_executable(haiku_action_server src/haiku_action_server.cpp)
ament_target_dependencies(haiku_action_server
                          "rclcpp"
                          "rclcpp_action"
                          "rclcpp_components"
                          "tutorial_action_definition")
   
# This is so ROS2 can find the package                                               
install(TARGETS
        haiku_action_server
        DESTINATION lib/${PROJECT_NAME})

ament_package()
