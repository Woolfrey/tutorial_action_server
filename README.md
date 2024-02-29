# ROS2 Tutorial 3.2 Creating an Action Server & Action Client

This is Part 3 in a series of ROS2 Tutorials:
1. [Publishers & Subscribers](https://github.com/Woolfrey/tutorial_publisher_subscriber)
2. Services
     1. [Defining a Service](https://github.com/Woolfrey/tutorial_service_definition)
     2. [Creating a Service & Client](https://github.com/Woolfrey/tutorial_service_client)
4. Actions
     1. [Defining an Action](https://github.com/Woolfrey/tutorial_action_definition)
     2. [Creating an Action Server & Client](https://github.com/Woolfrey/tutorial_action_server)
        
## Contents
- [What Are They?](#what-are-they)
- [1. Creating an Action Server](#1-creating-an-action-server)
- [2. Creating an Action Client](#2-creating-an-action-client)

## What are They?

Actions are one of 3 communication protocols in ROS2:

| Sender | Receiver | Node Interaction | Periodicity |
|--------|----------|------------------|-------------|
| Publisher | Subscriber | Indirect | Continuous |
| Server | Client | Direct | By request |
| Action Server | Action Client | Direct | By request, with continuous updates. |

- In the `publisher` & `subscriber` paradigm, a single publisher streams information to a topic. Multiple subscribers may then connect to this topic and use this information as they please. A publisher is suitable for streaming information, like sensor data on a robot.

<img src="https://github.com/Woolfrey/tutorial_action_definition/assets/62581255/8fb65629-6dd1-46ac-9d85-2a4eb794a16d" alt="image" width="300" height="auto">

- A `server` processes a one-time request from a `client`. This is suited to information that is required sporadically. For example, retrieving an update of a map, or generating a new path to a desired location. The information is passed directly between nodes that others cannot access.
- The `action` protocol amalgmates the concept of the publisher & subscriber with that of the client & server. An `action client` makes a request to an `action server`. Whilst processing this request, the action server publishes information on its progress.

<img src="https://github.com/Woolfrey/tutorial_action_definition/assets/62581255/3f2a0cd3-7664-4d56-bcc4-d6f057790604" alt="image" width = "300" height="auto">

Actions are suitable for structured tasks, for example telling a robot to drive to a particular location. It is a finite task that is executed infrequently, hence the server/client feature. But we may want to receive continual updates on its progress (time until completion, tracking accuracy, etc.), hence the publisher/subscriber aspect.

<img src="https://github.com/Woolfrey/tutorial_action_definition/assets/62581255/6aad8ead-929f-4a87-8bb2-d521f0825d33" alt="image" width="300" height="auto">

An `Action.action` file is structured as follows:
```
# Goal
datatype goal
---
# Result
datatype result
---
# Feedback
datatype feedback
```

:arrow_backward: [Go back.](#ros2-tutorial-32-creating-an-action-server--action-client)

## 1. Creating an Action Server

:rotating_light: This assumes you have completed & compiled the action definition in the [previous tutorial](https://github.com/woolfrey/tutorial_action_definition) :rotating_light:

Be sure to source ROS2 and your local ROS2 working directory so that the `tutorial_action_definition` package can be found:
```
source /opt/ros/<distribution>/setup.bash
source ./install/setup.bash
```
Substitute `<distribution>` with your version of ROS2 (foxy, humble, etc.).

i) First navigate to `<your_ros_workspace>/src` and create a new package:
```
ros2 pkg create --dependencies tutorial_action_definition rclcpp rclcpp_action rclcpp_components -- tutorial_action_server
```
This package has the following dependencies:
- `tutorial_action_definition` is the previous package we created.
- `rclcpp` is the ROS2 C++ libraries,
- `rclcpp_action` is used for the ROS2 Action application programming interface (API),
- `rclcpp_components` I don't know what this does ¯\_(ツ)_/¯
ii) Now navigate in to the package and create a new `src` directory (if it doesn't already exist):
```
cd tutorial_action_server
mkdir src && cd src
```
iii) Inside the `src` folder, create a file called `haiku_action_server.cpp` and insert the following code:
```
TO DO.
```
iv) Navigate back to `/tutorial_action_server` and modify the `CMakeLists.txt` package with the following before the `ament_package()` line:
```
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
```
:bangbang: Also check the following lines of code are at the top of the file:
```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tutorial_action_definition REQUIRED)
```
iv) Navigate back to the root of your ROS2 workspace and build the package:
```
colcon build --packages-select tutorial_action_server
```

:arrow_backward: [Go back.](#ros2-tutorial-32-creating-an-action-server--action-client)

### :mag: The Code Explained

:arrow_backward: [Go back.](#ros2-tutorial-32-creating-an-action-server--action-client)

## 2. Creating an Action Client

:arrow_backward: [Go back.](#ros2-tutorial-32-creating-an-action-server--action-client)

### :mag: The Code Explained

:arrow_backward: [Go back.](#ros2-tutorial-32-creating-an-action-server--action-client)
