# tutorial_action_server
Demonstrates the implentation of an action server and client in ROS2.

Create package:
```
ros2 pkg create --dependencies tutorial_action_definition rclcpp rclcpp_action rclcpp_components -- tutorial_action_server
```
Dependencies:
- `tutorial_action_definition` is the previous package we created.
- `rclcpp` is the ROS2 C++ client library,
- `rclcpp_action` is used for the ROS2 Action application programming interface (API),
- `rclcpp_components` ROS2 nodes?

```
mkdir src && cd src
```
then:
```
gedit fibonacci_server.cpp
```
