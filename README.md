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
v) Navigate back to the root of your ROS2 workspace and build the package:
```
colcon build --packages-select tutorial_action_server
```

:arrow_backward: [Go back.](#ros2-tutorial-32-creating-an-action-server--action-client)

### :mag: The Code Explained

An action server requires 3 functions:
1. A `GoalCallback` that will process an action request from a client,
2. A `CancelCallback` that will process a request to cancel an action being executed, and
3. An `AcceptedCallback` where the actual work of an action is executed.

This function below is the `GoalCallback`. When the action client makes a request to the server, it immediately invokes this function. If the request is invalid, it will `REJECT` and inform the client. Otherwise it immediately jumps to the `read_poem()` function below.
```
rclcpp_action::GoalResponse process_request(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const HaikuAction::Goal> request)
{
     (void)uuid;
          
     if(request->number_of_lines < 1) return rclcpp_action::GoalResponse::REJECT;
     else                             return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
```
This is the `CancelCallback` function. It could be programmed to check if its safe or feasible to cancel an action:
```
rclcpp_action::CancelResponse cancel_action(const std::shared_ptr<RequestManager> requestManager)
{
     (void)requestManager;
     
     RCLCPP_INFO(rclcpp::get_logger("haiku_action_server"),"Received cancellation request.");
    
     return rclcpp_action::CancelResponse::ACCEPT;
}
```
This is the `AcceptedCallback` that is invoked by the `GoalCallback`:
```
void read_poem(const std::shared_ptr<RequestManager> requestManager)
{        
     ...
}
```
It will loop through the total number of lines requested, and repeatedly add the 3 lines of the haiku together:
```
for(int i = 0; i < requestManager->get_goal()->number_of_lines && (rclcpp::ok()); i++)
{      
     ...
}
```
At a certain point with the above `for` loop it checks for a cancellation request:
```
if(requestManager->is_canceling())
{
     requestManager->canceled(result);
     
     RCLCPP_INFO(rclcpp::get_logger("haiku_action_server"), "Haiku reading cancelled at line %d", i+1);
}
```
On this line it publishes the feedback data to a hidden topic:
```
requestManager->publish_feedback(feedback);
```
We need to call `ros2 topic list --include-hidden-topics` in order to see it.

This line of code keeps the timing on the `for` loop to 1Hz:
```
loopRate.sleep();
```
In the `main()` function we create a node:
```
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("haiku_action_server");
```
and use it to generate the action server:
```
rclcpp_action::Server<HaikuAction>::SharedPtr actionServer =
rclcpp_action::create_server<HaikuAction>(node->get_node_base_interface(),
                                          node->get_node_clock_interface(),
                                          node->get_node_logging_interface(),
                                          node->get_node_waitables_interface(),
                                          "haiku_action_service",
                                          &process_request,
                                          &cancel_action,
                                          &read_poem);
```
The 5th argument is the name of the service "haiku_action_server" that any related client must match. 

The last 3 arguments are the `GoalCallback`, `CancelCallback`, and `AcceptedCallback` functions, respectively.

This line runs the node which will look for action requests and execute them:
```
rclcpp::spin(node);
```

:arrow_backward: [Go back.](#ros2-tutorial-32-creating-an-action-server--action-client)

## 2. Creating an Action Client

i) In the `tutorial_action_server/src` folder, create `haiku_action_client.cpp` and enter the following code:
```
#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 action libaries
#include <tutorial_action_definition/action/haiku.hpp>                                              // Custom action built in another project

// Structure of Haiku.action:
//
// # Goal
// int32 number_of_lines
// ---
// # Result
// string poem
// ---
// # Feedback
// int32 line_number
// string current_line
     
using HaikuAction = tutorial_action_definition::action::Haiku;                                      // Makes referencing easier

using RequestManager = rclcpp_action::ClientGoalHandle<HaikuAction>;                                // For ease of use

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           MAIN                                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
     rclcpp::init(argc, argv);                                                                      // Start up ROS2
     
     if(argc != 2)
     {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Incorrect number of arguments. Usage: "
                                                     "'ros2 run tutoral_action_server haiku_action_client n' "
                                                     "where n is the number of lines to print.");
                                                      
          return -1;
     }
     
     std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("haiku_action_client");         // Create node
     
     rclcpp_action::Client<HaikuAction>::SharedPtr client =
     rclcpp_action::create_client<HaikuAction>(node,"haiku_action_service");                        // Create client to interface with named service
     
     // Wait for the server to appear; if not, shut down
     if(not client->wait_for_action_server(std::chrono::seconds(5)))
     {
          RCLCPP_ERROR(node->get_logger(), "Waited 5 seconds for the server and found nothing.");
          
          return -1;                                                                                // Shut down with error
     }
     
     RCLCPP_INFO(node->get_logger(), "Requesting a poem from the server.");
     
     // Create and send the request to the server
     
     HaikuAction::Goal request; request.number_of_lines = std::stoi(argv[1]);                        // Create request
     
     std::shared_future<RequestManager::SharedPtr> requestManager = client->async_send_goal(request); // This class is used for processing the request to the server
      
     if(rclcpp::spin_until_future_complete(node, requestManager) != rclcpp::FutureReturnCode::SUCCESS) // Check to see if successfully sent
     {
          RCLCPP_ERROR(node->get_logger(), "Failed to send request.");
          
          return -1;
     }    
     else if(not requestManager.get())                                                              // Check to see if request is accepted
     {
          RCLCPP_ERROR(node->get_logger(), "Request was rejected by server.");
          
          return -1;
     }

     // Wait for the result
     
     auto responseManager = client->async_get_result(requestManager.get());                         // This class contains information on the response from the server                    
      
     if(rclcpp::spin_until_future_complete(node,responseManager) != rclcpp::FutureReturnCode::SUCCESS) // I don't really know what this does
     {
          RCLCPP_ERROR(node->get_logger(), "Failed to obtain response.");
          
          return -1;
     }
        
     auto response = responseManager.get();                                                         // As it says on the label
      
     if(response.code == rclcpp_action::ResultCode::SUCCEEDED)
     {
          std::string message = "Here is the poem:\n" + response.result->poem;
          RCLCPP_INFO(node->get_logger(), message.c_str());
     }
     else
     {
          RCLCPP_ERROR(node->get_logger(), "There was an error with the Haiku server.");
     }
    
     rclcpp::shutdown();                                                                            // Stop ROS2
}
```
ii) Modify the `CMakeLists.text` file in the root directory of `tutorial_action_server` with the following lines:
```
add_executable(haiku_action_client src/haiku_action_client.cpp)
ament_target_dependencies(haiku_action_client
                          "rclcpp"
                          "rclcpp_action"
                          "tutorial_action_definition")
```
Be sure to also add it to the install targets so ROS2 can find it as a package:
```
install(TARGETS
        haiku_action_server
        haiku_action_client
        DESTINATION lib/${PROJECT_NAME})
```
iii) Navigate back to the root of your ROS2 workspace and build the package:
```
colcon build --packages-select tutorial_action_server
```
iv) Be sure to locally source after building:
```
source ./install/setup.bash
```
v) Launch the action server (if you haven't already done so):
```
ros2 run tutorial_action_server haiku_action_server
```
vi) Launch the action client in another window:
```
ros2 run tutorial_action_server haiku_action_client n
```
Where `n` is an integer argument for the number of lines to print. You should see something like:

<img src="https://github.com/Woolfrey/tutorial_action_server/assets/62581255/51a66d62-352b-4d3a-9848-afa0555bb0bb" width="800" height="auto">

Once completed the server will return the poem with the number of lines requested. The client will then print this to the terminal as shown.

vii) In a _third_ terminal, you can check the output of the hidden feedback topic:
```
ros2 topic list --include-hidden-topics
```
<img src="https://github.com/Woolfrey/tutorial_action_server/assets/62581255/7bd8174f-d73d-4ccf-8bbf-af81d89f387e" width="800" height="auto">

It is possible to echo this topic as the actions server is running:

```
ros2 topic echo /haiku_action_service/_action/feedback
```
And you should see:

<img src="https://github.com/Woolfrey/tutorial_action_server/assets/62581255/f623a396-f87a-4566-8306-1a4fe2be1aac" width="800" height="auto">

Remember that you have to source: `source ./install/setup.bash` otherwise ROS2 will not recognise the message/action type.

:arrow_backward: [Go back.](#ros2-tutorial-32-creating-an-action-server--action-client)

### :mag: The Code Explained

Here we create the node and give it a name:
```
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("haiku_action_client");
```
It needs to be a shared pointer so we can spin it later.

In this line we create the client:
```
rclcpp_action::Client<HaikuAction>::SharedPtr client = rclcpp_action::create_client<HaikuAction>(node,"haiku_action_service");
```
The name "haiku_action_service" must match what is advertised by the server.

Here the client will wait 5 seconds to give time for the server to appear (if it doesn't already exist):
```
if(not client->wait_for_action_server(std::chrono::seconds(5)))
{
     RCLCPP_ERROR(node->get_logger(), "Waited 5 seconds for the server and found nothing.");
     
     return -1;
}
```
Once it times out, it will shut down.

In these lines of code we create the request from the argument when we launched the client. Then we send it to the server:
```
HaikuAction::Goal request; request.number_of_lines = std::stoi(argv[1]);
std::shared_future<RequestManager::SharedPtr> requestManager = client->async_send_goal(request);
```

This line simply checks if the request was sent successfully:
```
if(rclcpp::spin_until_future_complete(node, requestManager) != rclcpp::FutureReturnCode::SUCCESS)
{
     RCLCPP_ERROR(node->get_logger(), "Failed to send request.");
     
     return -1;
}    
else if(not requestManager.get())
{
     RCLCPP_ERROR(node->get_logger(), "Request was rejected by server.");
     
     return -1;
}
```

Here we wait for the action to be completed:
```
auto responseManager = client->async_get_result(requestManager.get());
 
if(rclcpp::spin_until_future_complete(node,responseManager) != rclcpp::FutureReturnCode::SUCCESS)
{
     RCLCPP_ERROR(node->get_logger(), "Failed to obtain response.");
     
     return -1;
}
```

Finally, we get the response and print to to the terminal (if successful):
```
auto response = responseManager.get();
 
if(response.code == rclcpp_action::ResultCode::SUCCEEDED)
{
     std::string message = "Here is the poem:\n" + response.result->poem;
     RCLCPP_INFO(node->get_logger(), message.c_str());
}
else
{
     RCLCPP_ERROR(node->get_logger(), "There was an error with the Haiku server.");
}
```


:arrow_backward: [Go back.](#ros2-tutorial-32-creating-an-action-server--action-client)
