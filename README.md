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
Implement code:
```
#include "tutorial_action_definition/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class FibonacciActionServer : public rclcpp::Node
{
     public:
          using Fibonacci  = tutorial_action_definition::action::Fibonacci;                         // Makes referencing easier
          
          using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;                            // A class for managing goals in an action server

          /**
           * Constructor.
           * An action server is created with the name 'fibonacci'.
           */
          FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
          : Node("fibonacci_action_server_node", options)
          {
               using namespace std::placeholders;                                                   // Needed for _1, _2 arguments below
               
               this->_actionServer
               = rclcpp_action::create_server<Fibonacci>(this,                                                         // Attach this node                            
                                                         "fibonacci",                                                  // Name to be advertised
                                                         std::bind(&FibonacciActionServer::request_action,this,_1,_2), // Link to callback function
                                                         std::bind(&FibonacciActionServer::cancel_action,this,_1),
                                                         std::bind(&FibonacciActionServer::accept_request,this,_1));
          }
     
     private:

          rclcpp_action::Server<Fibonacci>::SharedPtr _actionServer;                                // ROS2 server with Fibonacci as the action type

          /**
           * Processes the request to compute the Fibonacci sequence.
           * @param uuid This is used internally by ROS2.
           * @param request The specific input made by a client.
           * @return REJECT = 1 if invalid argument, ACCEPT_AND_EXECUTE = 2 otherwise.
           */
          rclcpp_action::GoalResponse request_action(const rclcpp_action::GoalUUID          &uuid,
                                                     std::shared_ptr<const Fibonacci::Goal> request)
          {
               if(request->order < 2)
               {
                    RCLCPP_INFO(this->get_logger(), "Please enter a number greater than 2.");
                    
                    (void)uuid;                                                                     // I don't know what this does

                    return rclcpp_action::GoalResponse::REJECT;
               }
               else
               {
                    std::string message = "Received request to compute the first "
                                        + std::to_string(request->order) +
                                        " elements of the Fibonacci sequence.";
                                        
                    RCLCPP_INFO(this->get_logger(), message.c_str());
                    
                    (void)uuid;                                                                     // colcon will raise a warning if we don't do something with the argument
                    
                    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
               }
          }
          
          /**
           * Processes cancellation.
           * @param actionManager This is not used, but is required by ROS2.
           * @return ACCEPT = 2
           */
          rclcpp_action::CancelResponse cancel_action(const std::shared_ptr<GoalHandle> actionManager)
          {
               RCLCPP_INFO(this->get_logger(), "Canceling computation of Fibonacci sequence.");
               
               (void)actionManager;                                                                 // colcon will raise a warning if we don't use the argument
               
               return rclcpp_action::CancelResponse::ACCEPT;                                        // Other possibility is REJECT
          }

          /**
           * This function is executed before the main control thread is called.
           * @param actionManager
           */
          void accept_request(const std::shared_ptr<GoalHandle> actionManager)
          {
               using namespace std::placeholders;                                                   // Provides the '_1' argument below
               
               std::thread{std::bind(&FibonacciActionServer::execute,this,_1), actionManager}.detach(); // 
          }
          
          /**
           * This is the main control thread.
           * @param actionManager 
           */
          void execute(const std::shared_ptr<GoalHandle> actionManager)
          {
               int numElements = actionManager->get_goal()->order;                                  // Makes referencing easier
               
               std::string message = "Computing the first " + std::to_string(numElements) + " "
                                     "elements of the Fibonacci sequence.";
                                     
               RCLCPP_INFO(this->get_logger(), message.c_str());                                    // Convert std::string to char*
               
               rclcpp::Rate loopRate(1);                                                            // Set a loop rate of 1Hz

               auto feedback = std::make_shared<Fibonacci::Feedback>();                             // ROS2 requires shared pointers
 
               feedback->partial_sequence.push_back(0);                                             // First number in Fibonacci sequence
               feedback->partial_sequence.push_back(1);                                             // Second number in Fibonacci sequence
               
               auto result = std::make_shared<Fibonacci::Result>();                                 // ROS2 requires shared pointers
               
               for(int i = 1; (i < numElements) && rclcpp::ok(); ++i)
               {
                    // Check for cancellation
                    if(actionManager->is_canceling())
                    {
                         result->sequence = feedback->partial_sequence;                             // Put partial sequence in result
                         
                         actionManager->canceled(result);                                           // Cancel with result
                         
                         RCLCPP_INFO(this->get_logger(), "Computation of Fibonacci sequence cancelled at element %d", i);
                    }
                    
                    feedback->partial_sequence.push_back(feedback->partial_sequence[i] +
                                                         feedback->partial_sequence[i-1]);          // Compute next Fibonacci number
                    
                    actionManager->publish_feedback(feedback);                                      // As it says
                    
                    RCLCPP_INFO(this->get_logger(), "Publishing Fibonacci sequence up to element %d", i);
                    
                    loopRate.sleep();                                                               // Wait up to the next second
               }
               
               // Check that objective is complete
               if(rclcpp::ok())
               {
                    result->sequence = feedback->partial_sequence;                              

                    actionManager->succeed(result);                                                 // Add result
                    
                    RCLCPP_INFO(this->get_logger(), "Finished computing Fibonacci sequence prematurely.");
               }
          }
};                                                                                                  // Semicolon required after class definition

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           MAIN                                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
     rclcpp::init(argc, argv);                                                                      // Start up ROS2
     
     auto actionServerNode = std::make_shared<FibonacciActionServer>();                             // Create node (as shared pointer)

     rclcpp::spin(actionServerNode);                                                                // Runs the node indefinitely

     rclcpp::shutdown();
     
     return 0;
}
```
:arrow_backward: [Go back.](#ros2-tutorial-32-creating-an-action-server--action-client)

### :mag: The Code Explained

:arrow_backward: [Go back.](#ros2-tutorial-32-creating-an-action-server--action-client)

## 2. Creating an Action Client

:arrow_backward: [Go back.](#ros2-tutorial-32-creating-an-action-server--action-client)

### :mag: The Code Explained

:arrow_backward: [Go back.](#ros2-tutorial-32-creating-an-action-server--action-client)
