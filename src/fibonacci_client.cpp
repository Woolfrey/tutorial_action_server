#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tutorial_action_definition/action/fibonacci.hpp>

using Fibonacci = tutorial_action_definition::action::Fibonacci;                                    // Makes referencing easier

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                             MAIN                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
     rclcpp::init(argc, argv);                                                                      // Start up ROS2
     
     auto node = rclcpp::Node::make_shared("fibonacci_action_client_node");                         // Create nodes
     
     auto client = rclcpp_action::create_client<Fibonacci>(node,"fibonacci");                       // Create client and attach node
     
     // Wait for the server to appear; if not, shut down
     if(not client->wait_for_action_server(std::chrono::seconds(5)))
     {
          RCLCPP_ERROR(node->get_logger(), "Waited 5 seconds for the server and found nothing.");
          
          return -1;                                                                                // Shut down with error
     }
     
     // Create goal message
     auto goal = Fibonacci::Goal();
     goal.order = 5;
     
     // Send goal and wait for result
     auto sendGoalFuture = client->async_send_goal(goal);
     
     if(rclcpp::spin_until_future_complete(node, sendGoalFuture) != rclcpp::FutureReturnCode::SUCCESS)
     {
          RCLCPP_ERROR(node->get_logger(), "Failed to send goal.");
          
          return -1;
     }
     
     auto goalHandle = sendGoalFuture.get();
     if(not goalHandle)
     {
          RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server.");
          
          return -1;
     }

     // Wait for the result
     auto resultFuture = client->async_get_result(goalHandle);
     if(rclcpp::spin_until_future_complete(node,resultFuture) != rclcpp::FutureReturnCode::SUCCESS)
     {
          RCLCPP_ERROR(node->get_logger(), "Failed to get result.");
          
          return -1;
     }
     
     auto result = resultFuture.get();
     if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
     {
          std::string sequence = "Result: ";
          for(auto element : result.result->sequence) sequence += " " + std::to_string(element);
          
          RCLCPP_INFO(node->get_logger(), sequence.c_str());
     }
     else
     {
          RCLCPP_ERROR(node->get_logger(), "Action did not succeed.");
     }
     
     rclcpp::shutdown();
     
     return 0;
}
