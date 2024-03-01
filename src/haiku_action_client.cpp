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
          RCLCPP_INFO(node->get_logger(), response.result->poem.c_str());
     }
     else
     {
          RCLCPP_ERROR(node->get_logger(), "There was an error with the Haiku server.");
     }
    
     rclcpp::shutdown();                                                                            // Stop ROS2
}
