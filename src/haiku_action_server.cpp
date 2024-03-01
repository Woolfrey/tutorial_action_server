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
using RequestManager = rclcpp_action::ServerGoalHandle<HaikuAction>;                                // For ease of use

/**
 * Forward declaration of goal request callback function.
 * @param uuid I have no idea what this does ¯\_(ツ)_/¯
 * @param request The number of lines of the poem to print/return.
 * @return REJECT if the request is < 1 line, otherwise ACCEPT_AND_EXECUTE
 */
rclcpp_action::GoalResponse process_request(const rclcpp_action::GoalUUID            &uuid,
                                            std::shared_ptr<const HaikuAction::Goal> request)
{
     (void)uuid;                                                                                    // This prevents colcon build from throwing a warning message
          
     if(request->number_of_lines < 1) return rclcpp_action::GoalResponse::REJECT;                   // Number of lines must be positive
     else                             return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * Forward declaration for cancelling of action.
 * @param manager This isn't used, but required by ROS2.
 * @return Reponse to the cancel request (in this case always ACCEPT)
 */
rclcpp_action::CancelResponse cancel_action(const std::shared_ptr<RequestManager> requestManager)
{
     (void)requestManager;                                                                          // Stops colcon build throwing a warning
     
     RCLCPP_INFO(rclcpp::get_logger("haiku_action_server"),"Received cancellation request.");       // Inform user
    
     return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * This function is called when a requested action is accepted.
 * @param manager An object that contains information on the HaikuAction data.
 */
void read_poem(const std::shared_ptr<RequestManager> requestManager)
{        
     RCLCPP_INFO(rclcpp::get_logger("haiku_action_server"), "Here is a haiku:");
     
     HaikuAction::Feedback::SharedPtr feedback;
     
     HaikuAction::Result::SharedPtr result;
     
     rclcpp::Rate loopRate(1);                                                                      // Keeps timing on 'for' loop
     
     int counter = 1;                                                                               // This keeps track of which line to add
     
     for(int i = 0; i < requestManager->get_goal()->number_of_lines && (rclcpp::ok()); i++)
     {      
          RCLCPP_INFO(rclcpp::get_logger("haiku_action_server"),"Line number %d", i);
          
          // Get the current line of the poem
          if(counter == 1)
          {
               // feedback->current_line = "Worker bees can leave.\n";
               counter++;
          }
          else if(counter == 2) 
          {
               // feedback->current_line = "Even drones can fly away.\n";
               counter++;
          }
          else // counter == 3
          {  
              // feedback->current_line = "The Queen is their slave.\n\n";
               counter = 1;
          }
          
          // feedback->line_number = i+1;                                                           // Current line of total
          
          // result->poem += feedback->current_line;                                                // Add the current line to the total
          
          // Check for cancellation
          if(requestManager->is_canceling())
          {
               requestManager->canceled(result);                                                    // Return total so far
               
               RCLCPP_INFO(rclcpp::get_logger("haiku_action_server"), "Haiku reading cancelled at line %d", i+1); // Inform the user
          }
          
          // requestManager->publish_feedback(feedback);                                            // As it says on the label
          
          loopRate.sleep();                                                                         // Wait for 1 second
     }
     
     // Finished
     if(rclcpp::ok())
     {
          // requestManager->succeed(result);
          
          RCLCPP_INFO(rclcpp::get_logger("haiku_action_server"), "Finished reading the haiku.");
     }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
     rclcpp::init(argc, argv);                                                                      // Start up ROS2 (if not already running)
     
     std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("haiku_action_server");         // Create node
       
     // Generate server from node
     rclcpp_action::Server<HaikuAction>::SharedPtr actionServer =
     rclcpp_action::create_server<HaikuAction>(node->get_node_base_interface(),
                                               node->get_node_clock_interface(),
                                               node->get_node_logging_interface(),
                                               node->get_node_waitables_interface(),
                                               "haiku_action_service",
                                               &process_request,
                                               &cancel_action,
                                               &read_poem);
     
     RCLCPP_INFO(node->get_logger(), "Ready to read you a poem ^_^");                               // Inform the user
     
     rclcpp::spin(node);                                                                            // Run the node indefinitely
     
     rclcpp::shutdown();                                                                            // Shut down the node
     
     return 0;                                                                                      // Exit main() with no issues
}
