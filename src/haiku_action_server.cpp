#include <rclcpp/rclcpp.hpp>                                                                        // ROS2 C++ libraries
#include <rclcpp_action/rclcpp_action.hpp>                                                          // ROS2 action libaries
#include <tutorial_action_definition/action/haiku.hpp>                                              // Custom action built in another project

// NOTE: This style of programming is no longer the standard for ROS2, and is only intended to
//       improve comprehension. The new ROS2 paradigm is to use object oriented programming.
std::shared_ptr<rclcpp::Node> node = nullptr;                                                       // Forward declaration so it is in scope of main() and callback functions             



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
     
     if(request->number_of_lines < 1) return rclcpp_action::GoalResponse::REJECT;                   // Number of lines must be positive
     else                             return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * Forward declaration for cancelling of action.
 * @param manager This isn't used, but required by ROS2.
 * @return Reponse to the cancel request (in this case always ACCEPT)
 */
rclcpp_action::CancelResponse cancel_action(const std::shared_ptr<RequestManager> manager)
{
     (void)manager;                                                                                 // Stops colcon build throwing a warning
     
     RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Received cancellation request.");                    // Inform user
     
     return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * This function is called when a requested action is accepted.
 * @param manager An object that contains information on the HaikuAction data.
 */
void accept_request(const std::shared_ptr<RequestManager> manager)
{   
     // using namespace std::placeholders;
     // std::thread{std::bind(&ActionServer::execute, this,_1), manager}.detach();
     
     (void)manager;
      
     rclcpp::Rate loopRate(2.0);
     
     int count = 1;
     
     while(rclcpp::ok())
     {
          if(count == 1)
          {
               std::cout << "Worker bees can leave.\n";
               count++;
          }
          else if(count == 2)
          {
               std::cout << "Even drones can fly away.\n";
               count++; 
          }
          else
          {
               std::cout << "The Queen is their slave.\n\n";
               count = 1;
          }
     }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
     rclcpp::init(argc, argv);   
     
     node = rclcpp::Node::make_shared("haiku_action_server");                                       // Assign node to empty memory
     
     
     rclcpp_action::Server<HaikuAction>::SharedPtr actionServer =
     rclcpp_action::create_server<HaikuAction>(node->get_node_base_interface(),
                                               node->get_node_clock_interface(),
                                               node->get_node_logging_interface(),
                                               node->get_node_waitables_interface(),
                                               "haiku_action_service",
                                               &process_request,
                                               &cancel_action,
                                               &accept_request);
     
     RCLCPP_INFO(node->get_logger(), "Ready to read you a poem ^_^");
     
     rclcpp::spin(node);
     
     rclcpp::shutdown();
     
     return 0;                                                                                      // Exit main() with no issues
}
