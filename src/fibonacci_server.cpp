//#include <functional>
//#include <future>
//#include <memory>
//#include <string>
//#include <sstream>

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
           */
          FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
          : Node("fibonacci_action_server_node", options)
          {
               using namespace std::placeholders;                                                   // Needed for _1, _2 arguments below
               
               this->_actionServer = rclcpp_action::create_server<Fibonacci>(this,
                                                                             "fibonacci",
                                                                             std::bind(&FibonacciActionServer::request_action, this, _1, _2),
                                                                             std::bind(&FibonacciActionServer::cancel_action,  this, _1),
                                                                             std::bind(&FibonacciActionServer::accept_request, this, _1));
          }
     
     private:

          rclcpp_action::Server<Fibonacci>::SharedPtr _actionServer;                                // ROS2 server with Fibonacci as the action type

          rclcpp_action::GoalResponse request_action(const rclcpp_action::GoalUUID          &uuid,
                                                     std::shared_ptr<const Fibonacci::Goal> goal)
          {
               return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          }
          
          rclcpp_action::CancelResponse cancel_action(const std::shared_ptr<GoalHandle> goal_handle)
          {
               return rclcpp_action::CancelResponse::ACCEPT;
          }
          
          void accept_request(const std::shared_ptr<GoalHandle> goal_handle)
          {
               // Worker bees can leave.
               // Even drones can fly away.
               // The Queen is their slave.
          }
          
          void execute(const std::shared_ptr<GoalHandle> goal_handle)
          {
          
          }
};                                                                                                  // Semicolon required after class definition


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           MAIN                                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
     rclcpp::init(argc, argv);                                                                      // Start up ROS2
     
     auto actionServerNode = std::make_shared<FibonacciActionServer>();                             // Create node (as shared pointer)

     rclcpp::spin(actionServerNode);

     rclcpp::shutdown();
     
     return 0;
}
