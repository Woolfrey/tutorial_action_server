//#include <functional>
//#include <future>
//#include <memory>
//#include <string>
//#include <sstream>

#include "tutorial_action_definition/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class FibonacciServer : rclcpp::Node
{
     public:
          using Fibonacci  = tutorial_action_definition::action::Fibonacci;                         // Makes referencing easier
          
          using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;                            // A class for managing goals in an action server

          /**
           * Constructor.
           */
          FibonacciServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
          : Node("action_server_node", options)
          {
          
          }
     
     private:

//          rclcpp_action::Server<Fibonacci> _actionServer;                                           // ROS2 server with Fibonacci as the action type

          /*
           * Function for requesting Fibonacci calculation.
           * @param goal An integer specifying which number in the sequence.
           *
          rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                  const Fibonacci::Goal &goal)
          {
               RCLCPP_INFO(this->get_logger(), "Requested to compute Fibonacci sequence up to order %d", goal->order); // Inform user

               (void)uuid;                                                                          // I don't know what this does ¯\_(ツ)_/¯

               return rclcpp::ACCEPT_AND_EXECUTE;
          }

           *
           * Function for cancelling an action under execution.
           * @goalHandle
           *
          rclcpp_action::CancelResponse handle_cancel(const GoalHandle &goalHandle)
          {
               RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");                 // Inform user

               (void)goal_handle;                                                                   // I don't know what this does ¯\_(ツ)_/¯

               return rclcpp_action::CancelResponse::ACCEPT;
          }

          
           * Function for when a goal is accepted for execution.
           *
           *
          void handle_accepted(const GoalHandle &goalHandle)
          {
               std::thread{std::bind(&FibonacciServer::execute, this, _1), goalHandle}.detach();    // Link to execute() function below
          }

          
           * Principle control thread which computes the Fibonacci sequence.
           *
          void execute(const GoalHandle &goalHandle)
          {
               RCLCPP_INFO(this->get_logger(), "Executing goal.");                                  // Inform user

               rclcpp::Rate loopRate(1);                                                            // Set a frequency of 1Hz for this thread

               int goal = goalHandle->get_goal();                                                   // Obtain the goal

               auto feedback = Fibonacci::Feedback;

               int sequence[] = feedback.partial_sequence;

               sequence.push_back(0);

               sequence.push_back(1);

               auto result = Fibonacci::Result;

               for(int i = 1; i < goal.order) && rclcpp::ok(); i++)
               {
                    // Check for cancellation
                    if(goalHandle.is_cancelling())
                    {
                         result.sequence = sequence;                                                // Put the current sequence as the result

                         goalHandle.canceled(result);                                               // Add to goal handle

                         RCLCPP_INFO(this->get_logger(), "Goal cancelled.");                        // Inform user

                         return;                                                                    // Exit the thread
                    }

                    sequence.push_back(sequence[i] + sequence[i-1]);                                // Compute next number in sequence
   
                    goalHandle.publish_feedback(feedback);                                          // As it says

                    RCLCPP_INFO(this->get_logger(), "Publishing feedback.");                        // Inform user

                    loopRate.sleep();                                                               // Synchronize with the control frequency
               }

               // Check if the objective has been achieved
               if(rclcpp::ok())
               {
                    result.sequence = sequence;

                    goalHandle.succeed(result);

                    RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
               }
          }*/
};                                                                                                  // Semicolon required after class definition


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           MAIN                                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
/*
     rclcpp::init(argc, argv);

     auto action_server = std::make_shared<ActionServer>();

     rclcpp::spin(action_server);

     rclcpp::shutdown();*/
     
     return 0;
}
