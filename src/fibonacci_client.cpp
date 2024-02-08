#include <iostream>

#include "tutorial_action_definition/action/fibonacci.hpp"                                          // Previously compiled package
#include "rclcpp/rclcpp.hpp"                                                                        // 
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class FibonacciActionClient : public rclcpp::Node
{
     using Fibonacci = tutorial_action_definition::action::Fibonacci;                               // Makes referencing easier
     
     using GoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>                                  // Makes referencing easier
     
     /**
      * Constructor.
      */
     FibonacciActionClient(const rclcpp::NodeOptions &options)
     : Node("fibonacci_action_client", options)
     {
          this->_client = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");               // Bind this ROS node to client server named 'fibonacci'
          
   
          this->_timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                                 std::bind(&FibonacciActionClient::request_sequence,this));
     }
     
     /**
      * Request that the server computes the Fibonacci sequence up to a specified number.
      */
     void request_sequence()
     {
          using namespace std::placeholders;                                                        // For arguments _1, _2 used below
    
          this->_timer->cancel();                                                                   // Stop?
          
          // If server does not appear, shut down
          if(not this->_client->wait_for_action_server())
          {
               RCLCPP_ERROR(this->get_logger(), "Waited for action server but it did not appear."); //
               
               rclcpp::shutdown();                                                                  // Shut down this node
          }
          
          // Get input from user
          auto input = Fibonacci::Goal();
          
          std::cout << "Enter an integer:" << std::endl;
 
          bool OK = false;
          while(not OK)
          {
               std::cin  >> input.order;
               std::cout << "Is " << input.order << " correct? Y/n" << std::endl;
               std::string response;
               std::cin >> response;
               
               if(response == "Y") OK == true;
          }
          
          RCLCPP_INFO(this->get_logger(), "Sending request to Fibonacci server.");
          
          auto sendOptions = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
          
          // Link callback functions
          sendOptions.goal_response_callback = std::bind(&FibonacciActionClient::process_response,this,_1);
          sendOptions.feedback_callback      = std::bind(&FibonacciActionClient::receive_feedback,this,_1,_2);
          sendOptions.result_callback        = std::bind(&FibonacciActionClient::check_result,this,_1);
          
          this->_client->async_send_goal(input, sendOptions);                                       // Send
     }
     
     private:
          
          rclcpp_action::Client<Fibonacci>::SharedPtr _client;                                      // Client class for Fibonacci action             
          
          rclcpp::TimeBase::SharedPtr _timer;                                                       // Stopwatch used to delay startup
          
          /**
           * Check the response from the action request.
           */
           void process_response(std::shared_future<GoalHandle::SharedPtr> response)
           {
               auto responseManager = response.get();
               
               if(not responseManager) RCLCPP_ERROR(this->get_logger(), "Request was rejected by the server.");
               else                    RCLCPP_INFO( this->get_logger(), "Request accepted by the server. Waiting for the result.");
           }
           
           /**
            * Prints out the intermediate sequences of Fibonacci numbers
            * @param SharedPtr I don't know what this does ┐(ﾟ ～ﾟ )┌
            * @param feedback Updates from the server as it executes the action
            */
           void receive_feedback(GoalHandle::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback> feedback)
           {
               std::string sequence = "Next number in the sequence is: ";
         
               for(auto number : feedback->partial_sequence) + std::to_string(number) + " ";        // Add the numbers to the string
               
               RCLCPP_INFO(this->get_logger(), sequence.c_str());                                   // Convert to char* and print
           }
           
           /**
            * Determines the result of the action and prints the appropriate response.
            * @response The final message received from the action server.
            */
           void check_result(const GoalHandle::WrappedResult &result)
           {
               switch(result.code)
               {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                         std::string message = "The result is: ";
                         for(auto number : result.result->sequence) message += std::to_string(number) + " ";
                         RCLCPP_INFO(this->get_logger(), message.c_str());                          // Conver to char* and print
                         break;
                         
                    case rclcpp_action::ResultCode::ABORTED:
                         RCLCPP_ERROR(this->get_logger(), "Action aborted");
                         break;
                    case rclcpp_action::ResultCode::CANCELED:
                         RCLCPP_ERROR(this->get_logger(), "Action canceled");
                         break;
                    default:
                         RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                         break;
               }
               
               rclcpp::shutdown();
           }
};                                                                                                  // Semicolon needed after class declaration
          
