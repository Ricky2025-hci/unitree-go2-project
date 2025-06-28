#include <unistd.h>  // Provides access to the POSIX API (not used directly here)
#include <cmath>     // Provides mathematical constants and functions like M_PI

#include "rclcpp/rclcpp.hpp"  // ROS 2 C++ client library
#include "unitree_api/msg/request.hpp"  // Message definition for Unitree robot movement requests
#include "common/ros2_sport_client.h"   // Helper class for generating movement requests (SportClient)

using std::placeholders::_1;  // Placeholder used with std::bind for callback functions

// Define a ROS 2 node class named 'soprt_request'
class soprt_request : public rclcpp::Node
{
public:
    // Constructor: initialize the node and set up publisher and timer
    soprt_request() : Node("req_sender")
    {
        // Create a publisher that sends messages to "/api/sport/request"
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        // Create a timer that triggers the timer_callback function every dt seconds
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(int(dt * 1000)),  // Convert seconds to milliseconds
            std::bind(&soprt_request::timer_callback, this));  // Bind the callback function
    }

private:
    // Enum to represent the movement state of the robot
    enum class State
    {
        MOVE1,    // First forward movement
        TURN1,    // First turn
        MOVE2,    // Second forward movement
        TURN2,    // Second turn
        STOPPED   // Final stop
    };

    // Function called periodically by the timer to send movement commands
    void timer_callback()
    {
        elapsed_time += dt;  // Increment the elapsed time

        switch (state)
        {
        case State::MOVE1:
            if (elapsed_time < move_duration)
            {
                sport_req.Move(req, move_speed, 0.0, 0.0);  // Move forward
            }
            else
            {
                state = State::TURN1;          // Switch to turning state
                elapsed_time = 0.0;            // Reset time
                RCLCPP_INFO(this->get_logger(), "First move done. Starting first turn.");
            }
            break;

        case State::TURN1:
            if (elapsed_time < turn_duration)
            {
                sport_req.Move(req, 0.0, 0.0, turn_speed);  // Rotate in place
            }
            else
            {
                state = State::MOVE2;          // Switch to second forward move
                elapsed_time = 0.0;
                RCLCPP_INFO(this->get_logger(), "First turn done. Starting second move.");
            }
            break;

        case State::MOVE2:
            if (elapsed_time < move_duration)
            {
                sport_req.Move(req, move_speed, 0.0, 0.0);  // Move forward again
            }
            else
            {
                state = State::TURN2;          // Switch to second turn
                elapsed_time = 0.0;
                RCLCPP_INFO(this->get_logger(), "Second move done. Starting second turn.");
            }
            break;

        case State::TURN2:
            if (elapsed_time < turn_duration)
            {
                sport_req.Move(req, 0.0, 0.0, turn_speed);  // Rotate again
            }
            else
            {
                sport_req.Move(req, 0.0, 0.0, 0.0);  // Stop movement
                state = State::STOPPED;             // Set state to STOPPED
                timer_->cancel();                   // Stop the timer
                RCLCPP_INFO(this->get_logger(), "Second turn done. Stopping.");
            }
            break;

        case State::STOPPED:
            break;  // Do nothing once stopped
        }

        req_puber->publish(req);  // Publish the request message
    }

    rclcpp::TimerBase::SharedPtr timer_;  // Timer to trigger movement updates
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;  // Publisher for sending movement requests
    unitree_api::msg::Request req;  // Message object for movement commands
    SportClient sport_req;  // Helper class to fill the request with movement data

    // Parameters
    double move_speed = 0.2;              // Forward speed in meters per second
    double target_distance = 1.0;         // Distance to travel forward in each move
    double dt = 0.002;                    // Time step in seconds
    double elapsed_time = 0.0;            // Accumulated time since state started
    double move_duration = target_distance / move_speed;  // Time to move forward 1 meter

    double turn_speed = M_PI / 2;         // Turning speed in radians per second (90°/s)
    double turn_duration = (215.0 * M_PI / 180.0) / turn_speed;  // Time to turn 215°, in seconds

    State state = State::MOVE1;           // Initial state
};

// Main function: entry point of the program
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // Initialize the ROS 2 system
    rclcpp::spin(std::make_shared<soprt_request>());  // Start the node and process callbacks
    rclcpp::shutdown();  // Cleanly shut down ROS 2
    return 0;  // Exit the program
}
