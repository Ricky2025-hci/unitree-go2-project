#include <unistd.h>  // For POSIX operating system API (not used directly here)
#include <cmath>     // For mathematical operations (e.g., std::abs, std::sqrt if needed)

#include "rclcpp/rclcpp.hpp"  // Main header for ROS 2 C++ client library
#include "unitree_go/msg/sport_mode_state.hpp"  // Message type for sport mode state (not used here directly)
#include "unitree_api/msg/request.hpp"          // Message type to send requests to Unitree robot
#include "common/ros2_sport_client.h"           // Custom class to create movement commands (SportClient)

using std::placeholders::_1;  // Placeholder for std::bind (used for callback binding, not used here)


// Node to periodically publish forward movement command
class soprt_request : public rclcpp::Node  // Define a ROS 2 node named 'soprt_request'
{
public:
    soprt_request() : Node("req_sender")  // Constructor initializes node with name "req_sender"
    {
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);  // Create a publisher for movement requests
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&soprt_request::timer_callback, this));  // Set up a timer to call the callback periodically
    };

private:
    void timer_callback()  // Callback function called at each timer tick
    {
        elapsed_time += dt;  // Increment elapsed time by dt

        if (elapsed_time < move_duration)  // If we haven't reached the target duration
        {
            sport_req.Move(req, move_speed, 0.0, 0.0);  // Send forward movement command
        }
        else
        {
            if (!stopped)  // If not yet stopped
            {
                sport_req.Move(req, 0.0, 0.0, 0.0);  // Send stop command
                stopped = true;  // Mark as stopped
                timer_->cancel();  // Stop the timer
                RCLCPP_INFO(this->get_logger(), "Moved %.2f meters, stopping.", target_distance);  // Log message
            }
        }

        req_puber->publish(req);  // Publish the movement request message
    }

    rclcpp::TimerBase::SharedPtr timer_;  // Timer object to periodically trigger callback
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;  // Publisher to send requests
    unitree_api::msg::Request req;  // Request message to be sent
    SportClient sport_req;  // Instance of helper class to build movement requests

    // Parameters
    double move_speed = 0.2;                  // Movement speed in meters per second
    double target_distance = 1.0;             // Target distance to move forward in meters
    double dt = 0.002;                        // Time step for control loop in seconds
    double elapsed_time = 0.0;                // Time passed since start
    double move_duration = target_distance / move_speed;  // Time needed to move the target distance
    bool stopped = false;                     // Flag to check whether the robot has stopped
};

int main(int argc, char *argv[])  // Main function
{
    rclcpp::init(argc, argv);  // Initialize ROS 2
    rclcpp::spin(std::make_shared<soprt_request>());  // Create node and start spinning (processing callbacks)
    rclcpp::shutdown();  // Shutdown ROS 2
    return 0;  // Exit program
}
