#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class PlantSimNode : public rclcpp::Node {
public:
    PlantSimNode() : Node("plant_sim_node"), state_(0.0) {
        // Subscribe to control output from controller
        control_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "control_output", 10,
            std::bind(&PlantSimNode::control_callback, this, std::placeholders::_1));
        
        // Publisher for simulated measurement
        measurement_pub_ = this->create_publisher<std_msgs::msg::Float64>("measurement", 10);

        // Timer to publish measurement at intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PlantSimNode::publish_measurement, this));
    }

private:
    void control_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        // For simple first-order system the state += control * dt
        double dt = 0.1; // 100 ms
        state_ += msg->data * dt;
        RCLCPP_INFO(this->get_logger(), "Applied control: %f, new state: %f", msg->data, state_);
    }

    void publish_measurement() {
        auto message = std_msgs::msg::Float64();
        message.data = state_;
        measurement_pub_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published measurement: %f", state_);
    }

    // Internal state variable
    double state_;

    // Defining communication handles
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr control_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr measurement_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlantSimNode>());
    rclcpp::shutdown();
    return 0;
}
