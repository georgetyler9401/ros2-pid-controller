#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "pid_controller/pid.hpp"

// create class for a pid controller node 
class PIDNode : public rclcpp::Node {
public:
    // outline constructor for node 
    PIDNode() : Node("pid_controller_node"), pid_(1.0, 0.0, 0.0), setpoint_(0.0), measurement_(0.0) {
        // declare and get parameters for PID gains
        this->declare_parameter("kp", 1.0);
        this->declare_parameter("ki", 0.0);
        this->declare_parameter("kd", 0.0);
        double kp = this->get_parameter("kp").as_double();
        double ki = this->get_parameter("ki").as_double();
        double kd = this->get_parameter("kd").as_double();
        pid_ = PID(kp, ki, kd);

        // declare and get parameter for setpoint making setpoint a topic so that it can be changed dynamically
        setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64>("setpoint", 10, std::bind(&PIDNode::setpoint_callback, this, std::placeholders::_1));

        // create publisher for control output
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("control_output", 10);

        // create subscription for measurement input with buffer size 10
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "measurement", 10,
            std::bind(&PIDNode::measurement_callback, this, std::placeholders::_1)); // bind the callback function

        // create timer to periodically compute and publish control output
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PIDNode::timer_callback, this));
    }
private:
    // callback function to handle incoming measurement messages
    void measurement_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        measurement_ = msg->data; // update the current measurement with data from message
        RCLCPP_INFO(this->get_logger(), "Received measurement: '%f'", measurement_); // log the measurement
    }
    void timer_callback() {
        double dt = 0.1; // time step in seconds between publication of control output
        double control_output = pid_.compute(setpoint_, measurement_, dt); // compute the control
        auto message = std_msgs::msg::Float64();
        message.data = control_output; // populate the message with control output
        publisher_->publish(message); // publish the control output
        RCLCPP_INFO(this->get_logger(), "Published control output: '%f'", control_output); // log the control output
    }

    
    void setpoint_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    setpoint_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Updated setpoint: '%f'", setpoint_);
}

    PID pid_; // instance of the PID controller
    double setpoint_; // desired target value
    double measurement_; // current measured value
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_; // publisher for control output
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_; // subscription for measurement input
    rclcpp::TimerBase::SharedPtr timer_; // timer to trigger periodic control computation
};

int main(int argc, char * argv[]) {
    // initialize ROS 2, create and spin the PID node, then shutdown
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDNode>());
    rclcpp::shutdown();
    return 0;
}