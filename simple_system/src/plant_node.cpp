#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "simple_system_interfaces/msg/plant_state.hpp" 
#include "simple_system_interfaces/msg/control_input.hpp" 

class PlantNode : public rclcpp::Node {
private:
    double position = 0.0;
    double tau = 0.5;       
    double K = 2.0;        
    double dt = 0.01; 

    rclcpp::Subscription<simple_system_interfaces::msg::ControlInput>::SharedPtr input_subscriber;
    rclcpp::Publisher<simple_system_interfaces::msg::PlantState>::SharedPtr state_publisher;
    rclcpp::TimerBase::SharedPtr timer;

    void control_input_callback(const simple_system_interfaces::msg::ControlInput::SharedPtr msg) {
        double input = msg->input;
        RCLCPP_INFO(this->get_logger(), "Received control input: %f", input);

        double newPosition = (dt * K * input + position) / (1 + dt / tau);
        RCLCPP_INFO(this->get_logger(), "Updated position from %f to %f", position, newPosition);
        position = newPosition;

        publish_state();
    }

    void publish_state() {
        auto message = simple_system_interfaces::msg::PlantState();
        message.position = position;
        state_publisher->publish(message);
    }

public:
    PlantNode() : Node("plant_node") {
        input_subscriber = this->create_subscription<simple_system_interfaces::msg::ControlInput>(
            "control_input", 10, std::bind(&PlantNode::control_input_callback, this, std::placeholders::_1));
        state_publisher = this->create_publisher<simple_system_interfaces::msg::PlantState>("plant_state", 10);
        timer = this->create_wall_timer(
            std::chrono::milliseconds(int(dt*1000)), [this]() { publish_state(); });
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlantNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
