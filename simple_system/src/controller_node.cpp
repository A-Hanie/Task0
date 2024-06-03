#include "rclcpp/rclcpp.hpp"
#include "simple_system_interfaces/msg/plant_state.hpp"
#include "simple_system_interfaces/msg/control_input.hpp"
#include "simple_system_interfaces/srv/set_setpoint.hpp"

class ControllerNode : public rclcpp::Node {
private:
    double setpoint = 0.0;
    double kp = 0.1; 
    double ki = 0.0;  
    double kd = 0.1;  

    double integral_error = 0.0;
    double last_error = 0.0;
    
    rclcpp::Publisher<simple_system_interfaces::msg::ControlInput>::SharedPtr control_publisher;
    rclcpp::Subscription<simple_system_interfaces::msg::PlantState>::SharedPtr state_subscriber;
    rclcpp::Service<simple_system_interfaces::srv::SetSetpoint>::SharedPtr setpoint_service;

    void state_callback(const simple_system_interfaces::msg::PlantState::SharedPtr msg) {
        double error = setpoint - msg->position;
        integral_error += error;
        double derivative_error = error - last_error;

        double control_signal = kp * error + ki * integral_error + kd * derivative_error;
        last_error = error;

        RCLCPP_INFO(this->get_logger(), "control signal: %f", control_signal);

        if(std::isnan(control_signal)) {
            control_signal = 0; 
        }

        auto control_msg = simple_system_interfaces::msg::ControlInput();
        control_msg.input = control_signal;
        control_publisher->publish(control_msg);
    }


    void handle_setpoint(const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<simple_system_interfaces::srv::SetSetpoint::Request> request,
                         std::shared_ptr<simple_system_interfaces::srv::SetSetpoint::Response> response) {
        setpoint = request->setpoint;
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Setpoint updated : %f", setpoint);
    }

public:
    ControllerNode() : Node("controller_node") {
        
        this->declare_parameter<double>("setpoint", 0.0);
        this->declare_parameter<double>("kp", 0.1);
        this->declare_parameter<double>("ki", 0.0);
        this->declare_parameter<double>("kd", 0.1);

        this->get_parameter("setpoint", setpoint);
        this->get_parameter("kp", kp);
        this->get_parameter("ki", ki);
        this->get_parameter("kd", kd);

        
        control_publisher = this->create_publisher<simple_system_interfaces::msg::ControlInput>("control_input", 10);
        state_subscriber = this->create_subscription<simple_system_interfaces::msg::PlantState>(
            "plant_state", 10, std::bind(&ControllerNode::state_callback, this, std::placeholders::_1));
        setpoint_service = this->create_service<simple_system_interfaces::srv::SetSetpoint>(
            "set_setpoint", std::bind(&ControllerNode::handle_setpoint, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
