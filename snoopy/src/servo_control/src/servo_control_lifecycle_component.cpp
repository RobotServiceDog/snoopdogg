
#include "servo_control/servo_control_lifecycle_component.hpp"

ServoControlLifecycleNode::ServoControlLifecycleNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("ServoControlLifecycleNode", options)
{
    RCLCPP_INFO(get_logger(), "ServoControlLifecycleNode constructed (unconfigured).");
}

CallbackReturn ServoControlLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring servo...");

    // joint_state_msg_ = sensor_msgs::msg::JointState();
    // joint_angles_ = {0., 0., 0., 0., 45., 45., 45., 45., -45., -45., -45., -45.};
    // joint_state_msg_.position = joint_angles_;


    this->init_udp_socket();
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&ServoControlLifecycleNode::servo_callback, this, std::placeholders::_1)
    );

    // udp_timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(100),
    //     std::bind(&ServoControlLifecycleNode::timer_callback, this)
    // );
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn ServoControlLifecycleNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating servo...");

    // udp_timer_->reset();
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn ServoControlLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating servo...");

    joint_state_subscriber_.reset();
    // udp_timer_->cancel();

    close(sockfd_);

    return CallbackReturn::SUCCESS;
}

CallbackReturn ServoControlLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up servo node...");

    joint_state_subscriber_.reset();
    // udp_timer_.reset();

    close(sockfd_);

    return CallbackReturn::SUCCESS;
}

CallbackReturn ServoControlLifecycleNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Shutting down from state: %s", state.label().c_str());

    return CallbackReturn::SUCCESS;
}

void ServoControlLifecycleNode::init_udp_socket()
{
    RCLCPP_INFO(get_logger(), "Creating UDP socket...");

    ip_ = "172.17.0.1"; // Change if required on PI
    port_ = 5005;

    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) { perror("socket creation failed"); }

    std::memset(&servaddr_, 0, sizeof(servaddr_));
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(port_);
    if (inet_pton(AF_INET, ip_.c_str(), &servaddr_.sin_addr) <= 0) {
        std::cerr << "Invalid IP address\n";
    }
}

void ServoControlLifecycleNode::servo_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received JointState with %zu positions", msg->position.size());
    // Handle servo commands here

    std::string message = "Hello from C++ UDP\n";

    ssize_t sent_bytes = sendto(
        sockfd_,
        msg->position.data(),
        msg->position.size() * sizeof(double),
        0,
        (const sockaddr*)&servaddr_,
        sizeof(servaddr_)
    );

    if (sent_bytes < 0) {
        perror("sendto failed");
    } else {
         RCLCPP_INFO(get_logger(), "Message sent.");
    }    

}

// void ServoControlLifecycleNode::timer_callback()
// {
//     std::string message = "Hello from C++ UDP\n";

//     ssize_t sent_bytes = sendto(
//         sockfd_,
//         joint_state_msg_.position.data(),
//         joint_state_msg_.position.size() * sizeof(double),
//         0,
//         (const sockaddr*)&servaddr_,
//         sizeof(servaddr_)
//     );

//     if (sent_bytes < 0) {
//         perror("sendto failed");
//     } else {
//          RCLCPP_INFO(get_logger(), "Message sent.");
//     }    
// }

// Register this component so it can be loaded into a component container
RCLCPP_COMPONENTS_REGISTER_NODE(ServoControlLifecycleNode)
