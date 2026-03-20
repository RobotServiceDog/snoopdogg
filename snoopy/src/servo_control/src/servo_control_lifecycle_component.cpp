#include "servo_control/servo_control_lifecycle_component.hpp"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

ServoControlLifecycleNode::ServoControlLifecycleNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("ServoControlLifecycleNode", options), 
      sockfd_(-1) // Initialize socket as invalid
{
    RCLCPP_INFO(get_logger(), "ServoControlLifecycleNode constructed (unconfigured).");
}

CallbackReturn ServoControlLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring servo...");

    // Initialize data
    joint_angles_ = {0., 0., 0., 0., 45., 45., 45., 45., -45., -45., -45., -45.};
    joint_state_msg_.position = joint_angles_;

    // Setup UDP Socket
    this->init_udp_socket();
    if (sockfd_ < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize socket during configuration.");
        return CallbackReturn::FAILURE;
    }

    // Create Subscriber (it will not receive data until activated)
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&ServoControlLifecycleNode::servo_callback, this, std::placeholders::_1)
    );

    // Create Timer (it will not fire until activated)
    udp_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ServoControlLifecycleNode::timer_callback, this)
    );
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn ServoControlLifecycleNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating servo... Communication enabled.");
    
    // In ROS 2 Lifecycle, Managed entities (timers/subscribers) 
    // are automatically enabled/disabled by the transition.
    // Do NOT .reset() them here as that deletes the objects.
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn ServoControlLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating servo... Communication paused.");
    
    // We don't close the socket here because we might "Activate" again soon.
    return CallbackReturn::SUCCESS;
}

CallbackReturn ServoControlLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up servo node...");

    // Properly destroy communication objects
    joint_state_subscriber_.reset();
    udp_timer_.reset();

    if (sockfd_ >= 0) {
        close(sockfd_);
        sockfd_ = -1;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ServoControlLifecycleNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Shutting down from state: %s", state.label().c_str());
    if (sockfd_ >= 0) {
        close(sockfd_);
    }
    return CallbackReturn::SUCCESS;
}

void ServoControlLifecycleNode::init_udp_socket()
{
    ip_ = "172.17.0.1"; 
    port_ = 5005;

    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        RCLCPP_ERROR(get_logger(), "Socket creation failed: %s", strerror(errno));
        return;
    }

    std::memset(&servaddr_, 0, sizeof(servaddr_));
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(port_);
    
    if (inet_pton(AF_INET, ip_.c_str(), &servaddr_.sin_addr) <= 0) {
        RCLCPP_ERROR(get_logger(), "Invalid IP address: %s", ip_.c_str());
    }
}

void ServoControlLifecycleNode::servo_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Update the internal message so the timer sends the latest data
    joint_state_msg_.position = msg->position;
}

void ServoControlLifecycleNode::timer_callback()
{
    // Important: check if node is ACTIVE. Lifecycle timers can sometimes 
    // fire during transitions.
    if (sockfd_ < 0 || joint_state_msg_.position.empty()) {
        return;
    }

    ssize_t sent_bytes = sendto(
        sockfd_,
        joint_state_msg_.position.data(),
        joint_state_msg_.position.size() * sizeof(double),
        0,
        (const struct sockaddr*)&servaddr_,
        sizeof(servaddr_)
    );

    if (sent_bytes < 0) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "UDP send failed: %s", strerror(errno));
    } else {
         RCLCPP_DEBUG(get_logger(), "Sent %zd bytes to %s:%d", sent_bytes, ip_.c_str(), port_);
    }    
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ServoControlLifecycleNode)