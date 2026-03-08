#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <string>


using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ServoControlLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit ServoControlLifecycleNode(const rclcpp::NodeOptions &options);

    protected:
        CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    private:
        void init_udp_socket();
        void servo_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void timer_callback();

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

        rclcpp::TimerBase::SharedPtr udp_timer_; // For testing UDP communication
        sensor_msgs::msg::JointState joint_state_msg_;
        std::vector<double> joint_angles_;


        std::string ip_;
        int port_;
        int sockfd_;
        sockaddr_in servaddr_;
};