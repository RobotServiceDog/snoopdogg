#include "apriltag_detection/apriltag_detection_lifecycle_component.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>
#include "cv_bridge/cv_bridge.hpp"

ApriltagDetectionLifecycleNode::ApriltagDetectionLifecycleNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("ApriltagDetectionLifecycleNode", options),
      image_topic_("/camera/image_raw"),
      camera_info_topic_("/camera/camera_info"),
      tag_size_m_(0.162),
      target_tag_id_(-1),
      publish_period_ms_(100),
      has_camera_info_(false)
{
    this->declare_parameter<std::string>("image_topic", image_topic_);
    this->declare_parameter<std::string>("camera_info_topic", camera_info_topic_);
    this->declare_parameter<double>("tag_size_m", tag_size_m_);
    this->declare_parameter<int>("target_tag_id", target_tag_id_);
    this->declare_parameter<int>("publish_period_ms", publish_period_ms_);

    RCLCPP_INFO(get_logger(), "ApriltagDetectionLifecycleNode constructed (unconfigured).");
}

CallbackReturn ApriltagDetectionLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{
    this->get_parameter("image_topic", image_topic_);
    this->get_parameter("camera_info_topic", camera_info_topic_);
    this->get_parameter("tag_size_m", tag_size_m_);
    this->get_parameter("target_tag_id", target_tag_id_);
    this->get_parameter("publish_period_ms", publish_period_ms_);

    if (tag_size_m_ <= 0.0) {
        RCLCPP_ERROR(get_logger(), "Parameter tag_size_m must be > 0.0, got %.4f", tag_size_m_);
        return CallbackReturn::FAILURE;
    }

    if (publish_period_ms_ <= 0) {
        RCLCPP_ERROR(get_logger(), "Parameter publish_period_ms must be > 0, got %d", publish_period_ms_);
        return CallbackReturn::FAILURE;
    }

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    apriltag_pose_msg_ = geometry_msgs::msg::Pose2D();

    apriltag_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>(
        "/apriltag_pose", rclcpp::QoS(10));
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ApriltagDetectionLifecycleNode::image_callback, this, std::placeholders::_1));
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ApriltagDetectionLifecycleNode::camera_info_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Configuring detection. image_topic=%s camera_info_topic=%s tag_size_m=%.3f target_tag_id=%d",
        image_topic_.c_str(), camera_info_topic_.c_str(), tag_size_m_, target_tag_id_);

    return CallbackReturn::SUCCESS;
}

CallbackReturn ApriltagDetectionLifecycleNode::on_activate(const rclcpp_lifecycle::State &)
{
    apriltag_pose_pub_->on_activate();

    timer_ = this -> create_wall_timer(
        std::chrono::milliseconds(publish_period_ms_),
        std::bind(&ApriltagDetectionLifecycleNode::apriltag_detection_callback, this)
    );

    RCLCPP_INFO(get_logger(), "Activating detection...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ApriltagDetectionLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    timer_.reset();
    apriltag_pose_pub_->on_deactivate();

    RCLCPP_INFO(get_logger(), "Deactivating detection...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ApriltagDetectionLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    timer_.reset();
    apriltag_pose_pub_.reset();
    image_sub_.reset();
    camera_info_sub_.reset();

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_image_.reset();
    camera_matrix_.release();
    dist_coeffs_.release();
    has_camera_info_ = false;

    RCLCPP_INFO(get_logger(), "Cleaning up detection node...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ApriltagDetectionLifecycleNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Shutting down from state: %s", state.label().c_str());

    return CallbackReturn::SUCCESS;
}

void ApriltagDetectionLifecycleNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_image_ = msg;
}

void ApriltagDetectionLifecycleNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            camera_matrix.at<double>(row, col) = msg->k[row * 3 + col];
        }
    }

    cv::Mat dist_coeffs;
    if (msg->d.empty()) {
        dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    } else {
        dist_coeffs = cv::Mat(msg->d).reshape(1, 1).clone();
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    camera_matrix_ = camera_matrix;
    dist_coeffs_ = dist_coeffs;
    has_camera_info_ = true;
}

void ApriltagDetectionLifecycleNode::apriltag_detection_callback()
{
    if (!apriltag_pose_pub_ || !apriltag_pose_pub_->is_activated()) {
        return;
    }

    sensor_msgs::msg::Image::ConstSharedPtr image_msg;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        image_msg = latest_image_;
        if (has_camera_info_) {
            camera_matrix = camera_matrix_.clone();
            dist_coeffs = dist_coeffs_.clone();
        }
    }

    if (!image_msg) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Waiting for images on %s", image_topic_.c_str());
        return;
    }

    if (camera_matrix.empty() || dist_coeffs.empty()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Waiting for camera_info on %s",
            camera_info_topic_.c_str());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(image_msg);
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "cv_bridge conversion failed: %s", e.what());
        return;
    }

    cv::Mat gray_image;
    if (cv_ptr->image.channels() == 1) {
        gray_image = cv_ptr->image;
    } else {
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(
        gray_image, dictionary_, corners, ids, cv::aruco::DetectorParameters::create(), rejected);

    if (ids.empty()) {
        return;
    }

    int selected_index = 0;
    if (target_tag_id_ >= 0) {
        auto it = std::find(ids.begin(), ids.end(), target_tag_id_);
        if (it == ids.end()) {
            return;
        }
        selected_index = static_cast<int>(std::distance(ids.begin(), it));
    }

    std::vector<std::vector<cv::Point2f>> selected_corners{corners.at(selected_index)};
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    cv::aruco::estimatePoseSingleMarkers(
        selected_corners, static_cast<float>(tag_size_m_), camera_matrix, dist_coeffs, rvecs, tvecs);

    if (rvecs.empty() || tvecs.empty()) {
        return;
    }

    cv::Mat rotation_matrix;
    cv::Rodrigues(rvecs.front(), rotation_matrix);
    const cv::Vec3d tag_z_axis(
        rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 2));
    const double heading = std::atan2(tag_z_axis[0], tag_z_axis[2]);

    apriltag_pose_msg_.x = tvecs.front()[0];
    apriltag_pose_msg_.y = tvecs.front()[2];  // Pose2D.y used for forward depth (z in camera frame).
    apriltag_pose_msg_.theta = heading;

    apriltag_pose_pub_->publish(apriltag_pose_msg_);
}

// Register this component so it can be loaded into a component container
RCLCPP_COMPONENTS_REGISTER_NODE(ApriltagDetectionLifecycleNode)
