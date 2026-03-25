#include "apriltag_detection/apriltag_detection_lifecycle_component.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

ApriltagDetectionLifecycleNode::ApriltagDetectionLifecycleNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("ApriltagDetectionLifecycleNode", options),
      image_topic_("/camera/image_raw"),
      camera_info_topic_("/camera/camera_info"),
      tag_size_m_(0.162),
      target_tag_id_(-1),
      environment_(0),
      camera_device_index_(0),
      camera_read_period_ms_(100),
      use_camera_info_topic_(true),
      camera_matrix_param_({1242.86002, 0.0, 578.436600, 0.0, 1238.06090, 322.452739, 0.0, 0.0, 1.0}),
      distortion_coefficients_param_({-0.22467977, -0.14194663, -0.00117893, -0.00083476, 0.11091554}),
      calibration_width_(1280),
      calibration_height_(720),
      has_camera_info_(false)
{
    this->declare_parameter<std::string>("image_topic", image_topic_);
    this->declare_parameter<std::string>("camera_info_topic", camera_info_topic_);
    this->declare_parameter<double>("tag_size_m", tag_size_m_);
    this->declare_parameter<int>("target_tag_id", target_tag_id_);
    this->declare_parameter<int>("environment", environment_);
    this->declare_parameter<int>("camera_device_index", camera_device_index_);
    this->declare_parameter<int>("camera_read_period_ms", camera_read_period_ms_);
    this->declare_parameter<bool>("use_camera_info_topic", use_camera_info_topic_);
    this->declare_parameter<std::vector<double>>("camera_matrix", camera_matrix_param_);
    this->declare_parameter<std::vector<double>>(
        "distortion_coefficients", distortion_coefficients_param_);
    this->declare_parameter<int>("calibration_width", calibration_width_);
    this->declare_parameter<int>("calibration_height", calibration_height_);

    RCLCPP_INFO(get_logger(), "ApriltagDetectionLifecycleNode constructed (unconfigured).");
}

CallbackReturn ApriltagDetectionLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{
    this->get_parameter("image_topic", image_topic_);
    this->get_parameter("camera_info_topic", camera_info_topic_);
    this->get_parameter("tag_size_m", tag_size_m_);
    this->get_parameter("target_tag_id", target_tag_id_);
    this->get_parameter("environment", environment_);
    this->get_parameter("camera_device_index", camera_device_index_);
    this->get_parameter("camera_read_period_ms", camera_read_period_ms_);
    this->get_parameter("use_camera_info_topic", use_camera_info_topic_);
    this->get_parameter("camera_matrix", camera_matrix_param_);
    this->get_parameter("distortion_coefficients", distortion_coefficients_param_);
    this->get_parameter("calibration_width", calibration_width_);
    this->get_parameter("calibration_height", calibration_height_);

    if (environment_ != 0 && environment_ != 1) {
        RCLCPP_ERROR(get_logger(), "Parameter environment must be 0 (sim) or 1 (real), got %d", environment_);
        return CallbackReturn::FAILURE;
    }

    if (tag_size_m_ <= 0.0) {
        RCLCPP_ERROR(get_logger(), "Parameter tag_size_m must be > 0.0, got %.4f", tag_size_m_);
        return CallbackReturn::FAILURE;
    }

    if (camera_read_period_ms_ <= 0) {
        RCLCPP_ERROR(get_logger(), "Parameter camera_read_period_ms must be > 0, got %d", camera_read_period_ms_);
        return CallbackReturn::FAILURE;
    }

    if (camera_matrix_param_.size() != 9) {
        RCLCPP_ERROR(
            get_logger(), "Parameter camera_matrix must have 9 values, got %zu",
            camera_matrix_param_.size());
        return CallbackReturn::FAILURE;
    }

    if (distortion_coefficients_param_.empty()) {
        RCLCPP_ERROR(get_logger(), "Parameter distortion_coefficients must not be empty.");
        return CallbackReturn::FAILURE;
    }

    if (calibration_width_ <= 0 || calibration_height_ <= 0) {
        RCLCPP_ERROR(
            get_logger(), "Calibration resolution must be positive, got %dx%d",
            calibration_width_, calibration_height_);
        return CallbackReturn::FAILURE;
    }

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    apriltag_pose_msg_ = geometry_msgs::msg::Pose2D();

    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            camera_matrix_.at<double>(row, col) = camera_matrix_param_[row * 3 + col];
        }
    }
    dist_coeffs_ = cv::Mat(distortion_coefficients_param_).reshape(1, 1).clone();
    has_camera_info_ = true;

    apriltag_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>(
        "/apriltag_pose", rclcpp::QoS(10));

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = callback_group_;

    if (environment_ == 0) {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ApriltagDetectionLifecycleNode::image_callback, this, std::placeholders::_1),
            subscription_options);

        if (use_camera_info_topic_) {
            camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic_, rclcpp::SensorDataQoS(),
                std::bind(&ApriltagDetectionLifecycleNode::camera_info_callback, this, std::placeholders::_1),
                subscription_options);
        } else {
            camera_info_sub_.reset();
        }
    } else {
        image_sub_.reset();
        camera_info_sub_.reset();
        if (use_camera_info_topic_) {
            RCLCPP_WARN(
                get_logger(),
                "environment=1 (real) uses direct camera capture. camera_info_topic subscription is disabled.");
        }
    }

    RCLCPP_INFO(
        get_logger(),
        "Configuring detection. environment=%d image_topic=%s camera_info_topic=%s "
        "camera_device_index=%d camera_read_period_ms=%d "
        "use_camera_info_topic=%s tag_size_m=%.3f target_tag_id=%d calibration_resolution=%dx%d",
        environment_, image_topic_.c_str(), camera_info_topic_.c_str(),
        camera_device_index_, camera_read_period_ms_,
        use_camera_info_topic_ ? "true" : "false", tag_size_m_, target_tag_id_,
        calibration_width_, calibration_height_);

    return CallbackReturn::SUCCESS;
}

CallbackReturn ApriltagDetectionLifecycleNode::on_activate(const rclcpp_lifecycle::State &)
{
    apriltag_pose_pub_->on_activate();

    if (environment_ == 1) {
        if (!camera_capture_.open(camera_device_index_)) {
            RCLCPP_ERROR(get_logger(), "Failed to open camera device index %d", camera_device_index_);
            return CallbackReturn::FAILURE;
        }

        camera_capture_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(calibration_width_));
        camera_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(calibration_height_));

        camera_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(camera_read_period_ms_),
            std::bind(&ApriltagDetectionLifecycleNode::real_detection_timer_callback, this));

        RCLCPP_INFO(
            get_logger(),
            "Activating detection in real mode with direct camera capture (device=%d, period_ms=%d).",
            camera_device_index_, camera_read_period_ms_);
    } else {
        RCLCPP_INFO(get_logger(), "Activating detection in simulation mode (ROS image subscription).");
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ApriltagDetectionLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    camera_timer_.reset();
    if (camera_capture_.isOpened()) {
        camera_capture_.release();
    }

    apriltag_pose_pub_->on_deactivate();

    RCLCPP_INFO(get_logger(), "Deactivating detection...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ApriltagDetectionLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    camera_timer_.reset();
    if (camera_capture_.isOpened()) {
        camera_capture_.release();
    }

    apriltag_pose_pub_.reset();
    image_sub_.reset();
    camera_info_sub_.reset();
    callback_group_.reset();

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
    if (environment_ != 0) {
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "cv_bridge conversion failed: %s", e.what());
        return;
    }

    process_frame_and_publish(cv_ptr->image);
}

void ApriltagDetectionLifecycleNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
    if (environment_ != 0 || !use_camera_info_topic_) {
        return;
    }

    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            camera_matrix_.at<double>(row, col) = msg->k[row * 3 + col];
        }
    }

    if (msg->d.empty()) {
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    } else {
        dist_coeffs_ = cv::Mat(msg->d).reshape(1, 1).clone();
    }

    has_camera_info_ = true;
}

void ApriltagDetectionLifecycleNode::real_detection_timer_callback()
{
    if (environment_ != 1) {
        return;
    }
    
    if (!camera_capture_.isOpened()) {
        if (!camera_capture_.open(camera_device_index_)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Failed to open camera device index %d", camera_device_index_);
            return;
        }
        camera_capture_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(calibration_width_));
        camera_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(calibration_height_));
        
    }

    cv::Mat frame;
    if (!camera_capture_.read(frame) || frame.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read frame from camera device %d", camera_device_index_);
        return;
    }

    process_frame_and_publish(frame);
}

void ApriltagDetectionLifecycleNode::process_frame_and_publish(const cv::Mat &frame)
{
    if (!apriltag_pose_pub_ || !apriltag_pose_pub_->is_activated()) {
        return;
    }

    if (frame.empty()) {
        return;
    }

    if (frame.cols != calibration_width_ || frame.rows != calibration_height_) {
        if (use_camera_info_topic_ && environment_ == 0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Image resolution is %dx%d but static calibration is %dx%d. "
                "CameraInfo should override these intrinsics if available.",
                frame.cols, frame.rows, calibration_width_, calibration_height_);
        } else {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Image resolution is %dx%d but calibration is %dx%d. "
                "Pose estimation may be inaccurate.",
                frame.cols, frame.rows, calibration_width_, calibration_height_);
        }
    }

    if (!has_camera_info_ || camera_matrix_.empty() || dist_coeffs_.empty()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Waiting for camera_info or static calibration");
        return;
    }

    cv::Mat gray_image;
    if (frame.channels() == 1) {
        gray_image = frame;
    } else if (frame.channels() == 3) {
        cv::cvtColor(frame, gray_image, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
        cv::cvtColor(frame, gray_image, cv::COLOR_BGRA2GRAY);
    } else {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Unsupported image channel count: %d", frame.channels());
        return;
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
        selected_corners, static_cast<float>(tag_size_m_), camera_matrix_, dist_coeffs_, rvecs, tvecs);

    if (rvecs.empty() || tvecs.empty()) {
        return;
    }

    cv::Mat rotation_matrix;
    cv::Rodrigues(rvecs.front(), rotation_matrix);
    const cv::Vec3d tag_z_axis(
        rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 2));
    // Find the heading such that the tag's backward direction (z-axis) points along the positive x-axis in the robot's frame.
    const double heading = std::atan2(tag_z_axis[0], tag_z_axis[2]);

    // apriltag_pose_msg_.x = tvecs.front()[0];
    // apriltag_pose_msg_.y = tvecs.front()[2];  // Pose2D.y used for forward depth (z in camera frame).

    apriltag_pose_msg_.x = tvecs.front()[2];
    apriltag_pose_msg_.y = -1 * tvecs.front()[0]; // -1 to align with right hand rule
    apriltag_pose_msg_.theta = heading;

    printf("%f\n", apriltag_pose_msg_.y);

    apriltag_pose_pub_->publish(apriltag_pose_msg_);
}

// Register this component so it can be loaded into a component container
RCLCPP_COMPONENTS_REGISTER_NODE(ApriltagDetectionLifecycleNode)
