#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>

class ThermalCameraNode : public rclcpp::Node
{
public:
    ThermalCameraNode()
        : Node("thermal_camera_node")
    {
        this->declare_parameter<std::string>("video_device", "/dev/video0");
        this->get_parameter("video_device", video_device_);

        // Distance compensation parameters
        this->declare_parameter<float>("reference_distance", 1.0f); // meters
        this->declare_parameter<float>("compensation_coefficient", 2.0f); // °C/m
        this->get_parameter("reference_distance", ref_distance_);
        this->get_parameter("compensation_coefficient", comp_coeff_);

        // Publishers
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/thermal_camera/image_raw", 10);
        temp_pub_ = this->create_publisher<std_msgs::msg::Float32>("/thermal_camera/body_temperature", 10);

        // Open thermal camera with V4L2 backend
        cap_.open(video_device_, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", video_device_.c_str());
            rclcpp::shutdown();
            return;
        }

        // Force Y16, 160x120, 9 FPS
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 160);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 120);
        cap_.set(cv::CAP_PROP_FPS, 9);
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','1','6',' '));

        // Start capture thread
        capture_thread_ = std::thread(&ThermalCameraNode::captureLoop, this);

        // Timer to publish frames
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&ThermalCameraNode::publishFrame, this));
    }

    ~ThermalCameraNode() {
        stop_capture_ = true;
        if (capture_thread_.joinable())
            capture_thread_.join();
    }

private:
    void captureLoop() {
        cv::Mat frame;
        while (!stop_capture_) {
            cap_ >> frame;
            if (frame.empty()) continue;

            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = frame.clone();
        }
    }

    // Replace this function with your actual distance sensor input
    float getDistanceToPerson() {
        // Example: fixed 1 meter for testing
        return 1.0f;
    }

    void publishFrame() {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (latest_frame_.empty()) return;
            frame = latest_frame_;
        }

        cv::Mat display_frame;

        // Ensure 16-bit
        if (frame.type() != CV_16UC1) {
            if (frame.channels() == 3)
                cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            frame.convertTo(frame, CV_16U, 256);
        }

        // Convert to Celsius
        cv::Mat temp_C;
        frame.convertTo(temp_C, CV_32F, 1.0 / 100.0);
        temp_C = temp_C - 273.15f;

        // Mask for human body temps
        cv::Mat mask;
        cv::inRange(temp_C, 28.0, 40.0, mask);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Find largest contour (assume person)
            size_t largest_idx = 0;
            double max_area = 0;
            for (size_t i = 0; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area > max_area) {
                    max_area = area;
                    largest_idx = i;
                }
            }

            cv::Rect roi = cv::boundingRect(contours[largest_idx]);
            cv::Mat roi_temp = temp_C(roi);
            cv::Scalar mean_val = cv::mean(roi_temp);
            float avg_temp = static_cast<float>(mean_val[0]);

            // Distance compensation
            float distance = getDistanceToPerson();
            float corrected_temp = avg_temp + comp_coeff_ * (distance - ref_distance_);

            // Publish corrected temperature ONLY if person detected
            std_msgs::msg::Float32 temp_msg;
            temp_msg.data = corrected_temp;
            temp_pub_->publish(temp_msg);

            // Overlay on display
            cv::normalize(temp_C, display_frame, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::applyColorMap(display_frame, display_frame, cv::COLORMAP_JET);
            cv::rectangle(display_frame, roi, cv::Scalar(0, 255, 0), 2);

            RCLCPP_INFO(this->get_logger(), "Measured [C]: %.2f, Corrected [C]: %.2f", avg_temp, corrected_temp);

        } else {
            // No person detected -> don't publish temperature
            cv::normalize(temp_C, display_frame, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::applyColorMap(display_frame, display_frame, cv::COLORMAP_JET);
        }

        // Publish thermal image in all cases
        std_msgs::msg::Header header;
        header.stamp = this->now();
        auto img_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, display_frame).toImageMsg();
        image_pub_->publish(*img_msg);
    }

    // ROS
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // OpenCV
    cv::VideoCapture cap_;
    std::string video_device_;

    // Threading
    std::thread capture_thread_;
    std::mutex frame_mutex_;
    cv::Mat latest_frame_;
    bool stop_capture_ = false;

    // Distance compensation
    float ref_distance_ = 1.0f; // meters
    float comp_coeff_ = 2.0f;   // °C/m
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThermalCameraNode>());
    rclcpp::shutdown();
    return 0;
}

