#include <chrono>
#include <memory>
#include <cmath>
#include <string>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode()
    : Node("odometry_node"),
      wheel_radius_(0.033),
      wheel_separation_(0.160),
      ticks_per_rev_(360),
      last_left_ticks_(0),
      last_right_ticks_(0),
      x_(0.0), y_(0.0), theta_(0.0),
      last_time_(this->now())
    {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(50ms, std::bind(&OdometryNode::update, this));
        openSerial("/dev/ttyUSB0");
    }

private:
    void openSerial(const std::string &device) {
        serial_fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", device.c_str());
            rclcpp::shutdown();
        }

        termios tty{};
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to get serial attributes");
            rclcpp::shutdown();
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag = tty.c_oflag = tty.c_lflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;
        tcsetattr(serial_fd_, TCSANOW, &tty);
    }

    void update() {
        char buffer[128];
        int n = read(serial_fd_, buffer, sizeof(buffer) - 1);
        if (n <= 0) return;

        buffer[n] = '\0';
        std::string data(buffer);

        int left_ticks = 0, right_ticks = 0;
        if (sscanf(data.c_str(), "L:%d R:%d", &left_ticks, &right_ticks) != 2) return;

        // Tick differences
        int d_left = left_ticks - last_left_ticks_;
        int d_right = right_ticks - last_right_ticks_;
        last_left_ticks_ = left_ticks;
        last_right_ticks_ = right_ticks;

        // Time difference
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Distance in meters
        double dist_left = 2 * M_PI * wheel_radius_ * d_left / ticks_per_rev_;
        double dist_right = 2 * M_PI * wheel_radius_ * d_right / ticks_per_rev_;
        double dist = (dist_left + dist_right) / 2.0;
        double delta_theta = (dist_right - dist_left) / wheel_separation_;

        // Update pose
        theta_ += delta_theta;
        x_ += dist * std::cos(theta_);
        y_ += dist * std::sin(theta_);

        // Velocities
        double vx = dist / dt;
        double vth = delta_theta / dt;

        // Create odometry msg
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation = tf2::toMsg(q);
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.angular.z = vth;

        odom_pub_->publish(odom);

        // Broadcast tf
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = current_time;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(tf);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_fd_;

    // Robot parameters
    const double wheel_radius_;
    const double wheel_separation_;
    const int ticks_per_rev_;

    // Encoder state
    int last_left_ticks_, last_right_ticks_;
    double x_, y_, theta_;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
