#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class MotorOdometryNode : public rclcpp::Node
{
public:
  MotorOdometryNode()
  : Node("motor_odometry_node"),
    serial_fd_(-1),
    x_(0.0), y_(0.0), th_(0.0),
    left_pos_(0.0), right_pos_(0.0),
    v_lin_(0.0), v_ang_(0.0),
    has_last_time_(false)
  {
    // ---- Parameters ----
    port_             = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_             = this->declare_parameter<int>("baud", 115200);
    wheel_radius_     = this->declare_parameter<double>("wheel_radius", 0.065);
    wheel_base_       = this->declare_parameter<double>("wheel_base", 0.305);
    ticks_per_rev_    = this->declare_parameter<int>("ticks_per_rev", 522);
    max_wheel_radps_  = this->declare_parameter<double>("max_wheel_radps", 12.0);
    publish_tf_       = this->declare_parameter<bool>("publish_tf", true);
    invert_left_      = this->declare_parameter<bool>("invert_left", false);
    invert_right_     = this->declare_parameter<bool>("invert_right",false);


    // ---- Serial ----
    openSerial();

    // ---- ROS I/O ----
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&MotorOdometryNode::cmdVelCb, this, std::placeholders::_1));

    odom_pub_  = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    if (publish_tf_)
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // Timer: read serial + publish
    timer_ = this->create_wall_timer(50ms, std::bind(&MotorOdometryNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "Motor odometry node started: port=%s baud=%d", port_.c_str(), baud_);
  }

  ~MotorOdometryNode() override
  {
    if (serial_fd_ >= 0) {
      std::string stop = "R0,L0\n";
      (void)write(serial_fd_, stop.c_str(), stop.size());
      close(serial_fd_);
    }
  }

private:
  // Serial initialization
  void openSerial()
  {
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial port %s: %s", port_.c_str(), strerror(errno));
      throw std::runtime_error("serial open failed");
    }

    termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_FATAL(get_logger(), "tcgetattr failed: %s", strerror(errno));
      throw std::runtime_error("tcgetattr failed");
    }

    speed_t spd = B115200;
    switch (baud_) {
      case 9600: spd = B9600; break;
      case 19200: spd = B19200; break;
      case 38400: spd = B38400; break;
      case 57600: spd = B57600; break;
      case 115200: spd = B115200; break;
      default:
        spd = B115200; break;
    }

    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_FATAL(get_logger(), "tcsetattr failed: %s", strerror(errno));
      throw std::runtime_error("tcsetattr failed");
    }
  }

  void writeCommand(double left_w_radps, double right_w_radps)
  {
    double l = std::clamp(left_w_radps  / max_wheel_radps_, -1.0, 1.0);
    double r = std::clamp(right_w_radps / max_wheel_radps_, -1.0, 1.0);
    if (invert_left_)  l = -l;
    if (invert_right_) r = -r;

    std::ostringstream ss;
    ss.setf(std::ios::fixed); ss.precision(3);
    ss << "R" << r << ",L" << l << "\n";
    const std::string out = ss.str();

    if (serial_fd_ >= 0)
      (void)write(serial_fd_, out.c_str(), out.size());
  }

  void readSerialAndProcess()
  {
    if (serial_fd_ < 0) return;

    char buf[256];
    for (;;) {
      int n = read(serial_fd_, buf, sizeof(buf));
      if (n > 0) rx_buffer_.append(buf, n);
      else break;
    }

    size_t pos;
    while ((pos = rx_buffer_.find('\n')) != std::string::npos) {
      std::string line = rx_buffer_.substr(0, pos);
      if (!line.empty() && line.back() == '\r') line.pop_back();
      rx_buffer_.erase(0, pos + 1);
      handleLine(line);
    }
  }

  void handleLine(const std::string &line)
  {
    if (line.rfind("ENC", 0) == 0) {
      long lt=0, rt=0;
      if (sscanf(line.c_str(), "ENC,%ld,%ld", &lt, &rt) == 2)
        onEncoder(lt, rt);
    }
  }

  void cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    v_lin_ = msg->linear.x;
    v_ang_ = msg->angular.z;

    double v_r = (2.0 * v_lin_ - v_ang_ * wheel_base_) / (2.0 * wheel_radius_);
    double v_l = (2.0 * v_lin_ + v_ang_ * wheel_base_) / (2.0 * wheel_radius_);

    writeCommand(v_l, v_r);
  }

  void onTimer()
  {
    readSerialAndProcess();
    publishJointStates(this->get_clock()->now());
  }

  void onEncoder(long left_ticks, long right_ticks)
  {
    rclcpp::Time now = this->get_clock()->now();
    if (!has_last_time_) {
      last_time_ = now;
      prev_left_ticks_ = left_ticks;
      prev_right_ticks_ = right_ticks;
      has_last_time_ = true;
      return;
    }

    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) {
      last_time_ = now;
      prev_left_ticks_ = left_ticks;
      prev_right_ticks_ = right_ticks;
      return;
    }

    long dL_ticks = left_ticks  - prev_left_ticks_;
    long dR_ticks = right_ticks - prev_right_ticks_;
    prev_left_ticks_  = left_ticks;
    prev_right_ticks_ = right_ticks;
    last_time_ = now;

    double meters_per_tick = (2.0 * M_PI * wheel_radius_) / static_cast<double>(ticks_per_rev_);
    double dL = dL_ticks * meters_per_tick;
    double dR = dR_ticks * meters_per_tick;

    if (invert_left_)  dL = -dL;
    if (invert_right_) dR = -dR;

    double d_center = 0.5 * (dL + dR);
    double d_theta  = (dL - dR) / wheel_base_;

    double th_mid = th_ + 0.5 * d_theta;
    x_  += d_center * std::cos(th_mid);
    y_  += d_center * std::sin(th_mid);
    th_ += d_theta;

    double v_lin_est = d_center / dt;
    double v_ang_est = d_theta  / dt;

    publishOdom(now, v_lin_est, v_ang_est);
    updateWheelPositions(dt, v_lin_est, v_ang_est);
    publishJointStates(now);
    if (publish_tf_) publishTF(now);
  }

  void publishOdom(const rclcpp::Time &stamp, double v_lin, double v_ang)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, th_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_footprint";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = tf2::toMsg(q);
    odom.twist.twist.linear.x = v_lin;
    odom.twist.twist.angular.z = v_ang;

    odom_pub_->publish(odom);
  }

  void publishTF(const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id  = "base_footprint";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, th_);
    t.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(t);
  }

  void updateWheelPositions(double dt, double v_lin, double v_ang)
  {
    double w_l = (2.0 * v_lin - v_ang * wheel_base_) / (2.0 * wheel_radius_);
    double w_r = (2.0 * v_lin + v_ang * wheel_base_) / (2.0 * wheel_radius_);

    left_pos_  += w_l * dt;
    right_pos_ += w_r * dt;
  }

  void publishJointStates(const rclcpp::Time &stamp)
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = {"left_wheel_joint", "right_wheel_joint"};
    js.position = {left_pos_, right_pos_};
    joint_pub_->publish(js);
  }

  // ==== Members ====
  std::string port_;
  int baud_;
  double wheel_radius_;
  double wheel_base_;
  int ticks_per_rev_;
  double max_wheel_radps_;
  bool publish_tf_;
  bool invert_left_;
  bool invert_right_;

  int serial_fd_;
  std::string rx_buffer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  double x_, y_, th_;
  double left_pos_, right_pos_;
  double v_lin_, v_ang_;

  long prev_left_ticks_  = 0;
  long prev_right_ticks_ = 0;

  rclcpp::Time last_time_;
  bool has_last_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
