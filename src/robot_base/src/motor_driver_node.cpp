#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <cerrno>
#include <algorithm> // For std::clamp

class MotorDriverNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Motor Driver Node object
     */
    MotorDriverNode() : Node("motor_driver_node"), serial_fd_(-1)
    {
        // --- Parameters ---
        // Declare parameters with default values. These can be overridden in a launch file.
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 9600);
        this->declare_parameter<double>("wheel_base", 0.15); // Distance between the two wheels in meters
        this->declare_parameter<int>("pwm_range", 255);      // The max PWM value for the motors (0-255 for L298N)

        // Get the parameter values
        this->get_parameter("port", port_);
        this->get_parameter("baudrate", baudrate_);
        this->get_parameter("wheel_base", wheel_base_);
        this->get_parameter("pwm_range", pwm_range_);

        // --- Serial Port Setup ---
        openSerialPort();

        // --- ROS Subscription ---
        // Subscribe to the /cmd_vel topic to receive velocity commands.
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorDriverNode::cmdVelCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Motor driver node started successfully.");
    }

    /**
     * @brief Destroy the Motor Driver Node object
     */
    ~MotorDriverNode()
    {
        // Ensure the serial port is closed when the node is shut down.
        if (serial_fd_ != -1)
        {
            // Send a final stop command to the Arduino before closing.
            RCLCPP_INFO(this->get_logger(), "Sending stop command and closing serial port.");
            write(serial_fd_, "R0,L0\n", 6);
            close(serial_fd_);
        }
    }

private:
    // --- Member Variables ---
    std::string port_;
    int baudrate_;
    double wheel_base_;
    int pwm_range_;
    int serial_fd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    /**
     * @brief Opens and configures the serial port for communication.
     */
    void openSerialPort()
    {
        // Open the serial port in read-write mode, non-blocking.
        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s", port_.c_str(), strerror(errno));
            return;
        }

        struct termios tty;
        memset(&tty, 0, sizeof tty);

        if (tcgetattr(serial_fd_, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return;
        }

        // Set Baud Rate
        cfsetospeed(&tty, baudrateToConstant(baudrate_));
        cfsetispeed(&tty, baudrateToConstant(baudrate_));

        // Standard Serial Configuration (8N1)
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
        tty.c_iflag &= ~IGNBRK;                     // Ignore break signal
        tty.c_lflag = 0;                            // No signaling chars, no echo
        tty.c_oflag = 0;                            // No remapping, no delays
        tty.c_cc[VMIN] = 0;                         // Read doesn't block
        tty.c_cc[VTIME] = 5;                        // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
        tty.c_cflag |= (CLOCAL | CREAD);       // Ignore modem controls, enable reading
        tty.c_cflag &= ~(PARENB | PARODD);     // No parity
        tty.c_cflag &= ~CSTOPB;                // 1 stop bit
        tty.c_cflag &= ~CRTSCTS;               // No hardware flow control

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %d baud", port_.c_str(), baudrate_);
        }
    }

    /**
     * @brief Converts an integer baudrate to its termios constant.
     * @param baudrate The baudrate as an integer (e.g., 9600).
     * @return The termios speed_t constant (e.g., B9600).
     */
    speed_t baudrateToConstant(int baudrate)
    {
        switch (baudrate)
        {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            default:
                RCLCPP_WARN(this->get_logger(), "Unsupported baudrate %d. Defaulting to 9600.", baudrate);
                return B9600;
        }
    }

    /**
     * @brief Callback function for the /cmd_vel subscription.
     * Calculates wheel speeds and sends them to the Arduino.
     * @param msg The received geometry_msgs::msg::Twist message.
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (serial_fd_ == -1)
        {
            // Throttle warning to avoid flooding the console
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Serial not available. Skipping cmd_vel command.");
            return;
        }

        // Extract linear and angular velocities from the message
        double linear_vel = msg->linear.x;
        double angular_vel = msg->angular.z;

        // --- Differential Drive Kinematics ---
        // Calculate the velocity for each wheel
        // v_right = linear + (angular * wheel_base / 2)
        // v_left  = linear - (angular * wheel_base / 2)
        // These velocities are in a conceptual range, not m/s yet.
        // We will map them directly to the PWM range.
        double right_vel = linear_vel + (angular_vel * wheel_base_ / 2.0);
        double left_vel = linear_vel - (angular_vel * wheel_base_ / 2.0);

        // --- Map Velocities to PWM values ---
        // The Twist message gives velocities typically from -1.0 to 1.0.
        // We scale this to our motor's PWM range.
        int right_pwm = static_cast<int>(right_vel * pwm_range_);
        int left_pwm = static_cast<int>(left_vel * pwm_range_);

        // --- Clamp PWM values ---
        // Ensure the PWM values are within the allowed range [-pwm_range, pwm_range]
        right_pwm = std::clamp(right_pwm, -pwm_range_, pwm_range_);
        left_pwm = std::clamp(left_pwm, -pwm_range_, pwm_range_);

        // --- Build and Send Command ---
        // Create the command string in the format "R<speed>,L<speed>\n"
        // Example: "R255,L255\n" or "R-100,L100\n"
        std::ostringstream cmd_stream;
        cmd_stream << "R" << right_pwm << ",L" << left_pwm << "\n";
        std::string cmd_str = cmd_stream.str();

        int bytes_written = write(serial_fd_, cmd_str.c_str(), cmd_str.length());

        if (bytes_written < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to serial: %s", strerror(errno));
        }
        else
        {
            // Log the sent command for debugging purposes
            RCLCPP_DEBUG(this->get_logger(), "Sent command: %s", cmd_str.c_str());
        }
    }
};

/**
 * @brief Main function to initialize and run the ROS 2 node.
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorDriverNode>());
    rclcpp::shutdown();
    return 0;
}
