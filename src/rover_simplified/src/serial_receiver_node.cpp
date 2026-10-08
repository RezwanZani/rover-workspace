#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// Strict 1-byte alignment matching Python struct '<cffb' (10 bytes total)
#pragma pack(push, 1)
struct IncomingTelemetry
{
    char sync_char;  // Expected '$' (0x24)
    float linear_x;  // Forward / reverse velocity (m/s)
    float angular_z; // Yaw rate (rad/s)
    int8_t cmd_flag; // 0 = Normal teleop, >0 = Hardware test override
};
#pragma pack(pop)

class SerialReceiverNode : public rclcpp::Node
{
public:
    SerialReceiverNode() : Node("serial_receiver_node"), serial_fd_(-1), is_stopped_(false)
    {
        this->declare_parameter<std::string>("port", "/tmp/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter<double>("watchdog_timeout_sec", 0.5);

        port_name_ = this->get_parameter("port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        std::string topic = this->get_parameter("cmd_vel_topic").as_string();
        watchdog_timeout_sec_ = this->get_parameter("watchdog_timeout_sec").as_double();

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);

        if (!init_serial_port(port_name_, baud_rate_))
        {
            RCLCPP_FATAL(this->get_logger(), "[SERIAL ERROR] Failed to initialize serial port %s", port_name_.c_str());
        }

        last_valid_msg_time_ = this->now();

        // 100 Hz polling timer
        read_timer_ = this->create_wall_timer(
            10ms, std::bind(&SerialReceiverNode::poll_serial, this));

        // 10 Hz watchdog timer
        watchdog_timer_ = this->create_wall_timer(
            100ms, std::bind(&SerialReceiverNode::watchdog_check, this));

        RCLCPP_INFO(this->get_logger(), "=================================================");
        RCLCPP_INFO(this->get_logger(), "Rover Serial Receiver Node Initialized");
        RCLCPP_INFO(this->get_logger(), "Target Port    : %s @ %d baud", port_name_.c_str(), baud_rate_);
        RCLCPP_INFO(this->get_logger(), "Publishing To  : %s", topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Watchdog Cutoff: %.1f seconds", watchdog_timeout_sec_);
        RCLCPP_INFO(this->get_logger(), "=================================================");
    }

    ~SerialReceiverNode() override
    {
        if (serial_fd_ != -1)
        {
            close(serial_fd_);
        }
    }

private:
    std::string port_name_;
    int baud_rate_;
    double watchdog_timeout_sec_;
    int serial_fd_;
    bool is_stopped_;
    rclcpp::Time last_valid_msg_time_;
    std::vector<uint8_t> rx_buffer_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr read_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    bool init_serial_port(const std::string &port, int baud)
    {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "[SERIAL] System error opening %s: %s", port.c_str(), strerror(errno));
            return false;
        }

        fcntl(serial_fd_, F_SETFL, O_NONBLOCK);

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0)
        {
            return false;
        }

        // Force complete raw binary mode
        cfmakeraw(&tty);

        speed_t speed = B115200;
        if (baud == 9600)
            speed = B9600;
        else if (baud == 57600)
            speed = B57600;
        else if (baud == 115200)
            speed = B115200;

        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        tcflush(serial_fd_, TCIOFLUSH);
        return (tcsetattr(serial_fd_, TCSANOW, &tty) == 0);
    }

    void poll_serial()
    {
        if (serial_fd_ == -1)
            return;

        // Drain serial hardware buffer into vector accumulator
        uint8_t temp[256];
        int n = 0;
        while ((n = read(serial_fd_, temp, sizeof(temp))) > 0)
        {
            RCLCPP_INFO(this->get_logger(), "[DEBUG] Read %d bytes from PTY", n);
            if (n >= 1) {
                RCLCPP_INFO(this->get_logger(), "[DEBUG] First byte: %02X ('%c')", temp[0], temp[0]);
            }
            rx_buffer_.insert(rx_buffer_.end(), temp, temp + n);
        }


        // Process frames in buffer
        while (!rx_buffer_.empty())
        {
            // Mode A: Raw 10-byte binary packet starting with '$'
            if (rx_buffer_[0] == '$')
            {
                if (rx_buffer_.size() < sizeof(IncomingTelemetry))
                {
                    // RCLCPP_INFO(this->get_logger(), "[DEBUG] Found $, waiting for 10 bytes. Have %zu", rx_buffer_.size());
                    break; // Wait for full 10 bytes
                }

                IncomingTelemetry payload;
                std::memcpy(&payload, rx_buffer_.data(), sizeof(IncomingTelemetry));
                rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sizeof(IncomingTelemetry));

                process_telemetry(payload);
                bounce_heartbeat_ping(payload.cmd_flag);
                continue;
            }

            // Mode B: XBee API Mode 1 frame starting with 0x7E
            if (rx_buffer_[0] == 0x7E)
            {
                if (rx_buffer_.size() < 3)
                {
                    break; // Wait for length bytes
                }

                uint16_t frame_len = (static_cast<uint16_t>(rx_buffer_[1]) << 8) | rx_buffer_[2];
                size_t total_packet_len = frame_len + 4; // Delimiter(1) + Len(2) + Payload + Checksum(1)

                if (frame_len < 1 || frame_len > 128)
                {
                    RCLCPP_INFO(this->get_logger(), "[DEBUG] Mode B corrupted len %d, dropping 0x7E", frame_len);
                    rx_buffer_.erase(rx_buffer_.begin()); // Corrupted marker, slide window
                    continue;
                }

                if (rx_buffer_.size() < total_packet_len)
                {
                    break; // Wait for full frame
                }

                RCLCPP_INFO(this->get_logger(), "[DEBUG] Mode B full frame received");

                // Checksum validation
                uint8_t sum = 0;
                for (size_t i = 3; i < static_cast<size_t>(3 + frame_len); ++i)
                {
                    sum += rx_buffer_[i];
                }
                uint8_t calculated_checksum = 0xFF - sum;
                uint8_t received_checksum = rx_buffer_[total_packet_len - 1];

                if (calculated_checksum == received_checksum)
                {
                    uint8_t frame_type = rx_buffer_[3];
                    size_t rf_offset = 0;
                    
                    // Search for '$' telemetry inside the validated XBee frame
                    // Frame header is at least 4 bytes. We need enough room for the telemetry struct.
                    if (total_packet_len >= 4 + sizeof(IncomingTelemetry)) 
                    {
                        for (size_t i = 4; i <= total_packet_len - 1 - sizeof(IncomingTelemetry); ++i)
                        {
                            if (rx_buffer_[i] == '$')
                            {
                                rf_offset = i;
                                break;
                            }
                        }
                    }

                    if (rf_offset != 0)
                    {
                        IncomingTelemetry payload;
                        std::memcpy(&payload, &rx_buffer_[rf_offset], sizeof(IncomingTelemetry));
                        process_telemetry(payload);
                        bounce_heartbeat_ping(payload.cmd_flag);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "[DEBUG] Valid XBee Frame (Type: %02X) ignored - No '$' telemetry found.", frame_type);
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "[DEBUG] Mode B checksum failed! %02X != %02X", calculated_checksum, received_checksum);
                }

                rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total_packet_len);
                continue;
            }

            // Slide window past leading noise
            RCLCPP_INFO(this->get_logger(), "[DEBUG] Dropping noise byte: %02X", rx_buffer_[0]);
            rx_buffer_.erase(rx_buffer_.begin());
        }
    }

    void process_telemetry(const IncomingTelemetry &payload)
    {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = static_cast<double>(payload.linear_x);
        // Invert angular_z to fix left/right steering direction
        twist.angular.z = -static_cast<double>(payload.angular_z);

        publisher_->publish(twist);

        last_valid_msg_time_ = this->now();
        is_stopped_ = false;

        RCLCPP_INFO(this->get_logger(),
                    "[RX OK] Sync: '%c' | Lin_X: %6.3f m/s | Ang_Z: %6.3f rad/s | Flag: %2d | -> /cmd_vel published",
                    payload.sync_char, payload.linear_x, payload.angular_z, payload.cmd_flag);
    }

    void bounce_heartbeat_ping(int8_t current_flag)
    {
        // Send 2-byte ping acknowledgment back across bridge
        uint8_t ack[2] = {'#', static_cast<uint8_t>(current_flag)};
        write(serial_fd_, ack, 2);
    }

    void watchdog_check()
    {
        auto elapsed = (this->now() - last_valid_msg_time_).seconds();

        if (elapsed > watchdog_timeout_sec_)
        {
            if (!is_stopped_)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "[WATCHDOG TIMEOUT] No packet received for %.2f seconds! Forcing rover halt (0.0, 0.0).", elapsed);

                auto stop_twist = geometry_msgs::msg::Twist();
                publisher_->publish(stop_twist);
                is_stopped_ = true;
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReceiverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}