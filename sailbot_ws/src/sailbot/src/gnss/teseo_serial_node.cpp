#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>

class TeseoSerialNode : public rclcpp::Node {
public:
    TeseoSerialNode() : Node("teseo_serial_node") {
        declare_parameter<std::string>("port", "/dev/ttyAMA0");
        declare_parameter<int>("baud", 460800);

        port_ = get_parameter("port").as_string();
        baud_ = get_parameter("baud").as_int();

        pub_ = create_publisher<std_msgs::msg::String>("/gnss/nmea_raw", 10);

        if (!open_port()) {
            RCLCPP_FATAL(get_logger(), "Failed to open Teseo GNSS port");
            throw std::runtime_error("Unable to open serial port");
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&TeseoSerialNode::read_loop, this)
        );

        RCLCPP_INFO(get_logger(), "Teseo serial node started.");
    }

private:
    bool open_port() {
        fd_ = ::open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) return false;

        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0) return false;

        cfmakeraw(&tty);

        speed_t speed;
        switch (baud_) {
            case 115200: speed = B115200; break;
            case 460800: speed = B460800; break;
            case 921600: speed = B921600; break;
            default:     speed = B460800; break; // safe default
        }

        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        tcsetattr(fd_, TCSANOW, &tty);
        return true;
    }

    void read_loop() {
        if (fd_ < 0) return;

        char buf[256];
        int n = ::read(fd_, buf, sizeof(buf) - 1);
        if (n <= 0) return;

        for (int i = 0; i < n; ++i) {
            char c = buf[i];
            if (c == '\n') {
                std_msgs::msg::String msg;
                msg.data = line_;
                pub_->publish(msg);
                line_.clear();
            } else if (c != '\r') {
                line_ += c;
            }
        }
    }

    int fd_ = -1;
    std::string port_;
    int baud_;

    std::string line_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeseoSerialNode>());
    rclcpp::shutdown();
    return 0;
}
