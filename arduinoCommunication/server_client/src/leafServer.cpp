#include <rclcpp/rclcpp.hpp>
#include "server_client/srv/leaf_command.hpp"
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class leafServer : public rclcpp::Node
{
public:
    leafServer() : Node("leafServerNode"), vacuum_on(false), spray_on(false)
    {
        serial_port_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            return;
        }

        struct termios tty;
        memset(&tty, 0, sizeof tty);

        if (tcgetattr(serial_port_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
            return;
        }

        // Baud rate
        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);

        // 8N1 mode
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

        // Raw mode (IMPORTANT!)
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_oflag &= ~OPOST;

        tcsetattr(serial_port_, TCSANOW, &tty);
        tcflush(serial_port_, TCIOFLUSH);

        // Arduino reboot delay
        sleep(2);


        command_service_ = this->create_service<server_client::srv::LeafCommand>(
            "send_command",
            std::bind(&leafServer::handle_command, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "leafServer ready. Listening for commands.");
    }

    ~leafServer()
    {
        if (serial_port_ >= 0) {
            close(serial_port_);
        }
    }

private:
    rclcpp::Service<server_client::srv::LeafCommand>::SharedPtr command_service_;
    int serial_port_;
    bool vacuum_on;
    bool spray_on;

    void send_to_arduino(const std::string &cmd)
    {
        std::string full_cmd = cmd + "\r\n";   // CRLF
        write(serial_port_, full_cmd.c_str(), full_cmd.length());
        tcdrain(serial_port_); // ensure data fully sent
        RCLCPP_INFO(this->get_logger(), "Sent to Arduino: %s", cmd.c_str());
    }


    void handle_command(
        const std::shared_ptr<server_client::srv::LeafCommand::Request> request,
        std::shared_ptr<server_client::srv::LeafCommand::Response> response)
    {
        const std::string &cmd = request->command;
        RCLCPP_INFO(this->get_logger(), "Command received: %s", cmd.c_str());

        if (cmd == "VACUUM_ON") {
            if (!vacuum_on) {
                send_to_arduino("VACUUM_ON");
                vacuum_on = true;
            }
        } 
        else if (cmd == "VACUUM_OFF") {
            if (vacuum_on) {
                send_to_arduino("VACUUM_OFF");
                vacuum_on = false;
            }
        } 
        else if (cmd == "SPRAY_ON") {
            if (!spray_on) {
                send_to_arduino("SPRAY_ON");
                spray_on = true;
            }
        } 
        else if (cmd == "SPRAY_OFF") {
            if (spray_on) {
                send_to_arduino("SPRAY_OFF");
                spray_on = false;
            }
        } 
        else {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", cmd.c_str());
        }

        response->response = "Command executed: " + cmd;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<leafServer>());
    rclcpp::shutdown();
    return 0;
}