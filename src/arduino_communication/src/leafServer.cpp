#include <rclcpp/rclcpp.hpp>
#include "arduino_communication/srv/leaf_command.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <sys/stat.h>

class leafServer : public rclcpp::Node
{
public:
    leafServer() : Node("leafServerNode"), vacuum_on(false), spray_on(false), fake_mode_(false)
    {
        // Try to find and open Arduino serial port
        std::vector<std::string> possible_ports = {
            "/dev/ttyUSB0",
            "/dev/ttyUSB1",
            "/dev/ttyUSB2",
            "/dev/ttyACM0",
            "/dev/ttyACM1",
            "/dev/ttyACM2"
        };
        
        serial_port_ = -1;
        std::string opened_port;
        
        for (const auto& port : possible_ports) {
            // Check if port exists
            struct stat st;
            if (stat(port.c_str(), &st) == 0 && S_ISCHR(st.st_mode)) {
                serial_port_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
                if (serial_port_ >= 0) {
                    opened_port = port;
                    RCLCPP_INFO(this->get_logger(), "Found Arduino at: %s", port.c_str());
                    break;
                }
            }
        }
        
        if (serial_port_ < 0) {
            RCLCPP_WARN(this->get_logger(), "⚠️  No Arduino hardware detected. Running in FAKE/SIMULATION mode.");
            RCLCPP_WARN(this->get_logger(), "   Commands will be logged but not sent to hardware.");
            RCLCPP_WARN(this->get_logger(), "   All service functionality will work normally for testing.");
            fake_mode_ = true;
            // Continue initialization without serial port
        } else {
            // Configure serial port if real hardware found

            struct termios tty;
            memset(&tty, 0, sizeof tty);

            if (tcgetattr(serial_port_, &tty) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr on %s", opened_port.c_str());
                close(serial_port_);
                serial_port_ = -1;
                fake_mode_ = true;
                RCLCPP_WARN(this->get_logger(), "Switching to FAKE mode due to serial configuration error.");
            } else {
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

                if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Error setting serial port attributes");
                    close(serial_port_);
                    serial_port_ = -1;
                    fake_mode_ = true;
                    RCLCPP_WARN(this->get_logger(), "Switching to FAKE mode due to serial setup error.");
                } else {
                    tcflush(serial_port_, TCIOFLUSH);
                    // Arduino reboot delay
                    sleep(2);
                }
            }
        }

        // Create service (works in both real and fake mode)
        command_service_ = this->create_service<arduino_communication::srv::LeafCommand>(
            "send_command",
            std::bind(&leafServer::handle_command, this, std::placeholders::_1, std::placeholders::_2));

        // Create marker publisher for RViz visualization
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("pump_status_marker", 10);
        
        // Create timer to update visualization
        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Update every 100ms
            std::bind(&leafServer::update_visualization, this));

        if (fake_mode_) {
            RCLCPP_INFO(this->get_logger(), "✓ leafServer ready in FAKE/SIMULATION mode. Listening for commands.");
            RCLCPP_INFO(this->get_logger(), "  Service '/send_command' is available for testing.");
        } else {
            RCLCPP_INFO(this->get_logger(), "✓ leafServer ready with Arduino hardware at %s. Listening for commands.", opened_port.c_str());
        }
    }

    ~leafServer()
    {
        if (serial_port_ >= 0) {
            close(serial_port_);
        }
        if (fake_mode_) {
            RCLCPP_INFO(this->get_logger(), "leafServer (FAKE mode) shutting down.");
        }
    }

private:
    rclcpp::Service<arduino_communication::srv::LeafCommand>::SharedPtr command_service_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    int serial_port_;
    bool vacuum_on;
    bool spray_on;
    bool fake_mode_;

    void send_to_arduino(const std::string &cmd)
    {
        if (fake_mode_) {
            // In fake mode, just log the command without sending
            RCLCPP_INFO(this->get_logger(), "[FAKE MODE] Would send to Arduino: %s", cmd.c_str());
        } else {
            // Real mode: send command via serial port
            if (serial_port_ >= 0) {
                std::string full_cmd = cmd + "\r\n";   // CRLF
                write(serial_port_, full_cmd.c_str(), full_cmd.length());
                tcdrain(serial_port_); // ensure data fully sent
                RCLCPP_INFO(this->get_logger(), "Sent to Arduino: %s", cmd.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "[FAKE MODE] Serial port not available. Command: %s", cmd.c_str());
            }
        }
    }


    void handle_command(
        const std::shared_ptr<arduino_communication::srv::LeafCommand::Request> request,
        std::shared_ptr<arduino_communication::srv::LeafCommand::Response> response)
    {
        const std::string &cmd = request->command;
        RCLCPP_INFO(this->get_logger(), "Command received: %s", cmd.c_str());

        // Vacuum pump commands
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
        // Spray pump commands
        else if (cmd == "SPRAY_ON" || cmd == "SPRAY_LEAF") {
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
            response->response = "Unknown command: " + cmd;
            return;
        }

        response->response = "Command executed: " + cmd;
        
        // Update visualization when state changes
        update_visualization();
    }
    
    void update_visualization()
    {
        // Create text marker for vacuum pump status
        visualization_msgs::msg::Marker vacuum_marker;
        vacuum_marker.header.frame_id = "base_link";
        vacuum_marker.header.stamp = this->now();
        vacuum_marker.ns = "pump_status";
        vacuum_marker.id = 0;
        vacuum_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        vacuum_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position: top-left area (left side, front, high up)
        vacuum_marker.pose.position.x = -0.5;  // Left side
        vacuum_marker.pose.position.y = 0.5;   // Front
        vacuum_marker.pose.position.z = 1.5;   // High up
        vacuum_marker.pose.orientation.w = 1.0;
        
        // Text content (English)
        std::string vacuum_text = "Vacuum Pump: ";
        vacuum_text += vacuum_on ? "ON" : "OFF";
        vacuum_marker.text = vacuum_text;
        
        // Text properties
        vacuum_marker.scale.z = 0.15;  // Text height
        vacuum_marker.color.a = 1.0;
        vacuum_marker.color.r = vacuum_on ? 0.0 : 1.0;  // Red when OFF
        vacuum_marker.color.g = vacuum_on ? 1.0 : 0.0;  // Green when ON
        vacuum_marker.color.b = 0.0;
        
        marker_pub_->publish(vacuum_marker);
        
        // Create text marker for spray pump status
        visualization_msgs::msg::Marker spray_marker;
        spray_marker.header.frame_id = "base_link";
        spray_marker.header.stamp = this->now();
        spray_marker.ns = "pump_status";
        spray_marker.id = 1;
        spray_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        spray_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position: below vacuum pump status (same x, y, lower z)
        spray_marker.pose.position.x = -0.5;  // Left side (same as vacuum)
        spray_marker.pose.position.y = 0.5;   // Front (same as vacuum)
        spray_marker.pose.position.z = 1.35;  // Slightly lower
        spray_marker.pose.orientation.w = 1.0;
        
        // Text content (English)
        std::string spray_text = "Spray Pump: ";
        spray_text += spray_on ? "ON" : "OFF";
        spray_marker.text = spray_text;
        
        // Text properties
        spray_marker.scale.z = 0.15;  // Text height
        spray_marker.color.a = 1.0;
        spray_marker.color.r = spray_on ? 0.0 : 1.0;  // Red when OFF
        spray_marker.color.g = spray_on ? 1.0 : 0.0;  // Green when ON
        spray_marker.color.b = 0.0;
        
        marker_pub_->publish(spray_marker);
        
        // In fake mode, add a visual indicator
        if (fake_mode_) {
            visualization_msgs::msg::Marker fake_mode_marker;
            fake_mode_marker.header.frame_id = "base_link";
            fake_mode_marker.header.stamp = this->now();
            fake_mode_marker.ns = "pump_status";
            fake_mode_marker.id = 2;
            fake_mode_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            fake_mode_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Position: below spray pump status
            fake_mode_marker.pose.position.x = -0.5;
            fake_mode_marker.pose.position.y = 0.5;
            fake_mode_marker.pose.position.z = 1.20;
            fake_mode_marker.pose.orientation.w = 1.0;
            
            fake_mode_marker.text = "[FAKE/SIMULATION MODE]";
            
            fake_mode_marker.scale.z = 0.12;
            fake_mode_marker.color.a = 1.0;
            fake_mode_marker.color.r = 1.0;  // Yellow/orange
            fake_mode_marker.color.g = 0.65;
            fake_mode_marker.color.b = 0.0;
            
            marker_pub_->publish(fake_mode_marker);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<leafServer>());
    rclcpp::shutdown();
    return 0;
}

