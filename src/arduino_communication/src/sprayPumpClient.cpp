#include <rclcpp/rclcpp.hpp>
#include "arduino_communication/srv/leaf_command.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("leaf_spray_node");
    auto client = node->create_client<arduino_communication::srv::LeafCommand>("send_command");

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "Waiting for leafServerNode...");
    }

    auto request = std::make_shared<arduino_communication::srv::LeafCommand::Request>();
    request->command = "SPRAY_LEAF"; 
    auto result_future = client->async_send_request(request);

    rclcpp::spin(node);
    rclcpp::shutdown();
}

