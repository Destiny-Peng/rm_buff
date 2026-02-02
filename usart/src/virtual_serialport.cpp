#include "usart/virtual_serialport.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VirtualSerial>("virtual_serial");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}