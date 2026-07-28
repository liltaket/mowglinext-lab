#include "rclcpp/rclcpp.hpp"
#include "mowgli_safety/safety_supervisor_node.hpp"
int main(int argc, char ** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<mowgli_safety::SafetySupervisorNode>()); rclcpp::shutdown(); return 0; }
