#include "rclcpp/rclcpp.hpp"

class AGVNode : public rclcpp::Node {
public:
  AGVNode() : Node("agv_node") {
    RCLCPP_INFO(this->get_logger(), "AGV Node started");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AGVNode>());
  rclcpp::shutdown();
  return 0;
}
