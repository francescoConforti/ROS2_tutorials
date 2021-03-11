#include "rclcpp/rclcpp.hpp"

class MyCustomNode : public rclcpp::Node { // CHANGE NAME
public:
  MyCustomNode() : Node("node_name") { // CHANGE NAME
  }

private:
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyCustomNode>(); // CHANGE NAME
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}