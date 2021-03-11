#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MoveTurtleNode : public rclcpp::Node {
public:
  MoveTurtleNode() : Node("move_turtle") {
    publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/turtle1/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Move Turtle has been started.");
    std::cout << "Let's move your robot.\nInput your speed: ";
    std::cin >> speed;
    std::cout << "Type your distance: ";
    std::cin >> distance;
    std::cout << "Forward?(0 for false, 1 for true) ";
    std::cin >> isForward;
    move(speed, distance, isForward);
  }

private:
  
  //move the robot straight
  void move(double speed, double distance, bool isForward);
    
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
  float speed, distance;
  bool isForward;
};

void MoveTurtleNode::move(double speed, double distance, bool isForward){
  auto msg = geometry_msgs::msg::Twist();
  if(isForward){
    msg.linear.x = abs(speed);
  }
  else{
    msg.linear.x = -abs(speed);
  }
  //x-axis only
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;

  int t0 = this->get_clock()->now().seconds();
  int current_distance = 0;
  //I keep publishing the velocity until i reach the desired distance (x = vt)
  while(current_distance < distance){
    publisher->publish(msg);
    int t1 = this->get_clock()->now().seconds();
    current_distance = speed * (t1-t0);
  }
  // stop the robot
  msg.linear.x = 0;
  publisher->publish(msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveTurtleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}