#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RotateTurtleNode : public rclcpp::Node {
public:
  RotateTurtleNode() : Node("rotate_turtle") {
    publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/turtle1/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Rotate Turtle has been started.");
    std::cout << "Let's rotate your robot.\nInput your speed (degrees/sec): ";
    std::cin >> speed;
    std::cout << "Type your distance (degrees): ";
    std::cin >> angle;
    std::cout << "Clockwise?(0 for false, 1 for true) ";
    std::cin >> clockwise;
    //Converting from angles to radians
    angular_speed = speed*2*PI/360;
    relative_angle = angle*2*PI/360;
    rotate(angular_speed, relative_angle, clockwise);
  }

private:
  
  //move the robot straight
  void rotate(double angular_speed, double relative_angle, bool clockwise);
    
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
  float speed, angle, angular_speed, relative_angle;
  bool clockwise;
  const double PI = 3.1415926535897;
};

void RotateTurtleNode::rotate(double angular_speed, double relative_angle, bool clockwise){
  auto vel_msg = geometry_msgs::msg::Twist();
  //We wont use linear components
  vel_msg.linear.x = 0;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;

  if(clockwise){
    vel_msg.angular.z = -abs(angular_speed);
  }
  else{
    vel_msg.angular.z = abs(angular_speed);
  }
  int t0 = this->get_clock()->now().seconds();
  double current_angle = 0;

  while(current_angle < relative_angle){
    publisher->publish(vel_msg);
    int t1 = this->get_clock()->now().seconds();
    current_angle = angular_speed * (t1-t0);
  }

  vel_msg.angular.z = 0;
  publisher->publish(vel_msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RotateTurtleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}