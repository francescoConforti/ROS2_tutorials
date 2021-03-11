#include "cmath"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

/* *************************************************************************************
    Keeping this just as history.
    For a version that actually works well, check the turtlesim_project_cpp package
    or turtlesim_catch_them_all for the python version
************************************************************************************** */

class TurtleBotNode : public rclcpp::Node {
public:
  TurtleBotNode() : Node("turtle_bot") {
    vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/turtle1/cmd_vel", 10);
    pose_subscriber = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10,
        std::bind(&TurtleBotNode::update_pose, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Turtle Bot has been started.");
    std::cout << "Set your x goal: ";
    std::cin >> goal_pose.x;
    std::cout << "Set your y goal: ";
    std::cin >> goal_pose.y;
    timer = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&TurtleBotNode::move2goal, this));
  }

private:
  void move2goal();
  void update_pose(const turtlesim::msg::Pose::SharedPtr data);
  double euclidean_distance(turtlesim::msg::Pose goal_pose);
  double linear_velocity(turtlesim::msg::Pose goal_pose, double constant = 2);
  double steering_angle(turtlesim::msg::Pose goal_pose);
  double angular_velocity(turtlesim::msg::Pose goal_pose, double constant = 5);

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber;
  turtlesim::msg::Pose pose = turtlesim::msg::Pose();
  turtlesim::msg::Pose goal_pose = turtlesim::msg::Pose();
  float distance_tolerance = 0.5;
};

void TurtleBotNode::update_pose(const turtlesim::msg::Pose::SharedPtr data) {
  this->pose.x = data->x;
  this->pose.y = data->y;
  RCLCPP_INFO(this->get_logger(), "New pose: %f %f", this->pose.x,
              this->pose.y);
}

double TurtleBotNode::euclidean_distance(turtlesim::msg::Pose goal_pose) {
  return sqrt(pow(goal_pose.x - this->pose.x, 2) +
              pow(goal_pose.y - this->pose.y, 2));
}

double TurtleBotNode::linear_velocity(turtlesim::msg::Pose goal_pose,
                                      double constant) {
  return constant * this->euclidean_distance(goal_pose);
}

double TurtleBotNode::steering_angle(turtlesim::msg::Pose goal_pose) {
  auto goal_theta = atan2(goal_pose.y - this->pose.y, goal_pose.x - this->pose.x);
  auto diff = goal_theta - this->pose.theta;
  if(diff > M_PI){
    diff -= 2*M_PI;
  }
  else if(diff < -M_PI){
    diff += 2*M_PI;
  }
  return diff;
}

double TurtleBotNode::angular_velocity(turtlesim::msg::Pose goal_pose,
                                       double constant) {
  return constant * (this->steering_angle(goal_pose));
}

void TurtleBotNode::move2goal() {
  // Porportional controller.
  // https://en.wikipedia.org/wiki/Proportional_control
  // loop is unpacked to enable use of wall_timer
  // Linear velocity in the x-axis.
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = this->linear_velocity(this->goal_pose);
  msg.linear.y = 0;
  msg.linear.z = 0;
  // Angular velocity in the z-axis.
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = this->angular_velocity(this->goal_pose);
  if (this->euclidean_distance(this->goal_pose) < distance_tolerance) {
    msg.linear.x = 0;
    msg.angular.z = 0;
    this->timer->cancel();
  }
  this->vel_publisher->publish(msg);
}

void my_handler(int s) {
  std::cout << "Caught signal: " << s << "\n";
  exit(1);
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleBotNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}