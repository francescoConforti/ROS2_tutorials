#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/msg/turtle_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

/*  **************************************
          THIS DOES NOT WORK!!!!
***************************************** */

class CatcherNode : public rclcpp::Node {
public:
  CatcherNode() : Node("catcher") {
    vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/turtle1/cmd_vel", 10);
    pose_subscriber = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10,
        std::bind(&CatcherNode::update_pose, this, std::placeholders::_1));
    spawn_subscriber =
        this->create_subscription<my_robot_interfaces::msg::TurtleInfo>(
            "/spawn", 10,
            std::bind(&CatcherNode::update_turtles, this,
                      std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Catcher Bot has been started.");
  }

private:
  void update_pose(const turtlesim::msg::Pose::SharedPtr data);
  void update_turtles(const my_robot_interfaces::msg::TurtleInfo data);
  double euclidean_distance(turtlesim::msg::Pose goal_pose);
  bool isCloser(my_robot_interfaces::msg::TurtleInfo turtle1,
                my_robot_interfaces::msg::TurtleInfo turtle2);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber;
  rclcpp::Subscription<my_robot_interfaces::msg::TurtleInfo>::SharedPtr
      spawn_subscriber;
  turtlesim::msg::Pose pose = turtlesim::msg::Pose();
  //std::vector<my_robot_interfaces::msg::TurtleInfo> turtles;
};

void CatcherNode::update_pose(const turtlesim::msg::Pose::SharedPtr data) {
  this->pose.x = data->x;
  this->pose.y = data->y;
  this->pose.theta = data->theta;

  // TODO: kill turtle if main one is in the same position

  RCLCPP_INFO(this->get_logger(), "New pose: %f %f", this->pose.x,
              this->pose.y);
}

void CatcherNode::update_turtles(my_robot_interfaces::msg::TurtleInfo data) {
  //turtles.push_back(data);
  // std::sort(turtles.begin(), turtles.end(), &CatcherNode::isCloser);
  RCLCPP_INFO(this->get_logger(), "Sorted turtles: %s", data.name);
  /*for (auto t : turtles) {
    RCLCPP_INFO(this->get_logger(), "%s", t.name);
  }*/
}

double CatcherNode::euclidean_distance(turtlesim::msg::Pose goal_pose) {
  return sqrt(pow(goal_pose.x - this->pose.x, 2) +
              pow(goal_pose.y - this->pose.y, 2));
}

bool CatcherNode::isCloser(my_robot_interfaces::msg::TurtleInfo turtle1,
                           my_robot_interfaces::msg::TurtleInfo turtle2) {
  auto pose = turtlesim::msg::Pose();
  double distance1, distance2;
  bool first_is_closer = true;
  pose.x = turtle1.x;
  pose.y = turtle1.y;
  distance1 = this->euclidean_distance(pose);
  pose.x = turtle2.x;
  pose.y = turtle2.y;
  distance2 = this->euclidean_distance(pose);
  if (distance2 < distance1) {
    first_is_closer = false;
  }
  return first_is_closer;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CatcherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}