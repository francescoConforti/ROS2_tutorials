#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "my_robot_interfaces/msg/turtle_info.hpp"
#include "cstdlib"
#include "ctime"

class SpawnerNode : public rclcpp::Node {
public:
  SpawnerNode() : Node("spawner") {
    this->declare_parameter("frequency", 0.5);
    srand (static_cast <unsigned> (time(0)));
    spawn_frequency = this->get_parameter("frequency").as_double();
    publisher_ = this->create_publisher<my_robot_interfaces::msg::TurtleInfo>(
        "/spawn", 10);
    timer = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000 / spawn_frequency)),
        std::bind(&SpawnerNode::callSpawnService, this));
  }

private:
  void callSpawnService();
  void spawn();

  float spawn_frequency;
  rclcpp::Publisher<my_robot_interfaces::msg::TurtleInfo>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer;
  std::vector<std::thread> threads_;
};

void SpawnerNode::callSpawnService(){
  this->threads_.push_back(std::thread(std::bind(&SpawnerNode::spawn, this)));
}

void SpawnerNode::spawn(){
  auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
  while(!client->wait_for_service(std::chrono::seconds(1))){
    RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up....");
  }

  auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
  request->x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/10.0));
  request->y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/10.0));
  request->theta = (-3.0) * static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/6.0));

  auto future = client->async_send_request(request);

  try{
    auto response = future.get();
    auto msg = my_robot_interfaces::msg::TurtleInfo();
    msg.name = response->name;
    msg.x = request->x;
    msg.y = request->y;
    publisher_->publish(msg);
  }
  catch(const std::exception &e){
    RCLCPP_ERROR(this->get_logger(), "Service call failed.");
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpawnerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}