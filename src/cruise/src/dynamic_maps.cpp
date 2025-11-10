#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include <random>
#include <vector>
#include <cstdlib>

using namespace std::chrono_literals;

class DynamicMapPublisher : public rclcpp::Node
{
public:
  DynamicMapPublisher()
  : Node("dynamic_map_publisher"),
    width_(500),
    height_(500),
    resolution_(0.1),
    robot_x_(0),
    robot_y_(0),
    goal_x_(width_ - 1),
    goal_y_(height_ - 1),
    last_obstacle_time_(this->now())
  {
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&DynamicMapPublisher::timerCallback, this));

    map_data_.assign(width_ * height_, 0);
    RCLCPP_INFO(this->get_logger(), "Dynamic Map Publisher started.");
  }

private:
  void timerCallback()
  {
    rclcpp::Time now = this->now();
    srand(time(0)); 

    int lower_bound = 1;
    int upper_bound = 100;

    int random_number = rand() % (upper_bound - lower_bound + 1) + lower_bound;

    // Add obstacles every 2 seconds
    if ((now - last_obstacle_time_).seconds() > 5.0 && !goal_reached_) {
      map_data_.assign(width_ * height_, 0);
      for(int i = 0; i < random_number; ++i)
        addRandomObstacle();
      last_obstacle_time_ = now;
      publishMap();
    }

    // Move robot toward goal
    if (!goal_reached_) {
      moveRobot();
    }


  }

  void addRandomObstacle()
  {
    if (goal_reached_) return;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> x_dist(0, width_ - 3);
    std::uniform_int_distribution<> y_dist(0, height_ - 3);

    int x = x_dist(gen);
    int y = y_dist(gen);

    // Small 2x2 block of obstacles
    for (int dy = 0; dy < 5; ++dy) {
      for (int dx = 0; dx < 5; ++dx) {
        int idx = (y + dy) * width_ + (x + dx);
        if (idx >= 0 && idx < (int)map_data_.size())
          map_data_[idx] = 100;
      }
    }
    RCLCPP_INFO(this->get_logger(), "Added obstacle near (%d, %d)", x, y);
  }

  void moveRobot()
  {
    // Simple straight-line motion (ignoring obstacles)
    if (robot_x_ < goal_x_) robot_x_++;
    if (robot_y_ < goal_y_) robot_y_++;

    if (robot_x_ == goal_x_ && robot_y_ == goal_y_) {
      goal_reached_ = true;
      RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping updates.");
    }
  }

  void publishMap()
  {
    nav_msgs::msg::OccupancyGrid map_msg;
    map_msg.header.frame_id = "map";
    map_msg.header.stamp = this->now();

    map_msg.info.resolution = resolution_;
    map_msg.info.width = width_;
    map_msg.info.height = height_;
    map_msg.info.origin.position.x = 0.0;
    map_msg.info.origin.position.y = 0.0;
    map_msg.info.origin.position.z = 0.0;

    // Copy base map
    std::vector<int8_t> data = map_data_;

    // Mark robot and goal
    int robot_idx = robot_y_ * width_ + robot_x_;
    int goal_idx = goal_y_ * width_ + goal_x_;
    if (robot_idx < (int)data.size()) data[robot_idx] = 50;  // gray
    if (goal_idx < (int)data.size()) data[goal_idx] = 0;     // keep goal free

    map_msg.data = data;

    map_pub_->publish(map_msg);
  }

  // === Variables ===
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int width_, height_;
  double resolution_;
  std::vector<int8_t> map_data_;

  int robot_x_, robot_y_;
  int goal_x_, goal_y_;
  bool goal_reached_ = false;

  rclcpp::Time last_obstacle_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicMapPublisher>());
  rclcpp::shutdown();
  return 0;
}
