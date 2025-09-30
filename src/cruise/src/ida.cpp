#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


struct node
{
    int x;
    int y;
    int h_cost;
    int g_cost;
    int f_cost;

    node(int x=0, int y=0)
        : x(x), y(y), h_cost(0), g_cost(0), f_cost(0) {}
};

class IDAStarPlanner: public rclcpp::Node
{
public:
    const int FOUND = -1;
    const int INF = 1e9;

    IDAStarPlanner() : Node("IDAStarPlanner"){
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("explored_nodes", 10);
    }

    void setMap(const nav_msgs::msg::OccupancyGrid &map)
    {
        width_ = map.info.width;
        height_ = map.info.height;
        resolution_ = map.info.resolution;
        origin_x_ = map.info.origin.position.x;
        origin_y_ = map.info.origin.position.y;

        grid_.assign(height_, std::vector<int>(width_, 0));
        for (size_t y = 0; y < height_; y++)
        {
            for (size_t x = 0; x < width_; x++)
            {
                int idx = y * width_ + x;
                grid_[y][x] = (map.data[idx] > 50 ? 1 : 0); // occupied if >50
                if(grid_[y][x] == 0 && (start_.x == 0 && start_.y == 0))
                {
                    start_.x = x;
                    start_.y = y;
                    RCLCPP_INFO(rclcpp::get_logger("IDAStarPlanner"),"Start set to %d,%d",start_.x,start_.y);
                }
                    
            }
        }
    }

    void setStartGoal(const node &start, const node &goal)
    {
        start_ = start;
        goal_ = goal;
    }

    int dfs(node curr, int threshold) {
        int f = curr.g_cost + h_c(curr);
        if (f > threshold) return f;
        if (curr.x == goal_.x && curr.y == goal_.y) {
            path_.push_back(curr);
            return FOUND;
        }

        int min_cost = INF;
        visited_[curr.y][curr.x] = true;
        path_.push_back(curr);

        for (auto d : dir_) {
            int new_x = curr.x + d[0];
            int new_y = curr.y + d[1];
            if (new_x < 0 || new_x >= width_ || new_y < 0 || new_y >= height_) continue;
            if (grid_[new_y][new_x] == 1) continue;
            if (visited_[new_y][new_x]) continue;

            node neighbor(new_x, new_y);
            neighbor.g_cost = curr.g_cost + ((d[0] == 0 || d[1] == 0) ? 1 : sqrt(2));

            int t = dfs(neighbor, threshold);
            if (t == FOUND) return FOUND;
            if (t < min_cost) min_cost = t;
        }

        path_.pop_back();
        visited_[curr.y][curr.x] = false;
        return min_cost;
        }

    std::vector<node> plan() {
        visited_.assign(height_, std::vector<bool>(width_, false));
        path_.clear();

        start_.g_cost = 0;
        start_.h_cost = h_c(start_);
        start_.f_cost = f_c(start_);

        int threshold = start_.f_cost;

        while (true) {
            int temp = dfs(start_, threshold);
            if (temp == FOUND) return path_;
            if (temp == INF) return {}; // no path
            threshold = temp;
        }
    }

private:
    int width_, height_;
    double resolution_, origin_x_, origin_y_;
    std::vector<std::vector<int>> grid_;
    std::vector<std::vector<bool>> visited_;
    std::vector<node> path_;
    node start_, goal_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    


    std::vector<std::vector<int>> dir_ = {
        {1, 0}, {0, 1}, {-1, 0}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    float f_c(const node &curr)
    {
        return curr.g_cost + curr.h_cost;
    }

    float h_c(const node &curr)
    {
        return std::abs(curr.x - goal_.x) + std::abs(curr.y - goal_.y);
    }

    float g_c(const node &curr)
    {
        return std::abs(curr.x - start_.x) + std::abs(curr.y - start_.y);
    }

    bool explore(node curr)
    {
        int thresh = curr.f_cost;
        visited_[curr.y][curr.x] = true;
        this->publishMarker(curr);
        path_.push_back(curr);

        if (curr.x == goal_.x && curr.y == goal_.y)
        {
            std::cout << "Goal found at: " << curr.x << "," << curr.y << std::endl;
            return true;
        }

        for (auto d : dir_)
        {
            int new_x = curr.x + d[0];
            int new_y = curr.y + d[1];
            if (new_x < 0 || new_x >= (int)width_ || new_y < 0 || new_y >= (int)height_)
                continue;
            if (grid_[new_y][new_x] == 1) // obstacle
                continue;
            if (visited_[new_y][new_x])
                continue;

            node neighbor(new_x, new_y);
            neighbor.g_cost = curr.g_cost + ((d[0] == 0 || d[1] == 0) ? 1 : sqrt(2));
            neighbor.h_cost = h_c(neighbor);
            neighbor.f_cost = f_c(neighbor);

            if (neighbor.f_cost > thresh)
                continue;

            if (explore(neighbor))
                return true;
        }

        path_.pop_back();
        return false;
    }

    void publishMarker(const node &n)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";   // must match your TF frame
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "ida_star";
        marker.id = n.y * width_ + n.x;   // unique ID per node
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = origin_x_ + n.x * resolution_ + resolution_/2.0;
        marker.pose.position.y = origin_y_ + n.y * resolution_ + resolution_/2.0;
        marker.pose.position.z = 0.05;
        marker.scale.x = resolution_;
        marker.scale.y = resolution_;
        marker.scale.z = 0.01;
        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        visualization_msgs::msg::MarkerArray arr;
        arr.markers.push_back(marker);
        marker_pub_->publish(arr);
    }

};

class PathPlannerNode : public rclcpp::Node
{
public:
    
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));

    PathPlannerNode()
    : Node("path_planner_node")
    {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", qos_profile,
            std::bind(&PathPlannerNode::mapCallback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", qos_profile,
            std::bind(&PathPlannerNode::goalCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

        RCLCPP_INFO(this->get_logger(), "Path Planner Node started with IDA*.");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),"Received new map");
        latest_map_ = msg;
        planner_.setMap(*msg);
        tryPlanPath();
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),"Received new goal %f %f",msg->pose.position.x,msg->pose.position.y);
        latest_goal_ = msg;
        tryPlanPath();
    }

    void tryPlanPath()
    {
        if (!latest_map_ || !latest_goal_)
            return;

        // Convert world goal to grid indices
        int gx = (latest_goal_->pose.position.x - latest_map_->info.origin.position.x) / latest_map_->info.resolution;
        int gy = (latest_goal_->pose.position.y - latest_map_->info.origin.position.y) / latest_map_->info.resolution;

        node start(0, 0); 
        node goal(gx, gy);

        planner_.setStartGoal(start, goal);
        auto nodes = planner_.plan();

        if (nodes.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No path found to the goal.");
            return;
        }

        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "map";

        for (auto &n : nodes)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = n.x * latest_map_->info.resolution + latest_map_->info.origin.position.x;
            pose.pose.position.y = n.y * latest_map_->info.resolution + latest_map_->info.origin.position.y;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }

        path_pub_->publish(path);
        RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", path.poses.size());
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    geometry_msgs::msg::PoseStamped::SharedPtr latest_goal_;

    IDAStarPlanner planner_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
