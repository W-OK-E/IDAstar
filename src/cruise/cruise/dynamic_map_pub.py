#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Bool
import numpy as np
import time


class DynamicMapPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_map_publisher')

        # Publisher for /map
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # Optional subscriber for goal reached
        self.create_subscription(Bool, '/goal_reached', self.goal_callback, 10)

        # Map parameters
        self.map_width = 50
        self.map_height = 50
        self.map_resolution = 0.1  # meters per cell

        # Internal map: initialize all free (0)
        self.map_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)

        # Control variables
        self.goal_reached = False
        self.last_obstacle_time = time.time()

        # Timer for publishing
        self.create_timer(1.0, self.publish_map)  # publish at 1 Hz

        self.get_logger().info("Dynamic Map Publisher started.")

    def goal_callback(self, msg: Bool):
        self.goal_reached = msg.data
        if self.goal_reached:
            self.get_logger().info("Goal reached! Stopping obstacle updates.")

    def add_random_obstacle(self):
        """Add a random obstacle block to the map."""
        if self.goal_reached:
            return
        x = np.random.randint(0, self.map_width - 2)
        y = np.random.randint(0, self.map_height - 2)
        self.map_data[y:y+2, x:x+2] = 100  # Mark as occupied
        self.get_logger().info(f"Added obstacle at ({x}, {y})")

    def publish_map(self):
        """Publish map and update obstacles every 2 seconds."""
        current_time = time.time()
        if not self.goal_reached and (current_time - self.last_obstacle_time > 2.0):
            self.add_random_obstacle()
            self.last_obstacle_time = current_time

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        # Map metadata
        msg.info = MapMetaData()
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0

        # Flatten map (row-major)
        msg.data = self.map_data.flatten().tolist()

        # Publish
        self.map_pub.publish(msg)
        self.get_logger().debug("Map published.")


def main(args=None):
    rclpy.init(args=args)
    node = DynamicMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
