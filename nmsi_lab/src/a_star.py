#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import math


class AStarCalculator(Node):
    def __init__(self) -> None:
        super().__init__('a_star')

        self.fixed_frame = 'map'
        self.robot_frame = 'base_link'

        self.subscription = self.create_subscription(
            PoseStamped, '/goal_pose', self.pose_callback, 10)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.dilated_map_subscription = self.create_subscription(
            OccupancyGrid, '/map_dilated', self.dilated_map_callback, map_qos)
        self.cost_map_subscription = self.create_subscription(
            OccupancyGrid, '/map_cost', self.cost_map_callback, map_qos)

        self.publisher = self.create_publisher(Path, '/path', 10)
        self.map_dilated: OccupancyGrid | None = None
        self.map_cost: OccupancyGrid | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def pose_callback(self, pose: PoseStamped) -> None:
        try:
            trans = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.robot_frame, Time())
        except TransformException as exc:
            self.get_logger().warning(
                f'Could not transform {self.robot_frame} to '
                f'{self.fixed_frame}: {exc}')
            return

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = trans.transform.translation.x
        start.pose.position.y = trans.transform.translation.y
        start.pose.position.z = trans.transform.translation.z
        start.pose.orientation = trans.transform.rotation
        self.a_star(start, pose)

    def dilated_map_callback(self, map_msg: OccupancyGrid) -> None:
        self.map_dilated = map_msg

    def cost_map_callback(self, map_msg: OccupancyGrid) -> None:
        self.map_cost = map_msg

    def choose_and_remove(self, opened):
        opened
        return min_value
    
    def calculate_neighbors(self, x):
        neighbors = []
        return neighbors
    
    def construct_path(self, previous, current):
        path = [current]
        while current in previous:
            current = previous[current]
            path.append(current)
        return path
    
    def calculate_h(self, start):
        
        h = math.sqrt((x1 - xg)**2 + (y1 - yg)**2)
        return h
    
    def a_star(self, start: PoseStamped, end: PoseStamped) -> None:
        calculated_path = Path()
        calculated_path.header.frame_id = 'map'
        calculated_path.header.stamp = self.get_clock().now().to_msg()
        calculated_path.poses.append(start)
        calculated_path.poses.append(end)
        self.publisher.publish(calculated_path)
        been = []
        goal = end
        g = {}
        h = self.calculate_h(start)
        previous = {}
        g[start] = 0
        opened = {}
        opened[start] = g[start] + h(start)

        while len(opened) > 0:
            x = self.choose_and_remove(opened)
            if x == goal:
                return self.construct_path(previous, goal)
            been.append(x)
            neighbors_x = self.calculate_neighbors(x)
            for y in neighbors_x:
                if y in been:
                    continue
                g_temp = g[x] + self.weight(x, y)
                if y not in g or g_temp < g[y]:
                    previous[y] = x
                    g[y] = g_temp
                    opened[y] = g[y] + h(y)
        return fail

    def weight(self, x, y):
        k = 2
        w = self.d(x, y) * (1 + k * (self.cost(y)/100))
        return w
    
    def cost(self, y):
        c = 0
        return c

    def d(self, x, y):
        distance = math.sqrt((x1 - xg)**2 + (y1 - yg)**2)
        return distance


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AStarCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
