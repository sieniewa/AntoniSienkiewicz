#!/usr/bin/env python3
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


# Ruchy: [dx, dy, koszt_ruchu]
# Ruchy proste mają koszt 1.0, ruchy po przekątnej sqrt(2) ≈ 1.414
NEIGHBOR_OFFSETS = [
    ( 0,  1, 1.0),   # Góra
    ( 0, -1, 1.0),   # Dół
    ( 1,  0, 1.0),   # Prawo
    (-1,  0, 1.0),   # Lewo
    ( 1,  1, 1.414), # Góra-Prawo
    (-1,  1, 1.414), # Góra-Lewo
    ( 1, -1, 1.414), # Dół-Prawo
    (-1, -1, 1.414)  # Dół-Lewo
]


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
        current = min(opened, key=opened.get)
        del opened[current]
        return current

    def calculate_neighbors(
        self, current, grid: OccupancyGrid):
        width = grid.info.width
        height = grid.info.height
        x, y = current
        neighbors = []

        def is_free(cell_x: int, cell_y: int):
            if not (0 <= cell_x < width and 0 <= cell_y < height):
                return False
            value = grid.data[cell_y * width + cell_x]
            return 0 <= value < 65

        for dx, dy, move_cost in NEIGHBOR_OFFSETS:
            neighbor_x = x + dx
            neighbor_y = y + dy
            if not is_free(neighbor_x, neighbor_y):
                continue
            if dx != 0 and dy != 0:
                if not is_free(x + dx, y) or not is_free(x, y + dy):
                    continue
            neighbors.append(((neighbor_x, neighbor_y), move_cost))

        return neighbors
    
    def a_star(self, start: PoseStamped, end: PoseStamped) -> None:
        if self.map_dilated is None:
            self.get_logger().warning('Brak: /map_dilated is not available')
            return

        grid = self.map_dilated
        width = grid.info.width
        height = grid.info.height
        if width == 0 or height == 0 or grid.info.resolution <= 0:
            self.get_logger().warning('Brak: /map_dilated has invalid dimensions')
            return

        start_cell = self.meters_to_cell(start, grid)
        goal_cell = self.meters_to_cell(end, grid)
        if not self.cell_in_bounds(start_cell, width, height):
            self.get_logger().warning('Brak: start is outside the map')
            return
        if (not self.cell_in_bounds(goal_cell, width, height)
                or not self.cell_is_free(goal_cell, grid)):
            self.get_logger().warning('Brak: goal is blocked or outside the map')
            return
        if not self.cell_in_bounds(goal_cell, width, height):
                    self.get_logger().warning('Brak: goal is outside the map')
                    return

        costs_available = (
            self.map_cost is not None
            and self.map_cost.info.width == width
            and self.map_cost.info.height == height
        )
        cost_scale = 5.0
        g_score = {start_cell: 0.0}
        previous = {}
        opened = {start_cell: self.distance(start_cell, goal_cell, grid)}
        closed = set()

        while opened:
            current = self.choose_and_remove(opened)
            if current == goal_cell:
                cells = self.construct_path(previous, current)
                calculated_path = Path()
                calculated_path.header.frame_id = 'map'
                calculated_path.header.stamp = self.get_clock().now().to_msg()
                calculated_path.poses.append(start)
                for cell in cells[1:]:
                    calculated_path.poses.append(self.cell_to_pose(cell, grid))
                calculated_path.poses.append(end)
                self.publisher.publish(calculated_path)
                return

            closed.add(current)
            for neighbor, move_cost in self.calculate_neighbors(current, grid):
                if neighbor in closed:
                    continue
                edge_cost = move_cost * grid.info.resolution
                if costs_available:
                    idx = neighbor[1] * width + neighbor[0]
                    map_cost = self.map_cost.data[idx]
                    if map_cost >= 0:
                        edge_cost *= 1.0 + cost_scale * (map_cost / 100.0)
                tent_g = g_score[current] + edge_cost
                if tent_g < g_score.get(neighbor, math.inf):
                    previous[neighbor] = current
                    g_score[neighbor] = tent_g
                    heuristic = self.distance(neighbor, goal_cell, grid)
                    opened[neighbor] = tent_g + heuristic

        self.get_logger().warning('Brak: no path to goal')

    def meters_to_cell(self, pose: PoseStamped, grid: OccupancyGrid):
        resolution = grid.info.resolution
        origin = grid.info.origin.position
        x = math.floor((pose.pose.position.x - origin.x) / resolution)
        y = math.floor((pose.pose.position.y - origin.y) / resolution)
        return x, y

    def cell_in_bounds(
        self, cell, width: int, height: int
    ) -> bool:
        x, y = cell
        return 0 <= x < width and 0 <= y < height

    def cell_is_free(self, cell, grid: OccupancyGrid):
        x, y = cell
        value = grid.data[y * grid.info.width + x]
        return 0 <= value < 65

    def cell_to_pose(self, cell, grid: OccupancyGrid) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = (
            grid.info.origin.position.x + (cell[0] + 0.5) * grid.info.resolution
        )
        pose.pose.position.y = (
            grid.info.origin.position.y + (cell[1] + 0.5) * grid.info.resolution
        )
        pose.pose.orientation.w = 1.0
        return pose

    def distance(
        self, first, second, grid: OccupancyGrid
    ) -> float:
        dx = (first[0] - second[0]) * grid.info.resolution
        dy = (first[1] - second[1]) * grid.info.resolution
        return math.hypot(dx, dy)

    def construct_path(self, previous, current):
        path = [current]
        while current in previous:
            current = previous[current]
            path.append(current)
        path.reverse()
        return path


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
