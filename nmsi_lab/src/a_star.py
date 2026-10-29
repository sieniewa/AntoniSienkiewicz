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

    def choose_and_remove(self, opened: dict, start: PoseStamped, end: PoseStamped):
        min_val = opened[start]
        f_val = opened[start]
        for i in range(len(opened)):
            h = self.distance(opened[i], end)
            g = self.distance(start, opened[i])
            f_val = g + h
            if f_val < min_val:
                min_val = f_val
                min_f = opened[i]
        opened.pop(min_f)
        return min_f
    
    def get_8_neighbors(current, mapa):
    
        neighbors = []
        map_height = len(mapa)
        map_width = len(mapa[0])
        
        current_x = current.x
        current_y = current.y

        for dx, dy, move_cost in NEIGHBOR_OFFSETS:
            neighbor_x = current_x + dx
            neighbor_y = current_y + dy

            # 1. Sprawdzenie granic mapy
            if 0 <= neighbor_x < map_width and 0 <= neighbor_y < map_height:
                
                # 2. Sprawdzenie, czy komórka nie jest przeszkodą
                if mapa[neighbor_y][neighbor_x] == 0: # 0 oznacza wolną przestrzeń
                    
                    # 3. Blokada "ścinania zakrętów" (Corner Cutting)
                    # Jeśli idziemy po skosie, sprawdź czy przejście nie blokuje się o narożnik robotem
                    if dx != 0 and dy != 0:
                        if mapa[current_y][current_x + dx] != 0 or mapa[current_y + dy][current_x] != 0:
                            continue # Pomiń ten krok, jeśli narożniki są zablokowane
                    
                    neighbors.append({
                        'x': neighbor_x,
                        'y': neighbor_y,
                        'cost': move_cost
                    })
                    
        return neighbors
    
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
        h = {}
        h[start] = self.distance(start, end)
        previous = {}
        g[start] = 0
        opened = {}
        opened[start] = g[start] + h[start]

        while len(opened) > 0:
            x = self.choose_and_remove(opened)
            if x == goal:
                return self.construct_path(previous, x)
            been.append(x)
            neighbors_x = self.calculate_neighbors(x, self.map_dilated.data)
            for y in neighbors_x:
                if y in been:
                    continue
                g_temp = g[x] + self.weight(x, y)
                if y not in g or g_temp < g[y]:
                    previous[y] = x
                    g[y] = g_temp
                    opened[y] = g[y] + h(y)
        return fail

    def weight(self, x: PoseStamped, y: PoseStamped):
        k = 2
        w = self.distance(x, y) * (1 + k * (self.cost(y)/100))
        return w
    
    def cost(self, y: PoseStamped):
        idx = y.pose.position.y * self.map_dilated.info.width + y.pose.position.x
        c = self.map_dilated.data[idx]
        return c

    def distance(self, x: PoseStamped, y: PoseStamped):
        x1 = x.pose.position.x
        y1 = y.pose.position.x
        x2 = x.pose.position.y
        y2 = y.pose.position.y
        distance = math.sqrt((x1 - y1)**2 + (x2 - y2)**2)
        return distance

    def construct_path(previous: dict, current: PoseStamped):
        path = [current]
        while current in previous:
            current = previous[current]
            path.dodaj_na_początek(current)
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
