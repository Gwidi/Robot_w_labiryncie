import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
import numpy as np

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer_node')

        # 1. Subskrypcje
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.dist_sub = self.create_subscription(Float32, '/referee/distance_left', self.dist_callback, 10)
        
        # 2. Klient Akcji Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 3. Zmienne stanu
        self.last_goal = (0.0, 0.0)
        self.current_map = None
        self.distance_to_goal = float('inf')
        self.is_moving = False
        self.exploration_active = True

        self.get_logger().info('Węzeł ExplorerNode został uruchomiony.')

    def dist_callback(self, msg):
        self.distance_to_goal = msg.data
        if self.distance_to_goal < 0.3: # Dotarto do celu (czerwony punkt)
            self.get_logger().info('CEL OSIĄGNIĘTY!')
            self.exploration_active = False

    def map_callback(self, msg):
        self.current_map = msg
        if self.exploration_active and not self.is_moving:
            self.find_and_send_goal()

    def find_and_send_goal(self):
        if self.current_map is None: return

        # Prosta analiza mapy w poszukiwaniu "granic" (frontiers)
        # Zamiana danych mapy na macierz numpy
        width = self.current_map.info.width
        height = self.current_map.info.height
        grid = np.array(self.current_map.data).reshape((height, width))

        # Szukamy punktów wolnych (0), które sąsiadują z nieznanymi (-1)
        # Uproszczony algorytm przeszukiwania siatki
        frontiers = []
        for y in range(1, height - 1, 5): # Skok co 5 komórek dla wydajności
            for x in range(1, width - 1, 5):
                if grid[y, x] == 0: # Wolna przestrzeń
                    # Sprawdź czy sąsiaduje z nieznanym (-1)
                    if -1 in grid[y-1:y+2, x-1:x+2]:
                        # Konwersja z indeksu mapy na współrzędne świata (metry)
                        world_x = x * self.current_map.info.resolution + self.current_map.info.origin.position.x
                        world_y = y * self.current_map.info.resolution + self.current_map.info.origin.position.y
                        frontiers.append((world_x, world_y))

        if not frontiers:
            self.get_logger().warn('Nie znaleziono nowych granic na mapie!')
            return

        # Wybór celu: w tym miejscu można dodać logikę kierującą robota 
        # tam, gdzie dystans do celu (distance_left) potencjalnie maleje.
        # Na potrzeby demo wybieramy najbliższą granicę.
        for target in frontiers:
            # Sprawdź czy cel nie jest tym samym co poprzednio
            dist_to_last = np.hypot(target[0] - self.last_goal[0], target[1] - self.last_goal[1])
            if dist_to_last > 0.5: # Wybierz cel oddalony o min. 0.5m od poprzedniego
                self.last_goal = target
                self.send_nav2_goal(target[0], target[1])
                break

    def send_nav2_goal(self, x, y):
        self.is_moving = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Wysyłanie robota do celu: x={x:.2f}, y={y:.2f}')
        
        self.nav_client.wait_for_server()
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Cel odrzucony przez Nav2.')
            self.is_moving = False
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.is_moving = False
        self.get_logger().info('Osiągnięto punkt graniczny, szukanie kolejnego...')

def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()