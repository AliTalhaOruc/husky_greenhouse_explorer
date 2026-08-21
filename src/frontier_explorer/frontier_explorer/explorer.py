import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
import numpy as np
import math
import tf2_ros
from rclpy.duration import Duration
from collections import deque

class AkilliSeraV3(Node):

    def __init__(self):
        super().__init__('akilli_sera_v3')

        # --- STATE ---
        self.state = "STARTUP_MOVE" 
        self.start_time = self.get_clock().now()
        self.spin_start_time = self.get_clock().now()
        self.spin_duration = 5.0 # 3sn ileri + 2sn ileri toplam süre
        
        self.max_retries = 1 # Üst üste en fazla 2 kez yer araması için manevra yapacak

        self.map_data = None
        self.map_info = None
        self.blacklist = []
        self.current_goal = None

        # --- Parametreler ---
        self.min_cluster_size = 10
      
        # --- ROS ---
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.2, self.main_loop)

        self.get_logger().info("🚀 Akıllı Sera V3 Başladı")

    # ---------------------------------------------------
    # MAP
    # ---------------------------------------------------

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width)
        )
        self.map_info = msg.info

    # ---------------------------------------------------
    # MAIN LOOP
    # ---------------------------------------------------

    def main_loop(self):

        if self.map_data is None or self.map_info is None:
            return

        if self.state == "STARTUP_MOVE":
            self.startup_logic()
        elif self.state == "IDLE":
            self.explore()

    # ---------------------------------------------------
    # EXPLORE
    # ---------------------------------------------------
    def startup_logic(self):
     twist = Twist()
     elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
     if elapsed < 1.0:
        return
     if elapsed < 3.0: # Sadece 2 saniye ileri
        twist.linear.x = 0.15 
        self.cmd_pub.publish(twist)
     else:
        self.stop_robot()
        self.state = "IDLE"
        self.get_logger().info("🚀 Başlangıç hareketi bitti, keşif başlıyor...")
    
    def explore(self):
        try:
            t = self.tf_buffer.lookup_transform(
             'map', 'base_link', rclpy.time.Time(),timeout=Duration(seconds=1.0))
            rx = t.transform.translation.x
            ry = t.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"TF lookup başarısız: {e}")
            return

        clusters = self.find_frontier_clusters()

        if not clusters:
            # Harita bittiyse veya yer yoksa zorlamıyoruz, kapatıyoruz.
             self.get_logger().info("🏁 Gidilecek yer kalmadı. Keşif tamamlandı.")
             self.stop_robot()
             self.state = "FINISHED"
             return

    # Eğer cluster bulursa deneme sayısını sıfırla
        
        target = self.select_best_cluster(clusters, rx, ry)
        if target:
         self.send_goal(target[0], target[1])
    # ---------------------------------------------------
    # FRONTIER DETECTION
    # ---------------------------------------------------

    def is_frontier(self, y, x):
        if self.map_data[y, x] != 0:
            return False
        neighbors = self.map_data[y-1:y+2, x-1:x+2]
        return np.any(neighbors == -1)

    def find_frontier_clusters(self):

        h, w = self.map_data.shape
        visited = np.zeros_like(self.map_data, dtype=bool)
        clusters = []

        for y in range(1, h-1):
            for x in range(1, w-1):

                if visited[y, x]:
                    continue
                if not self.is_frontier(y, x):
                    continue

                cluster = []
                queue = deque()
                queue.append((y, x))
                visited[y, x] = True

                while queue:
                    cy, cx = queue.popleft()
                    cluster.append((cy, cx))

                    for ny in range(cy-1, cy+2):
                        for nx in range(cx-1, cx+2):
                            if 0 < ny < h-1 and 0 < nx < w-1:
                                if not visited[ny, nx] and self.is_frontier(ny, nx):
                                    visited[ny, nx] = True
                                    queue.append((ny, nx))

                if len(cluster) >= self.min_cluster_size:
                    clusters.append(cluster)

        return clusters

    # ---------------------------------------------------
    # CLUSTER SELECTION
    # ---------------------------------------------------

    def select_best_cluster(self, clusters, rx, ry):
        res = self.map_info.resolution
        origin = self.map_info.origin.position

        best_score = -1
        best_target = None

        for cluster in clusters:
            ys = [p[0] for p in cluster]
            xs = [p[1] for p in cluster]

            cy = int(np.mean(ys))
            cx = int(np.mean(xs))

            # --- GÜVENLİK KONTROLÜ ---
            check_dist = int(0.5 / res)
            y_min, y_max = max(0, cy-check_dist), min(self.map_data.shape[0], cy+check_dist)
            x_min, x_max = max(0, cx-check_dist), min(self.map_data.shape[1], cx+check_dist)
            region = self.map_data[y_min:y_max, x_min:x_max]
        
            if np.any(region == 100):
                safety_multiplier = 0.5 
            else:
                safety_multiplier = 1.0

            wx_raw = cx * res + origin.x
            wy_raw = cy * res + origin.y
           
            # Blacklist kontrol
            if any(math.hypot(wx_raw-bx, wy_raw-by) < 0.5 for bx,by in self.blacklist):
                continue

            dist = math.hypot(wx_raw - rx, wy_raw - ry)
            gain = len(cluster)

            score = (gain * 2.0 + dist) * safety_multiplier

            if score > best_score:
                best_score = score
                best_target = (wx_raw, wy_raw)

        # --- DÖNGÜ BİTTİKTEN SONRA (FOR DIŞINDA) ---
        if best_target:
            wx, wy = best_target
            dist_to_goal = math.hypot(wx - rx, wy - ry)

            # Sadece hedef uzaktaysa (1m+) içeri çekme yap
            if dist_to_goal > 1.0:
                offset_factor = 0.20 
                wx = wx + (rx - wx) * offset_factor
                wy = wy + (ry - wy) * offset_factor
            
            return (wx, wy)
            
        return None

    # ---------------------------------------------------
    # NAVIGATION
    # ---------------------------------------------------

    def send_goal(self, x, y):

        if self.state != "IDLE":
            return
        self.get_logger().info(f"--- YENİ HEDEF GÖNDERİLİYOR --- X: {x:.2f}, Y: {y:.2f}")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.current_goal = (x, y)
        self.state = "NAVIGATING"

        self.nav_client.wait_for_server()

        self.nav_client.send_goal_async(goal)\
            .add_done_callback(self.goal_response)

    def goal_response(self, future):

        handle = future.result()

        if not handle.accepted:
            self.state = "IDLE"
            return

        handle.get_result_async()\
            .add_done_callback(self.goal_result)
    def goal_result(self, future):
       if self.current_goal:
          self.blacklist.append(self.current_goal)

         # Her hedeften sonra manevra YAPMA, sadece bekle
       self.state = "IDLE"
       self.get_logger().info("Hedef tamamlandı, bir sonrakine karar veriliyor...")

    # ---------------------------------------------------
    # SPIN
    # ---------------------------------------------------

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)


# ---------------------------------------------------
# MAIN
# ---------------------------------------------------

def main():
    rclpy.init()
    node = AkilliSeraV3()
    rclpy.spin(node)