import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

def create_pose(navigator, x, y, orientation_z=0.0, orientation_w=1.0):
    """Hedef noktası oluşturmak için yardımcı fonksiyon"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = orientation_z
    pose.pose.orientation.w = orientation_w
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Nav2'nin tam hazır olmasını bekle
    navigator.waitUntilNav2Active()

    # --- GÖREV LİSTESİ (Koordinatlarını RViz'den aldıklarınla değiştir) ---
    # Örnek: [Koridor 1 Giriş, Koridor 1 Çıkış, Koridor 2 Giriş, Koridor 2 Çıkış]
    mission_waypoints = [
        create_pose(navigator, -1.0, -1.5),   # Nokta 1
        create_pose(navigator, 1.5, -1.5),   # Nokta 2 (Koridor sonu)
        create_pose(navigator, 1.6, -0.5),   # Nokta 3 (Yan koridora geçiş)
        create_pose(navigator, -1.2, -0.5),   # Nokta 4
        create_pose(navigator, -1.2, 0.6),   # Nokta 5
        create_pose(navigator, 1.55, 0.6),   # Nokta 6
        create_pose(navigator, 1.35, 1.7),   # Nokta 7
        create_pose(navigator, -1.21, 1.7),   # Nokta 8
    ]

    print(f"Görev başlıyor: Toplam {len(mission_waypoints)} hedef nokta var.")

    for i, waypoint in enumerate(mission_waypoints):
        print(f"\n{i+1}. hedefe gidiliyor...")
        navigator.goToPose(waypoint)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            # Her 1 saniyede bir mesafe bilgisi basılabilir (isteğe bağlı)
            
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"--- {i+1}. Noktaya ulaşıldı! İlaçlama yapılıyor... ---")
            time.sleep(3) # 3 saniye ilaçlama bekleme simülasyonu
        else:
            print(f"!!! {i+1}. Noktaya ulaşılamadı. Görev iptal ediliyor. !!!")
            break

    print("\n--- TÜM GÖREV TAMAMLANDI. BAŞLANGIÇA DÖNÜLÜYOR. ---")
    # Başlangıca (0,0) dönmek istersen:
    navigator.goToPose(create_pose(navigator, -2.25, 0.01))
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()