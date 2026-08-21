import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # 1. Navigasyonun hazır olmasını bekle
    navigator.waitUntilNav2Active()

    # 2. Hedef Noktayı Tanımla
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # BURAYI DEĞİŞTİR: Rviz'de okuduğun bir koordinatı yaz
    goal_pose.pose.position.x = 2.0 
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0 # Baktığı yön

    # 3. Robotu Gönder
    print("Hedefe gidiliyor...")
    navigator.goToPose(goal_pose)

    # 4. Hareket bitene kadar bekle ve durumu kontrol et
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'Kalan mesafe: {feedback.distance_remaining:.2f} metre.')

    # 5. Sonucu Yazdır
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Hedefe başarıyla ulaşıldı!')
    else:
        print('Hedefe ulaşılamadı.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()