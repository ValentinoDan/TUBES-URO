import rclpy # library untuk mengembangkan Node ROS 2
from rclpy.node import Node # Kelas dasar untuk membuat node ROS
from sensor_msgs.msg import Image # Tipe pesan untuk data gambar
from geometry_msgs.msg import Twist # Tipe pesan untuk kecepatan linier dan sudut
from cv_bridge import CvBridge # Mengonversi data gambar antara ROS dan OpenCV
import cv2
import numpy as np

class LineFollowingNode(Node):
    def __init__(self):
        super().__init__('line_following_node')
        
        # Subscriber untuk gambar dari kamera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', # mendapatkan data gambar
            self.listener_callback,
            10)
        self.br = CvBridge()

        # Publisher untuk kecepatan
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10) # Menerbitkan perintah kecepatan ke topik /cmd_vel

    def listener_callback(self, data):
        # Konversi dari ROS Image ke OpenCV Image
        current_frame = self.br.imgmsg_to_cv2(data)

        # Konversi ke ruang warna HSV untuk mendeteksi warna merah
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Definisikan batas bawah dan atas untuk warna merah dalam ruang warna HSV
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])

        # Masker untuk mendeteksi warna merah
        red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)

        # Anda juga bisa menambahkan deteksi warna merah lainnya di bagian atas spektrum
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        red_mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)

        # Gabungkan kedua masker
        red_mask_combined = cv2.bitwise_or(red_mask, red_mask2)

        # Temukan kontur pada masker merah
        contours, _ = cv2.findContours(red_mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            # Menemukan kontur terbesar yang kemungkinan merupakan garis merah
            largest_contour = max(contours, key=cv2.contourArea)

            # Dapatkan bounding box persegi atau persegi panjang terbesar
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Gambarkan persegi panjang di sekitar garis
            cv2.rectangle(current_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Kotak biru
            
            # Menemukan titik tengah dari persegi panjang
            cx = int(x + w // 2)
            cy = int(y + h // 2)

            # Error relatif terhadap pusat gambar
            height, width = current_frame.shape[:2]
            error = cx - width // 2
            
            # Logika kontrol
            twist.linear.x = 0.2  # Kecepatan maju
            twist.angular.z = -error * 0.005  # Koreksi arah berdasarkan error
        else:
            # Jika tidak ada kontur merah, berhenti atau mencari garis
            twist.linear.x = 0.0
            twist.angular.z = 0.3  # Rotasi untuk mencari garis

        # Publikasi kecepatan
        self.velocity_publisher.publish(twist)

        # Tampilkan frame untuk debugging
        cv2.imshow("Red Mask", red_mask_combined)  # Menampilkan gambar dengan masker merah
        cv2.imshow("Detected Lines", current_frame)  # Menampilkan gambar dengan kotak
        cv2.waitKey(1)
    
def main(args=None):
    rclpy.init(args=args) # Inisiasi ROS2

    # Inisialisasi node line_following
    node = LineFollowingNode()

    try:
        # Menjalankan node
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node dihentikan oleh pengguna.")
    finally: # Kode yang akan dijalankan selalu, baik ada error maupun tidak
        # Menutup semua jendela OpenCV
        cv2.destroyAllWindows()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
