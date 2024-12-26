import rclpy # library untuk mengembangkan Node ROS 2
from rclpy.node import Node # Kelas dasar untuk membuat node ROS
from sensor_msgs.msg import Image # Tipe pesan untuk data gambar
from geometry_msgs.msg import Twist # Tipe pesan untuk kecepatan linier dan sudut
from cv_bridge import CvBridge # Mengonversi data gambar antara ROS dan OpenCV
import cv2

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

        # Konversi ke grayscale dan threshold
        gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY) # Mengubah gambar menjadi grayscale
        _, binary_frame = cv2.threshold(gray_frame, 100, 255, cv2.THRESH_BINARY)

        # Deteksi posisi garis menggunakan momen
        height, width = binary_frame.shape
        moments = cv2.moments(binary_frame)
        twist = Twist()

        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])  # Titik tengah garis
            error = cx - width // 2  # Error relatif terhadap pusat frame
            
            # Logika kontrol
            twist.linear.x = 0.2  # Kecepatan maju
            twist.angular.z = -error * 0.005  # Koreksi arah berdasarkan error

            # Tambahkan kotak di sekitar garis
            contours, _ = cv2.findContours(binary_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour) 
                cv2.rectangle(current_frame, (x, y), (x + w, y + h), (255, 0, 0), 2) # membuat kotak berwarna biru
        else:
            # Jika tidak ada garis, berhenti atau cari garis
            twist.linear.x = 0.0
            twist.angular.z = 0.3  # Rotasi untuk mencari garis

        # Publikasi kecepatan
        self.velocity_publisher.publish(twist)

        # Tampilkan frame untuk debugging
        cv2.imshow("Binary Frame", binary_frame) # Menampilkan gambar biner
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
