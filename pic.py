import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        # トピック '/object_detection/output_image' へのパブリッシャー作成
        self.publisher_ = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.bridge = CvBridge()

        # Webカメラのキャプチャ
        self.cap = cv2.VideoCapture(0)

        # カメラが開けなかった場合のエラーチェック
        if not self.cap.isOpened():
            self.get_logger().error("エラー: カメラが開けません")
            exit()

        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1秒間隔で画像を送信
        self.get_logger().info("カメラ画像の送信を開始しました")

    def timer_callback(self):
        # 1枚の画像をキャプチャして送信
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("エラー: フレームを取得できません")
            return
        
        # OpenCVの画像をROSのImageメッセージに変換
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            # 画像をトピックに送信
            self.publisher_.publish(ros_image)
            self.get_logger().info("画像を送信しました")
        except Exception as e:
            self.get_logger().error(f"画像の送信に失敗しました: {e}")

    def __del__(self):
        # カメラの解放
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

