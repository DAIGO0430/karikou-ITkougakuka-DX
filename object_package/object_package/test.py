import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('object_detection_subscriber')
        
        # セグメンテーションに使うブリッジのインスタンス
        self.bridge = CvBridge()

        # 画像トピックのサブスクライブ
        self.subscription = self.create_subscription(
            Image,
            '/object_detection/output_image',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        # ROSメッセージをOpenCVの画像に変換
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # セグメンテーション（例：単純な色による背景と前景の分離）
        # HSV空間に変換
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 色範囲を設定（背景と前景を区別する簡単な例）
        lower_bound = np.array([35, 50, 50])  # 背景色の下限
        upper_bound = np.array([85, 255, 255])  # 背景色の上限

        # マスクを作成
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        
        # マスクを使って画像をセグメンテーション
        segmented_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # セグメンテーションされた画像を表示
        cv2.imshow("Segmented Image", segmented_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    # ノードを作成して実行
    object_detection_subscriber = ObjectDetectionSubscriber()

    # ノードをスピン
    rclpy.spin(object_detection_subscriber)

    # クリーンアップ
    object_detection_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

