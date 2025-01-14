import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()
        self.yolo_model = YOLO("yolov8n.pt")  # Load YOLOv8 model

        # 画像のパブリッシャを作成
        self.publisher = self.create_publisher(Image, '/object_detection/output_image', 10)

        # 信頼度閾値を設定
        self.yolo_model.overrides['conf'] = 0.1  # 閾値を設定
        self.yolo_model.verbose = True

    def listener_callback(self, msg):
        # ROS ImageメッセージをOpenCV画像に変換
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting ROS Image to OpenCV: {str(e)}")
            return  # エラーが発生した場合は処理を中断する

        # 画像が空でないことを確認
        if cv_image is None or cv_image.size == 0:
            self.get_logger().error("Received an empty image from the camera.")
            return  # エラーが発生した場合は処理を中断する

        # YOLOv8を使用して物体検出を実行
        results = self.yolo_model(cv_image)

        # 検出結果を処理
        self.process_detections(cv_image, results)

        # 処理済み画像をROS Imageメッセージとしてパブリッシュ
        self.publish_processed_image(cv_image)

        # 処理済み画像を表示
        cv2.imshow("Object Detection", cv2.resize(cv_image, (1920, 1080)))
      
        cv2.waitKey(1)

    def publish_processed_image(self, cv_image):
        try:
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.publisher.publish(processed_image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting OpenCV image to ROS Image: {str(e)}")

    def process_detections(self, cv_image, results):
        # 検出結果を描画し、ログに記録する
        if results:
            for result in results:
                if result.boxes:
                    self.get_logger().info(f"Detected classes: {result.names}")
                    self.get_logger().info(f"Confidence scores: {result.boxes.conf}")
                    self.get_logger().info(f"Bounding boxes: {result.boxes.xyxy}")

                    # バウンディングボックスを描画
                    for box in result.boxes:
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        label = self.yolo_model.names[int(box.cls[0])]
                        confidence = box.conf[0].item()
                        text = f"{label} {confidence:.2f}"
                        cv2.putText(cv_image, text, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                else:
                    self.get_logger().info("No detections")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

