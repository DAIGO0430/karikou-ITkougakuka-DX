class ObjectDetectionNode:
    def __init__(self):
        image_path = 'home/suzuki112/ros2_ws/src/object_package/image/test_image.png'  # ここで画像パスが指定されている
        self.perform_detection(image_path)

