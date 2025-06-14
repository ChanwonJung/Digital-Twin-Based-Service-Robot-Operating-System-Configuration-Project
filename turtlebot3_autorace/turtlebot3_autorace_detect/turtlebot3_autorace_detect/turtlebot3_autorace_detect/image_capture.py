import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime
import threading

class CaptureImage(Node):

    def __init__(self):
        super().__init__('capture_image')

        self.cv_bridge = CvBridge()
        self.current_image = None
        self.lock = threading.Lock()

        # 저장 경로 설정
        self.save_dir = '/home/hongha/YOLO_Dataset/Traffic_Signs_Dataset'
        os.makedirs(self.save_dir, exist_ok=True)

        # /camera/image 토픽 구독
        self.sub_image = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        self.get_logger().info('CaptureImage Node Initialized')

    def image_callback(self, msg):
        try:
            img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.current_image = img
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def get_latest_image(self):
        with self.lock:
            return self.current_image.copy() if self.current_image is not None else None

    def save_image(self, img):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        save_path = os.path.join(self.save_dir, f'image_{timestamp}.jpg')
        cv2.imwrite(save_path, img)
        self.get_logger().info(f'Image saved: {save_path}')


def main(args=None):
    rclpy.init(args=args)
    node = CaptureImage()

    # 멀티스레드 실행자 사용
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():
            image = node.get_latest_image()
            if image is not None:
                # === 영상 크기 2배 확대 ===
                resized_image = cv2.resize(image, (image.shape[1]*2, image.shape[0]*2))
                cv2.imshow('Camera View - Press "s" to save, "q" to quit', resized_image)

                # cv2.imshow('Camera View - Press "s" to save, "q" to quit', image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('s'):
                    node.save_image(resized_image)
                elif key == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
