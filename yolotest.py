import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/yolo', 10)
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n-seg.pt")
        self.resize_factor = 0.5

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        small_frame = cv2.resize(cv_image, (int(cv_image.shape[1] * self.resize_factor), int(cv_image.shape[0] * self.resize_factor)))
        results = self.model(small_frame)
        for result in results:                  
            boxes = result.boxes.cpu().numpy()                     
            for box in boxes:
                print(result.names[int(box.cls[0])])
                #print(box)
        annotated_frame = results[0].plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.publisher.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    yolo_node = YOLONode()
    rclpy.spin(yolo_node)
    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
