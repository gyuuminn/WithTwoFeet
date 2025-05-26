import os, time
from ultralytics import YOLO
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

class IMGDetector(Node):
    def __init__(self):
        super().__init__('IMGDetector')

        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, "yolov8n.pt")
        # #YOLO 모델 불러오기 
        self.model = YOLO(model_path)
        self.model.fuse()               # small speed-up
        self.get_logger().info(f'Loaded YOLOv8 model: {model_path}')
        # ───────── ROS 객체들 ─────────
        self.bridge = CvBridge()
        # subscribe raw image
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        # publish detections
        self.pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        # 이미지 퍼블리시
        self.pub_vis = self.create_publisher(Image, '/yolo/vis_image', 10)
        
    # ── 콜백 ──────────────────────────
    def image_cb(self, msg: Image):
        t0 = time.time()

        # ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Inference (returns list of Results)
        results = self.model.track(source=frame, imgsz=320, device='cpu', classes=[0], persist=True, tracker='bytetrack.yaml', verbose=False)  # Pi 5라 CPU 추론
        det_array = Detection2DArray()
        det_array.header = msg.header

        # 첫 batch만 사용
        for i, det in enumerate(results[0].boxes.cpu()):
            x1, y1, x2, y2 = det.xyxy[0].tolist()
            conf = float(det.conf[0])
            track_id = int(det.id.item()) if det.id is not None else -1   
            label  = f'person{track_id}' if track_id >= 0 else 'person'

            # ROS2 Detection 메시지
            det_ros = Detection2D()
            cx = float((x1 + x2) * 0.5)     # numpy → py-float
            cy = float((y1 + y2) * 0.5)
            w  = float(x2 - x1)
            h  = float(y2 - y1)

            #Robot과 사람과의 거리
            FOCAL_LENGTH = 320  # 추정 값
            REAL_HEIGHT_M = 1.7 # 사람 키
            distance_m = (REAL_HEIGHT_M * FOCAL_LENGTH) / (h + 1e-6)  # h는 bbox 높이
            self.get_logger().info(f"Estimated distance: {distance_m:.2f} m")

            det_ros.bbox.center.position.x = cx
            det_ros.bbox.center.position.y = cy
            det_ros.bbox.size_x = w
            det_ros.bbox.size_y = h

            #Hypo
            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = label     # Jazzy 이후: class_id
            hypo.hypothesis.score    = conf
            det_ros.results.append(hypo)
        
            det_array.detections.append(det_ros)

            # 시각화
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
            cv2.putText(frame, f'{label} {conf:.2f}',(int(x1), int(y1)-4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 1, cv2.LINE_AA)
        self.pub.publish(det_array)

        # 디버그: 표시용 이미지
        vis_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        vis_msg.header = msg.header
        self.pub_vis.publish(vis_msg)
        dt = (time.time() - t0)*1000
        self.get_logger().debug(f'inference {dt:.1f} ms, {len(det_array.detections)} objects')


def main():
    rclpy.init()
    node = IMGDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
