import os, time
from ultralytics import YOLO
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32
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
        self.imgrw_sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)

        # publish detections
        self.det_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)

        # publish yolo+image
        self.pub_vis = self.create_publisher(Image, '/yolo/vis_image', 10)

        self.id_map = {}
        self.center_id = None
        self.next_label = 1   # 다음에 줄 순차 번호
        
    # ── 콜백 ──────────────────────────
    def image_cb(self, msg: Image):
        t0 = time.time()

        # ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Inference (returns list of Results)
        results = self.model.track(source=frame, imgsz=320, device='cpu', classes=[0], persist=True, tracker='bytetrack.yaml', verbose=False)  # Pi 5라 CPU 추론
        det_array = Detection2DArray()
        det_array.header = msg.header

        # motor.py로 이전
        #REAL_HEIGHT_M = 1.7 # 사람 키 추정
        #FOCAL_LENGTH =548.66 #Camera_info로 계산한 값

        center = frame.shape[1] * 0.5          # 이미지 가로 중앙
        closest = None
        min_offset = float("inf")

        for det in results[0].boxes.cpu():
            if det.id is None:          # 트래킹 ID 없는 건 패스
                
                continue
            
            #Bounding Box 좌표
            x1, y1, x2, y2 = det.xyxy[0].tolist()

            cx = float((x1 + x2) * 0.5)     # numpy → py-float
            cy = float((y1 + y2) * 0.5)
            w  = float(x2 - x1)
            h  = float(y2 - y1)

            offset = abs(cx - center)   # 중앙에서 얼마나 떨어졌나
            if offset < min_offset:
                min_offset, closest = offset, int(det.id.item())
    
            #신뢰도, raw id
            conf = float(det.conf[0])
            raw_id = int(det.id.item()) if det.id is not None else None
              
            # # ── 순차 번호로 매핑: 0 → 사람1, 1 → 사람2 … ─
            # if raw_id is not None:
            #     if raw_id not in self.id_map:
            #         self.id_map[raw_id] = self.next_label
            #         self.next_label += 1
            #     seq_id = self.id_map[raw_id]          # 사람 이름 번호
            #     label_txt = f"Person{seq_id}"
            # else:
            #     label_txt = "Person"

            if raw_id == self.center_id:      # 중앙 사람 → Label 1 고정
                seq_id = 1
                self.id_map[raw_id] = 1
            else:
                if raw_id not in self.id_map:
                    # Label 2부터 순차 부여
                    self.next_label = max(self.next_label, 2)
                    self.id_map[raw_id] = self.next_label
                    self.next_label += 1
                seq_id = self.id_map[raw_id]

            label_txt = f"Person{seq_id}"

            #Robot과 사람과의 거리 계산
            #distance_m = (REAL_HEIGHT_M * FOCAL_LENGTH) / (h + 1e-6)  # h는 bbox 높이

            #self.get_logger().info(f"[{label_txt}]  conf={conf:.2f}  h={h:.1f}px  dist={distance_m:.2f} m")

            # ROS2 Detection 메시지
            det_ros = Detection2D()
            det_ros.bbox.center.position.x = float(cx)
            det_ros.bbox.center.position.y = float(cy)
            det_ros.bbox.size_x = float(w)
            det_ros.bbox.size_y = float(h)

            #Hypo
            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = label_txt     # Jazzy 이후: class_id, “사람1”
            hypo.hypothesis.score    = conf
            det_ros.results.append(hypo)
            det_array.detections.append(det_ros)

            # 시각화
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
            cv2.putText(frame, f'{label_txt} {conf:.2f}',(int(x1), int(y1)-4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 1, cv2.LINE_AA)

        # 이 raw_id를 “가장 중앙”으로 기록    
        self.center_id = closest 
        #Det_array publish    
        self.det_pub.publish(det_array)

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
