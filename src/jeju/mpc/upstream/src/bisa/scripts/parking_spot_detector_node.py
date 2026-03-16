#!/usr/bin/env python3
"""
주차구역 카메라 감지 노드
==========================
카메라 이미지에서 흰색 사각형 주차선을 감지하여 주차구역 자세를 추정합니다.

알고리즘:
  1. 흰색 영역 HSV 임계값 처리
  2. 형태학적 연산으로 노이즈 제거
  3. 컨투어 검출 및 사각형 필터링
  4. 원근 변환으로 실제 좌표 추정
  5. /parking_spot 토픽 발행

구독:
  /camera/image_raw   (sensor_msgs/Image)
  /camera/camera_info (sensor_msgs/CameraInfo)

발행:
  /parking_spot       (bisa/ParkingSpot)
  /parking_spot_image (sensor_msgs/Image)  디버그 이미지
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import cv2
import numpy as np
import math
from typing import Optional, List, Tuple

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from cv_bridge import CvBridge

# bisa 커스텀 메시지
try:
    from bisa.msg import ParkingSpot
except ImportError:
    # 빌드 전 임포트 실패 대비
    ParkingSpot = None


class ParkingSpotDetector(Node):
    """
    흰색 사각형 주차선 감지 노드
    """

    def __init__(self):
        super().__init__('parking_spot_detector')

        # ---- 파라미터 선언 ----
        self.declare_parameter('image_topic',  '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('publish_debug_image', True)

        # 흰색 감지 임계값 (HSV)
        self.declare_parameter('white_h_min', 0)
        self.declare_parameter('white_h_max', 180)
        self.declare_parameter('white_s_min', 0)
        self.declare_parameter('white_s_max', 40)    # 낮은 채도 = 흰색
        self.declare_parameter('white_v_min', 180)   # 높은 명도 = 밝음
        self.declare_parameter('white_v_max', 255)

        # 주차구역 크기 파라미터 [m]
        self.declare_parameter('spot_width',  2.5)
        self.declare_parameter('spot_length', 5.0)
        # 소형 차량용
        self.declare_parameter('min_area_px',  1000)   # 최소 픽셀 면적
        self.declare_parameter('max_area_px',  200000) # 최대 픽셀 면적
        self.declare_parameter('aspect_ratio_min', 0.3)  # 최소 종횡비
        self.declare_parameter('aspect_ratio_max', 5.0)  # 최대 종횡비

        # 카메라 설치 파라미터
        self.declare_parameter('camera_height_m',   0.3)    # 카메라 높이 [m]
        self.declare_parameter('camera_pitch_deg', -30.0)   # 카메라 피치 (아래방향 양수)
        self.declare_parameter('camera_fov_deg',   60.0)    # 수평 시야각

        # 감지 신뢰도 임계값
        self.declare_parameter('confidence_threshold', 0.4)

        # ---- 파라미터 읽기 ----
        self._read_params()

        # ---- ROS2 구성 ----
        img_topic   = self.get_parameter('image_topic').value
        info_topic  = self.get_parameter('camera_info_topic').value
        self.pub_debug = self.get_parameter('publish_debug_image').value

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1)

        self.img_sub = self.create_subscription(
            Image, img_topic, self._image_callback, sensor_qos)
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self._camera_info_callback, 10)

        self.spot_pub = self.create_publisher(ParkingSpot, '/parking_spot', 10) \
            if ParkingSpot else None
        if self.pub_debug:
            self.debug_pub = self.create_publisher(Image, '/parking_spot_image', 10)

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None

        # 감지 히스토리 (False Positive 감소)
        self._detect_history: List[bool] = []
        self._history_len = 5

        self.get_logger().info('주차구역 감지 노드 시작')
        self.get_logger().info(f'  이미지 토픽: {img_topic}')

    def _read_params(self):
        """파라미터 로드"""
        self.white_hsv_lower = np.array([
            self.get_parameter('white_h_min').value,
            self.get_parameter('white_s_min').value,
            self.get_parameter('white_v_min').value,
        ], dtype=np.uint8)
        self.white_hsv_upper = np.array([
            self.get_parameter('white_h_max').value,
            self.get_parameter('white_s_max').value,
            self.get_parameter('white_v_max').value,
        ], dtype=np.uint8)
        self.spot_width_m  = self.get_parameter('spot_width').value
        self.spot_length_m = self.get_parameter('spot_length').value
        self.min_area_px   = self.get_parameter('min_area_px').value
        self.max_area_px   = self.get_parameter('max_area_px').value
        self.ar_min        = self.get_parameter('aspect_ratio_min').value
        self.ar_max        = self.get_parameter('aspect_ratio_max').value
        self.cam_h         = self.get_parameter('camera_height_m').value
        self.cam_pitch     = math.radians(self.get_parameter('camera_pitch_deg').value)
        self.cam_fov       = math.radians(self.get_parameter('camera_fov_deg').value)
        self.conf_thresh   = self.get_parameter('confidence_threshold').value

    def _camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def _image_callback(self, msg: Image):
        """이미지 수신 및 주차구역 감지"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'이미지 변환 오류: {e}')
            return

        spots, debug_img = self._detect_parking_spots(frame)

        # 발행
        for spot_data in spots:
            if spot_data['confidence'] >= self.conf_thresh:
                self._publish_spot(msg.header, spot_data)

        if self.pub_debug and debug_img is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except Exception:
                pass

    def _detect_parking_spots(self, frame: np.ndarray
                              ) -> Tuple[List[dict], Optional[np.ndarray]]:
        """
        흰색 사각형 주차선 감지 메인 함수

        반환:
            spots: 감지된 주차구역 정보 목록
            debug_img: 시각화 이미지
        """
        h, w = frame.shape[:2]
        debug_img = frame.copy()
        spots = []

        # ---- 1. 전처리 ----
        # 가우시안 블러로 노이즈 감소
        blur = cv2.GaussianBlur(frame, (5, 5), 0)

        # HSV 변환
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # 흰색 마스크
        mask_white = cv2.inRange(hsv, self.white_hsv_lower, self.white_hsv_upper)

        # ---- 2. 형태학적 연산 ----
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask_closed = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)
        mask_opened = cv2.morphologyEx(mask_closed, cv2.MORPH_OPEN, kernel)

        # ---- 3. 엣지 검출로 선 강조 ----
        edges = cv2.Canny(mask_opened, 50, 150)
        kernel_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges_dilated = cv2.dilate(edges, kernel_dilate, iterations=1)

        # ---- 4. 컨투어 검출 ----
        contours, _ = cv2.findContours(
            edges_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        spot_id = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area_px or area > self.max_area_px:
                continue

            # 최소 외접 사각형
            rect = cv2.minAreaRect(cnt)
            center, (rw, rh), angle = rect

            # 긴 변이 항상 length
            rect_w = max(rw, rh)
            rect_h = min(rw, rh)
            if rect_h < 1e-6:
                continue

            ar = rect_w / rect_h  # 종횡비
            if ar < self.ar_min or ar > self.ar_max:
                continue

            # 사각형 점수 계산 (볼록성, 충진률)
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            convexity = area / hull_area if hull_area > 0 else 0.0

            # 사각형 근사
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            is_quad = len(approx) == 4

            # 신뢰도 계산
            confidence = self._compute_confidence(
                convexity=convexity,
                is_quad=is_quad,
                aspect_ratio=ar,
                area_ratio=area / (w * h))

            if confidence < self.conf_thresh * 0.5:
                continue

            # ---- 5. 실세계 좌표 추정 ----
            world_pos = self._pixel_to_world(
                center[0], center[1], w, h)

            # 주차선 방향각 (이미지 좌표계 → 차량 좌표계)
            # angle은 OpenCV minAreaRect 기준 (수평 기준 시계방향)
            spot_angle = self._compute_spot_angle(angle, rw, rh)

            spots.append({
                'spot_id':    spot_id,
                'center_px':  center,
                'rect':       rect,
                'world_x':    world_pos[0],
                'world_y':    world_pos[1],
                'angle_rad':  spot_angle,
                'confidence': confidence,
                'contour':    cnt,
            })

            # 디버그 시각화
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            cv2.drawContours(debug_img, [box], 0, (0, 255, 0), 2)
            cv2.circle(debug_img,
                       (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)
            cv2.putText(debug_img,
                        f'P{spot_id} conf={confidence:.2f}',
                        (int(center[0])+10, int(center[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            spot_id += 1

        # 마스크 시각화 (오른쪽 하단에 섬네일)
        thumb = cv2.resize(mask_opened, (w//4, h//4))
        thumb_bgr = cv2.cvtColor(thumb, cv2.COLOR_GRAY2BGR)
        debug_img[h - h//4:h, 0:w//4] = thumb_bgr
        cv2.putText(debug_img, 'White Mask',
                    (5, h - h//4 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        return spots, debug_img

    def _compute_confidence(self, convexity: float, is_quad: bool,
                            aspect_ratio: float, area_ratio: float) -> float:
        """주차구역 감지 신뢰도 계산 [0, 1]"""
        score = 0.0

        # 볼록성 (높을수록 사각형에 가까움)
        score += 0.4 * min(convexity, 1.0)

        # 4각형 여부
        if is_quad:
            score += 0.3

        # 종횡비 (주차구역 일반 비율 1:2 ~ 1:3)
        ideal_ar = self.spot_length_m / self.spot_width_m
        ar_score = 1.0 - min(abs(aspect_ratio - ideal_ar) / ideal_ar, 1.0)
        score += 0.2 * ar_score

        # 면적 비율 (너무 크거나 작으면 감점)
        if 0.01 < area_ratio < 0.5:
            score += 0.1

        return min(score, 1.0)

    def _pixel_to_world(self, px: float, py: float,
                        img_w: int, img_h: int) -> Tuple[float, float]:
        """
        픽셀 좌표 → 차량 좌표계 실세계 위치 추정
        핀홀 카메라 모델 + 지면 투영

        가정: 카메라가 차량 전방을 바라봄, 지면은 평면
        """
        if self.camera_info is not None:
            # 카메라 내부 파라미터 사용
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]
        else:
            # 기본값 (FOV 기반 추정)
            fx = img_w / (2 * math.tan(self.cam_fov / 2))
            fy = fx
            cx = img_w / 2.0
            cy = img_h / 2.0

        # 픽셀 → 정규화 좌표
        nx = (px - cx) / fx
        ny = (py - cy) / fy

        # 지면 투영 (핀홀 모델)
        # 카메라 높이 h, 피치 α 가정 시:
        # 지면까지의 방향 벡터 계산
        cos_p = math.cos(self.cam_pitch)
        sin_p = math.sin(self.cam_pitch)

        # 카메라 좌표계에서 방향 벡터
        ray_x = nx
        ray_y = ny
        ray_z = 1.0

        # 피치 보정 후 지면 교점
        # 카메라 → 지면: t = h / (ray_z*cos_p + ray_y*sin_p)
        denom = ray_z * cos_p - ray_y * sin_p
        if abs(denom) < 1e-6:
            return (5.0, 0.0)  # 기본값 (감지 불확실)

        t = self.cam_h / denom

        if t < 0.1 or t > 20.0:  # 유효 범위 밖
            return (3.0, 0.0)

        # 차량 좌표계
        world_x = t * ray_z * cos_p  # 전방 거리
        world_y = t * ray_x           # 측방 거리

        return (world_x, world_y)

    def _compute_spot_angle(self, cv_angle: float,
                            rw: float, rh: float) -> float:
        """
        OpenCV minAreaRect 각도 → 차량 좌표계 주차방향 [rad]
        """
        # OpenCV 각도: [-90, 0) 범위, 긴 변 기준
        if rw < rh:
            angle_deg = cv_angle + 90.0
        else:
            angle_deg = cv_angle

        # 차량 기준 (전방=0, 좌=양수)
        angle_rad = math.radians(angle_deg)
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def _publish_spot(self, header: Header, spot_data: dict):
        """주차구역 정보 발행"""
        if self.spot_pub is None:
            return

        msg = ParkingSpot()
        msg.header = header

        # 자세
        msg.spot_pose = Pose()
        msg.spot_pose.position.x = spot_data['world_x']
        msg.spot_pose.position.y = spot_data['world_y']
        msg.spot_pose.position.z = 0.0

        # 방향 → 쿼터니언
        angle = spot_data['angle_rad']
        msg.spot_pose.orientation.z = math.sin(angle / 2)
        msg.spot_pose.orientation.w = math.cos(angle / 2)

        msg.width      = self.spot_width_m
        msg.length     = self.spot_length_m
        msg.confidence = spot_data['confidence']
        msg.spot_id    = spot_data['spot_id']
        msg.is_occupied = False
        msg.is_valid   = True

        self.spot_pub.publish(msg)
        self.get_logger().info(
            f'주차구역 감지: ID={msg.spot_id}, '
            f'위치=({msg.spot_pose.position.x:.2f}, {msg.spot_pose.position.y:.2f})m, '
            f'신뢰도={msg.confidence:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpotDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
