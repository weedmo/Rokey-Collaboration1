#!/usr/bin/env python3
"""
Headless 버전: Tkinter GUI 완전 제거!
- drawing_path_resampled 토픽을 구독
- Bezier 보간 적용
- yaw 계산 후 쿼터니언으로 변환
- domino_pose_array_GUI 토픽으로 PoseArray 메시지를 publish
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose
import math
import numpy as np

def cubic_bezier(p0, p1, p2, p3, n_points=10):
    """
    4개의 control point를 이용한 cubic Bezier curve 생성 함수

    Args:
        p0, p1, p2, p3 (ndarray): control point 좌표 (각각 shape: (2,))
        n_points (int): 곡선을 구성할 점의 개수

    Returns:
        ndarray: shape (n_points, 2), 곡선상의 좌표 목록
    """
    t = np.linspace(0, 1, n_points)
    curve = (
        ((1 - t) ** 3)[:, None] * p0 +
        3 * ((1 - t) ** 2)[:, None] * t[:, None] * p1 +
        3 * (1 - t)[:, None] * (t ** 2)[:, None] * p2 +
        (t ** 3)[:, None] * p3
    )
    return curve

def bezier_path(points, n_points_per_seg=10):
    """
    Bezier curve를 이용해 다수의 포인트로 이루어진 경로를 보간

    Args:
        points (list): [(x, y), ...] 형태의 원본 좌표 목록
        n_points_per_seg (int): 각 세그먼트(4개 점 기준)당 생성할 점 개수

    Returns:
        list: 보간된 (x, y) 좌표 리스트
    """
    points = np.array(points, dtype=float)
    if len(points) < 4:
        # 보간 불가능한 짧은 경로는 그대로 반환
        return [tuple(map(float, p)) for p in points]
    
    curve = []
    for i in range(0, len(points) - 3, 3):
        seg = cubic_bezier(points[i], points[i+1], points[i+2], points[i+3], n_points_per_seg)
        if i > 0:
            seg = seg[1:]  # 중복 제거
        curve.append(seg)

    # 남은 점들 처리 (Bezier 조건 불충분 시 직선으로 보간)
    if (len(points) - 1) % 3 != 0:
        tail = points[-2:]
        if len(tail) > 1:
            linear = np.linspace(tail[0], tail[1], n_points_per_seg)
            linear = linear[1:]  # 중복 제거
            curve.append(linear)
        else:
            curve.append(tail)

    curve = np.vstack(curve)
    return [tuple(map(float, p)) for p in curve]

def rpy_to_quaternion(roll, pitch, yaw):
    """
    Roll, Pitch, Yaw (라디안 단위)를 Quaternion(x, y, z, w)으로 변환

    Args:
        roll (float): x축 회전 (rad)
        pitch (float): y축 회전 (rad)
        yaw (float): z축 회전 (rad)

    Returns:
        dict: Quaternion 딕셔너리 {'x':, 'y':, 'z':, 'w':}
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return {
        'w': cr * cp * cy + sr * sp * sy,
        'x': sr * cp * cy - cr * sp * sy,
        'y': cr * sp * cy + sr * cp * sy,
        'z': cr * cp * sy - sr * sp * cy
    }

class PathPubSubNode(Node):
    """
    ROS2 Node:
    - drawing_path_resampled 토픽을 구독하여 Bezier 보간
    - 각 점 간 yaw 계산 + 쿼터니언 변환
    - PoseArray 메시지를 domino_pose_array_GUI 토픽으로 publish
    """
    def __init__(self):
        super().__init__('gui_path_pubsub')
        self.pub_pose = self.create_publisher(PoseArray, 'domino_pose_array_GUI', 10)
        self.sub = self.create_subscription(Float32MultiArray, 'drawing_path_resampled', self.cb, 10)
        self.latest_path = []
        self.bezier_path = []

    def cb(self, msg):
        """
        drawing_path_resampled 콜백
        - 메시지를 받아서 좌표 배열로 파싱
        - Bezier 보간 적용
        - 보간된 경로를 이용해 PoseArray 생성 및 publish
        """
        arr = msg.data
        if len(arr) < 2 or len(arr) % 2 != 0:
            self.get_logger().warn("수신한 좌표 길이가 잘못되었습니다.")
            self.latest_path = []
            self.bezier_path = []
            return

        self.latest_path = [(float(arr[i]), float(arr[i+1])) for i in range(0, len(arr), 2)]

        if len(self.latest_path) >= 4:
            self.bezier_path = bezier_path(self.latest_path, n_points_per_seg=3)
        else:
            self.bezier_path = self.latest_path[:]

        self.publish_posearray(self.bezier_path)

    def publish_posearray(self, path):
        """
        보간된 경로를 PoseArray 메시지로 변환하여 publish

        Args:
            path (list): [(x, y), ...] 형태의 2D 경로
        """
        if not hasattr(self, "pub_pose") or getattr(self.pub_pose, "handle", None) is None:
            print("[WARN] ROS2 퍼블리셔가 이미 소멸되었습니다. Publish를 생략합니다.")
            return
        if not rclpy.ok():
            print("[WARN] ROS2가 이미 종료되어 있어 publish를 생략합니다.")
            return

        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = "base_link"

        # 고정된 Roll/Pitch + 기준 yaw 설정
        roll_deg = 32.63
        pitch_deg = 180
        yaw_deg = 32.60

        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        base_yaw = math.radians(yaw_deg)

        # 경로를 따라 방향(yaw) 계산 후 쿼터니언 생성
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            yaw = base_yaw + math.atan2(y2 - y1, x2 - x1)
            quat = rpy_to_quaternion(roll, pitch, yaw)

            pose = Pose()
            pose.position.x = float(x1)
            pose.position.y = float(y1)
            pose.position.z = 0.0
            pose.orientation.x = quat['x']
            pose.orientation.y = quat['y']
            pose.orientation.z = quat['z']
            pose.orientation.w = quat['w']
            pa.poses.append(pose)

        # 마지막 점 처리
        if path:
            pose = Pose()
            pose.position.x = float(path[-1][0])
            pose.position.y = float(path[-1][1])
            pose.position.z = 0.0
            if len(path) > 1:
                x1, y1 = path[-2]
                x2, y2 = path[-1]
                yaw = math.atan2(y2 - y1, x2 - x1)
            else:
                yaw = 0.0
            quat = rpy_to_quaternion(roll, pitch, yaw)
            pose.orientation.x = quat['x']
            pose.orientation.y = quat['y']
            pose.orientation.z = quat['z']
            pose.orientation.w = quat['w']
            pa.poses.append(pose)

        try:
            self.pub_pose.publish(pa)
            self.get_logger().info(f'PoseArray published with {len(pa.poses)} poses')
        except Exception as e:
            print(f"[EXCEPTION] ROS2 publish_posearray 예외 발생: {e}")

def main(args=None):
    """
    ROS2 노드 실행 메인 함수
    """
    rclpy.init(args=args)
    node = PathPubSubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
