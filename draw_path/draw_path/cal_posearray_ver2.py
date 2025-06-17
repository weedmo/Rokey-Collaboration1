#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose
import math
import numpy as np

def yaw_to_quaternion(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def cubic_bezier(p0, p1, p2, p3, n_points=10):
    """Cubic Bezier curve for four control points"""
    points = []
    for t in np.linspace(0, 1, n_points):
        x = (1-t)**3*p0[0] + 3*(1-t)**2*t*p1[0] + 3*(1-t)*t**2*p2[0] + t**3*p3[0]
        y = (1-t)**3*p0[1] + 3*(1-t)**2*t*p1[1] + 3*(1-t)*t**2*p2[1] + t**3*p3[1]
        points.append((x, y))
    return points

class PathToPoseArrayNode(Node):
    def __init__(self):
        super().__init__('path_to_posearray_node')
        self.sub = self.create_subscription(
            Float32MultiArray,
            'drawing_path_resampled',
            self.cb,
            10
        )
        self.pub = self.create_publisher(
            PoseArray,
            'domino_pose_array',
            10
        )

    def cb(self, msg):
        data = msg.data
        if len(data) < 4 or len(data) % 2 != 0:
            self.get_logger().warn("Input path data invalid or too short.")
            return

        # 1. (x, y) 좌표 복원
        path = []
        for i in range(0, len(data), 2):
            x, y = data[i], data[i+1]
            path.append((x, y))

        # 2. 곡선/직선 보간 (옵션: 필요 없다면 아래 전체 block을 new_path = path로 간단히 변경 가능)
        # ----------- Bezier 보간/직선 구분 ---------------
        # (아래는 예시, 기존대로 곡선구간 판별해서 bezier만 적용)
        # 중앙값 yaw로 곡선/직선 판별
        yaws_for_thresh = []
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            dx = x2 - x1
            dy = y2 - y1
            yaws_for_thresh.append(math.atan2(dy, dx))
        if yaws_for_thresh:
            yaws_for_thresh.append(yaws_for_thresh[-1])
        else:
            yaws_for_thresh = [0.0]

        median_yaw = np.median(yaws_for_thresh)
        THRESH = 0.05  # 곡선 임계값 (라디안)
        bezier_indices = [i for i, yaw in enumerate(yaws_for_thresh) if abs(yaw - median_yaw) >= THRESH]

        new_path = []
        i = 0
        while i < len(path)-1:
            if i in bezier_indices:
                # 최소 4개 포인트가 있어야 cubic bezier 보간 가능
                if i+3 < len(path):
                    pts = cubic_bezier(path[i], path[i+1], path[i+2], path[i+3], n_points=7)
                    if not new_path or pts[0] != new_path[-1]:
                        new_path.extend(pts)
                    else:
                        new_path.extend(pts[1:])
                    i += 3
                else:
                    if not new_path or path[i] != new_path[-1]:
                        new_path.append(path[i])
                    i += 1
            else:
                if not new_path or path[i] != new_path[-1]:
                    new_path.append(path[i])
                i += 1
        if path[-1] != new_path[-1]:
            new_path.append(path[-1])
        # ---------------------------------------------

        # 3. yaw 계산: 모든 도미노는 (현재점 → 다음점)
        xyaw_list = []
        for i in range(len(new_path) - 1):
            x1, y1 = new_path[i]
            x2, y2 = new_path[i + 1]
            dx = x2 - x1
            dy = y2 - y1
            yaw = math.atan2(dy, dx)
            xyaw_list.append((x1, y1, yaw))
        # 마지막 점의 yaw는 이전 값 유지
        if len(new_path) >= 2:
            x_last, y_last = new_path[-1]
            _, _, last_yaw = xyaw_list[-1]
            xyaw_list.append((x_last, y_last, last_yaw))
        else:
            x_last, y_last = new_path[0]
            xyaw_list.append((x_last, y_last, 0.0))

        # 디버깅 로그 (degree)
        deg_list = [(math.degrees(yaw)) for x, y, yaw in xyaw_list]
        self.get_logger().info(f"[DEG] (yaw(deg)): {deg_list}")

        # 4. PoseArray 생성 및 publish
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "base_link"

        for x, y, yaw in xyaw_list:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = 0.0
            pose.orientation = yaw_to_quaternion(yaw)
            pose_array.poses.append(pose)

        self.pub.publish(pose_array)
        self.get_logger().info(f"Published {len(xyaw_list)} domino poses to 'domino_pose_array'")

def main(args=None):
    rclpy.init(args=args)
    node = PathToPoseArrayNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
