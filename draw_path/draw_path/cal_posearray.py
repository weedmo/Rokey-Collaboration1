#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose
import math


def yaw_to_quaternion(yaw):
    """
    2D yaw(라디안)을 ROS 쿼터니언으로 변환(z축 회전 전용)
    Args:
        yaw (float): z축 기준 방향 각(라디안)
    Returns:
        Quaternion: geometry_msgs.msg.Quaternion 객체
    """
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class PathToPoseArrayNode(Node):
    """
    Float32MultiArray로 받은 2D 경로(x, y, x, y, ...)를
    PoseArray(x, y, yaw → quaternion)로 변환해 publish하는 ROS2 노드
    """
    def __init__(self):
        super().__init__('path_to_posearray_node')
        self.sub = self.create_subscription(
            Float32MultiArray,
            'drawing_path_resampled',   # ← 구독 토픽만 바꿔주기!
            self.cb,
            10
        )
        self.pub = self.create_publisher(
            PoseArray,
            'domino_pose_array',
            10
        )

    def cb(self, msg):
        """
        drawing_path 토픽 수신 콜백
        - (x, y) 경로 복원
        - 점 간 yaw(방향 각) 계산
        - yaw → 쿼터니언 변환
        - PoseArray로 publish
        """
        data = msg.data
        if len(data) < 4 or len(data) % 2 != 0:
            self.get_logger().warn("Input path data invalid or too short.")
            return

        # (x, y) 좌표 복원
        path = []
        for i in range(0, len(data), 2):
            x, y = data[i], data[i+1]
            path.append((x, y))

        # 각 점별 yaw 계산 (연속 두 점의 벡터 방향)
        xyaw_list = []
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            dx = x2 - x1
            dy = y2 - y1
            yaw = math.atan2(dy, dx)
            xyaw_list.append((x1, y1, yaw))
        # 마지막 점: 이전 yaw 유지
        if len(path) >= 2:
            x_last, y_last = path[-1]
            _, _, last_yaw = xyaw_list[-1]
            xyaw_list.append((x_last, y_last, last_yaw))
        else:
            x_last, y_last = path[0]
            xyaw_list.append((x_last, y_last, 0.0))

        deg_list = [(math.degrees(yaw)) for _, _, yaw in xyaw_list]
        self.get_logger().info(f"[DEG] (x, y, yaw(deg)): {deg_list}")

        # PoseArray 메시지 생성 및 publish
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "base_link"  # 환경에 맞게 수정

        for x, y, yaw in xyaw_list:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = 0.0
            pose.orientation = yaw_to_quaternion(yaw)
            pose_array.poses.append(pose)

        self.pub.publish(pose_array)
        self.get_logger().info(f"Published {len(xyaw_list)} domino poses to 'domino_pose_array'")
        self.get_logger().info(f"Published {pose_array} domino poses to 'domino_pose_array'")

def main(args=None):
    """
    노드 실행 진입점. ROS2 노드 생성 및 spin
    """
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