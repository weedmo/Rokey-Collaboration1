#!/usr/bin/env python3
"""
2D 경로 좌표를 등간격(Uniform Interval)으로 샘플링하는 ROS2 노드 예제.

주요 기능
---------
- Float32MultiArray (x0, y0, x1, y1, ...) 형식의 경로 데이터 구독
- 지정된 간격(기본 3.0cm)마다 경로 포인트 샘플링 (간격은 파라미터로 변경 가능)
- 등간격 샘플링 결과를 Float32MultiArray로 publish
- ROS2 기반 실시간 동작 및 파라미터 동적 조정 지원

입출력 토픽
----------
- 입력: 'drawing_path'            (Float32MultiArray)
- 출력: 'drawing_path_resampled'  (Float32MultiArray)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class PathResamplerNode(Node):
    """
    입력 경로(좌표 시퀀스)를 받아 **등간격**으로 다시 샘플링하여 publish하는 ROS2 노드.

    - 입력: Float32MultiArray ("drawing_path")  
      (x0, y0, x1, y1, ..., xn, yn) 1차원 좌표 배열  
    - 출력: Float32MultiArray ("drawing_path_resampled")  
      등간격 샘플링된 (x, y) 좌표 시퀀스

    파라미터
    ----------
    interval : float, default=3.0
        샘플링 간격(단위: cm)
    """

    def __init__(self):
        """
        ROS2 노드 초기화  
        - 구독자(subscriber)와 퍼블리셔(publisher) 생성
        - interval 파라미터 선언
        """
        super().__init__('drawing_path_resampler_node')
        # 입력 좌표 경로 구독
        self.sub = self.create_subscription(
            Float32MultiArray,
            'drawing_path',
            self.cb,
            10
        )
        # 샘플링 결과 publish
        self.pub = self.create_publisher(
            Float32MultiArray,
            'drawing_path_resampled',
            10
        )
        # 샘플링 간격 파라미터 (동적으로 조절 가능)
        self.declare_parameter('interval', 3.0)  # 단위: cm

    def cb(self, msg):
        """
        좌표 경로 메시지 수신 콜백  
        - 등간격 샘플링 적용
        - 결과 publish

        Parameters
        ----------
        msg : Float32MultiArray
            (x0, y0, x1, y1, ..., xn, yn) 좌표 시퀀스
        """
        interval = float(self.get_parameter('interval').value)
        data = msg.data
        # 데이터 유효성 검사 (최소 2개 포인트, 짝수 개수)
        if len(data) < 4 or len(data) % 2 != 0:
            self.get_logger().warn("Input path data invalid or too short.")
            return
        # (N, 2) 배열로 변환
        points = np.array(data).reshape(-1, 2)
        # -------------------------
        # 등간격 샘플링 알고리즘 시작
        # -------------------------
        resampled = [points[0]]   # 시작점 추가
        for pt in points[1:]:
            dist = np.linalg.norm(pt - resampled[-1])
            if dist >= interval:
                resampled.append(pt)
        # 마지막 점은 항상 포함
        if not np.allclose(resampled[-1], points[-1]):
            resampled.append(points[-1])
        # -------------------------
        # 결과 플랫(flat) 배열로 변환 및 publish
        # -------------------------
        out = Float32MultiArray()
        out.data = [float(x) for pt in resampled for x in pt]
        self.pub.publish(out)
        # 디버깅 및 상태 출력
        self.get_logger().info(f"Published {len(resampled)} resampled points.")
        self.get_logger().info(f"Published {out} resampled points.")

def main(args=None):
    """
    ROS2 노드 실행 메인 진입점

    사용 예시:
      $ ros2 run <패키지명> <본_파이썬_파일명>.py

    파라미터(간격) 변경:
      $ ros2 run <패키지명> <본_파이썬_파일명>.py --ros-args -p interval:=5.0
    """
    rclpy.init(args=args)
    node = PathResamplerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
