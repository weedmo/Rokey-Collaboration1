#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import yaml
import os
from datetime import datetime
from collections import OrderedDict

class PathToYamlNode(Node):
    def __init__(self):
        super().__init__('path_to_yaml_node')
        self.sub = self.create_subscription(
            Float32MultiArray,
            'drawing_path',
            self.cb,
            10
        )
        self.yaml_save_dir = './'  # 저장할 디렉토리(필요시 변경)
        self.save_count = 0

    def cb(self, msg):
        data = msg.data
        if len(data) < 4 or len(data) % 2 != 0:
            self.get_logger().warn("Input path data invalid or too short.")
            return

        # (x, y) 좌표 복원
        path = []
        for i in range(0, len(data), 2):
            x, y = data[i], data[i+1]
            path.append((x, y))

        # 각 점별 yaw 계산
        xyaw = []
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            dx = x2 - x1
            dy = y2 - y1
            yaw = math.atan2(dy, dx)
            xyaw.append((x1, y1, yaw))
        # 마지막 점 (직전 yaw)
        if len(path) >= 2:
            x_last, y_last = path[-1]
            _, _, last_yaw = xyaw[-1]
            xyaw.append((x_last, y_last, last_yaw))
        else:
            x_last, y_last = path[0]
            xyaw.append((x_last, y_last, 0.0))

        # YAML 딕셔너리 생성 (domino1, domino2, ...)
        domino_dict = {}
        for i, (x, y, yaw) in enumerate(xyaw, 1):
            domino_dict[f'domino{i}'] = {
                'x': float(x),
                'y': float(y),
                'yaw': float(yaw)
            }

        # key가 반드시 'domino+숫자' 형태인지 확인 (안 맞으면 저장 X)
        for key in domino_dict:
            if not (key.startswith("domino") and key[6:].isdigit()):
                self.get_logger().error(f"Invalid domino key: {key} (domino 다음에 반드시 숫자가 있어야 합니다)")
                return

        # 숫자순 정렬(OrderedDict) 후 일반 dict로 변환해서 yaml 저장
        sorted_domino_dict = OrderedDict(
            sorted(
                domino_dict.items(),
                key=lambda x: int(x[0][6:])   # 'domino' 다음 숫자로 정렬
            )
        )

        # 파일명: domino_path_날짜_시각_N.yaml
        self.save_count += 1
        now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'domino_path_{now_str}_{self.save_count}.yaml'
        save_path = os.path.join(self.yaml_save_dir, filename)

        # 일반 dict로 변환해서 yaml 저장 (!!python/object/apply 없이)
        with open(save_path, 'w') as f:
            yaml.dump(dict(sorted_domino_dict), f, default_flow_style=False, allow_unicode=True)

        self.get_logger().info(f"Saved domino path (sorted, plain dict) to {save_path}")

def main(args=None):
    rclpy.init(args=args)
    node = PathToYamlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
