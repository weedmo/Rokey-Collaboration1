import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class PathResamplerNode(Node):
    def __init__(self):
        super().__init__('drawing_path_resampler_node')
        self.sub = self.create_subscription(
            Float32MultiArray,
            'drawing_path',
            self.cb,
            10
        )
        self.pub = self.create_publisher(
            Float32MultiArray,
            'drawing_path_resampled',
            10
        )
        self.declare_parameter('interval', 3.0)  # 단위: cm

    def cb(self, msg):
        interval = float(self.get_parameter('interval').value)
        data = msg.data
        if len(data) < 4 or len(data) % 2 != 0:
            self.get_logger().warn("Input path data invalid or too short.")
            return
        points = np.array(data).reshape(-1, 2)
        # 등간격 샘플링
        resampled = [points[0]]
        total = 0.0
        for pt in points[1:]:
            dist = np.linalg.norm(pt - resampled[-1])
            if dist >= interval:
                resampled.append(pt)
        # 마지막 점 추가
        if not np.allclose(resampled[-1], points[-1]):
            resampled.append(points[-1])
        # Flatten 후 publish
        out = Float32MultiArray()
        out.data = [float(x) for pt in resampled for x in pt]
        self.pub.publish(out)
        self.get_logger().info(f"Published {len(resampled)} resampled points.")
        self.get_logger().info(f"Published {out} resampled points.")

def main(args=None):
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
