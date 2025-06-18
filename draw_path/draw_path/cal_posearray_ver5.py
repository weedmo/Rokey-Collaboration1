#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose
import tkinter as tk
import threading
import math
import numpy as np

# ----------- Bezier 보간 함수 -----------
def cubic_bezier(p0, p1, p2, p3, n_points=10):
    t = np.linspace(0, 1, n_points)
    curve = (
        ((1-t)**3)[:, None] * p0 +
        3 * ((1-t)**2)[:, None] * t[:, None] * p1 +
        3 * (1-t)[:, None] * (t**2)[:, None] * p2 +
        (t**3)[:, None] * p3
    )
    return curve

def bezier_path(points, n_points_per_seg=10):
    points = np.array(points, dtype=float)
    if len(points) < 4:
        return [tuple(map(float, p)) for p in points]
    curve = []
    for i in range(0, len(points)-3, 3):
        seg = cubic_bezier(points[i], points[i+1], points[i+2], points[i+3], n_points_per_seg)
        if i > 0:
            seg = seg[1:]  # 앞점 중복 제거
        curve.append(seg)
    # 마지막 남은 점(끝점 부근)은 선형으로 이어붙임
    if (len(points)-1) % 3 != 0:
        tail = points[-2:]
        if len(tail) > 1:
            linear = np.linspace(tail[0], tail[1], n_points_per_seg)
            linear = linear[1:]  # 앞점 중복 제거
            curve.append(linear)
        else:
            curve.append(tail)
    curve = np.vstack(curve)
    return [tuple(map(float, p)) for p in curve]
# ---------------------------------------

class PathPubSubNode(Node):
    def __init__(self):
        super().__init__('gui_path_pubsub')
        self.pub_pose = self.create_publisher(PoseArray, 'domino_pose_array', 10)
        self.pub_i32 = self.create_publisher(Int32MultiArray, 'domino_pose_array_with_index', 10)
        self.sub = self.create_subscription(Float32MultiArray, 'drawing_path_resampled', self.cb, 10)
        self.latest_path = []
        self.bezier_path = []

    def cb(self, msg):
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

    def publish_posearray(self, path):
        # ROS2 퍼블리셔 handle 체크 및 상태 확인
        if not hasattr(self, "pub_pose") or getattr(self.pub_pose, "handle", None) is None:
            print("[WARN] ROS2 퍼블리셔가 이미 소멸되었습니다. Publish를 생략합니다.")
            return
        if not rclpy.ok():
            print("[WARN] ROS2가 이미 종료되어 있어 publish를 생략합니다.")
            return
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = "base_link"
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i+1]
            yaw = math.atan2(y2 - y1, x2 - x1)
            pose = Pose()
            pose.position.x = float(x1)
            pose.position.y = float(y1)
            pose.position.z = 0.0
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            pa.poses.append(pose)
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
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            pa.poses.append(pose)
        try:
            self.pub_pose.publish(pa)
            self.get_logger().info(f'PoseArray published with {len(pa.poses)} poses')
            self.get_logger().info(f'{pa} poses')
        except Exception as e:
            print(f"[EXCEPTION] ROS2 publish_posearray 예외 발생: {e}")

    def publish_with_index(self, selected_indices, n_domino):
        # ROS2 퍼블리셔 handle 체크 및 상태 확인
        if not hasattr(self, "pub_i32") or getattr(self.pub_i32, "handle", None) is None:
            print("[WARN] ROS2 퍼블리셔가 이미 소멸되었습니다. Publish를 생략합니다.")
            return
        if not rclpy.ok():
            print("[WARN] ROS2가 이미 종료되어 있어 publish를 생략합니다.")
            return
        data = []
        for idx in selected_indices:
            data += [idx, n_domino]
        msg = Int32MultiArray()
        msg.data = data
        try:
            self.pub_i32.publish(msg)
            self.get_logger().info(f'Int32MultiArray (index, n_domino) published with {len(selected_indices)} dominos')
            self.get_logger().info(f'{msg}')
        except Exception as e:
            print(f"[EXCEPTION] ROS2 publish_with_index 예외 발생: {e}")

class PathGUI:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.root = tk.Tk()
        self.root.title("Domino Path Publisher")

        self.canvas = tk.Canvas(self.root, width=480, height=480, bg='white')
        self.canvas.grid(row=0, column=0, columnspan=4, padx=10, pady=10)

        tk.Label(self.root, text="좌표 인덱스(1,2,3):").grid(row=1, column=0, sticky='e')
        self.idx_entry = tk.Entry(self.root, width=12)
        self.idx_entry.insert(0, "1")
        self.idx_entry.grid(row=1, column=1, sticky='w', columnspan=2)

        tk.Label(self.root, text="쌓을 개수:").grid(row=1, column=2, sticky='e')
        self.cnt_entry = tk.Entry(self.root, width=5)
        self.cnt_entry.insert(0, "10")
        self.cnt_entry.grid(row=1, column=3, sticky='w')

        self.pub_btn = tk.Button(self.root, text="Publish", command=self.on_publish)
        self.pub_btn.grid(row=2, column=0, columnspan=4, pady=10)

        self.canvas.create_text(
            480 - 5, 10,
            text="(0, 0)", anchor="ne",
            fill="blue", font=("Arial", 12, "bold")
        )

        self.last_drawn = []
        self.selected_indices = []
        self.n_domino = None
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._poll_path()

    def _poll_path(self):
        path = self.ros_node.bezier_path if len(self.ros_node.bezier_path) > 0 else self.ros_node.latest_path
        if path != self.last_drawn:
            self.last_drawn = list(path)
            self.draw_path(path)
        self.root.after(100, self._poll_path)

    def draw_path(self, path):
        self.canvas.delete("all")
        if not path:
            return
        xs = [x for x, y in path]
        ys = [y for x, y in path]
        pad = 40
        W, H = 480, 480
        if max(xs) - min(xs) > 1e-5:
            scale_x = (W - 2*pad) / (max(xs) - min(xs) + 1e-5)
        else:
            scale_x = 1.0
        if max(ys) - min(ys) > 1e-5:
            scale_y = (H - 2*pad) / (max(ys) - min(ys) + 1e-5)
        else:
            scale_y = 1.0
        def tx(x): return W - (pad + int((x - min(xs)) * scale_x))
        def ty(y): return pad + int((y - min(ys)) * scale_y)

        yaws = []
        for i in range(len(path)):
            if i < len(path) - 1:
                x1, y1 = path[i]
                x2, y2 = path[i+1]
                yaw = math.atan2(y2 - y1, -(x2 - x1))
            elif len(path) > 1:
                x1, y1 = path[-2]
                x2, y2 = path[-1]
                yaw = math.atan2(y2 - y1, -(x2 - x1))
            else:
                yaw = 0.0
            yaws.append(yaw)

        for i, (x, y) in enumerate(path):
            cx, cy = tx(x), ty(y)
            if i in getattr(self, 'selected_indices', []):
                self.canvas.create_oval(cx-7, cy-7, cx+7, cy+7, fill='red', outline='black', width=2)
                self.canvas.create_text(cx, cy+18, text=f"{self.n_domino}개", fill='red', font=("Arial", 11, "bold"))
            else:
                self.canvas.create_oval(cx-5, cy-5, cx+5, cy+5, fill='skyblue', outline='black')
            self.canvas.create_text(cx, cy-15, text=str(i+1), fill='red', font=("Arial", 12, "bold"))
            if i > 0:
                px, py = path[i-1]
                self.canvas.create_line(tx(px), ty(py), cx, cy, fill='black', width=2)

            arrow_len = 25
            yaw = yaws[i]
            ex = cx + arrow_len * math.cos(yaw)
            ey = cy + arrow_len * math.sin(yaw)
            self.canvas.create_line(cx, cy, ex, ey, fill='green', width=2, arrow=tk.LAST, arrowshape=(12,15,6))

    def on_publish(self):
        # 퍼블리셔 핸들, 상태 체크 및 예외 방어
        if not hasattr(self.ros_node, "pub_pose") or getattr(self.ros_node.pub_pose, "handle", None) is None:
            print("[WARN] ROS2 퍼블리셔가 이미 소멸되었습니다. Publish를 생략합니다.")
            return
        if not rclpy.ok():
            print("[WARN] ROS2가 이미 종료되어 있어 publish를 생략합니다.")
            return
        try:
            idx_str = self.idx_entry.get()
            n_domino = int(self.cnt_entry.get())
            if n_domino <= 0:
                return
            selected_indices = []
            for part in idx_str.split(','):
                idx = int(part.strip())
                if idx < 1:
                    return
                selected_indices.append(idx - 1)
        except ValueError:
            return
        path = self.ros_node.bezier_path if len(self.ros_node.bezier_path) > 0 else self.ros_node.latest_path
        if not path:
            return
        for idx in selected_indices:
            if idx < 0 or idx >= len(path):
                return
        try:
            self.ros_node.publish_posearray(path)
            self.ros_node.publish_with_index(selected_indices, n_domino)
        except Exception as e:
            print(f"[EXCEPTION] publish 중 예외 발생: {e}")
            return
        self.selected_indices = selected_indices
        self.n_domino = n_domino
        self.draw_path(path)

    def on_close(self):
        print("프로그램 종료: ROS2 shutdown 및 Tkinter 종료")
        rclpy.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()

def ros2_spin_thread(node):
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()

def main():
    rclpy.init()
    node = PathPubSubNode()
    threading.Thread(target=ros2_spin_thread, args=(node,), daemon=True).start()
    gui = PathGUI(node)
    gui.run()

if __name__ == '__main__':
    main()
