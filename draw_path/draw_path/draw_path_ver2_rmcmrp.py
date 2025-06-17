import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
import math
from collections import deque

def moving_average(point_queue):
    """
    큐에 담긴 좌표들의 단순 이동평균을 구함.
    (노이즈 제거용 smoothing)
    Args:
        point_queue (collections.deque): (x, y) 좌표가 저장된 큐
    Returns:
        (float, float): 평균 x, y 좌표
    """
    n = len(point_queue)
    avg_x = sum([p[0] for p in point_queue]) / n
    avg_y = sum([p[1] for p in point_queue]) / n
    return (avg_x, avg_y)

class DrawingNode(Node):
    """
    ROS2 노드 + Tkinter GUI가 결합된 그림판 클래스
    마우스로 경로를 그리면, 실시간 smoothing을 거쳐 
    Float32MultiArray 타입으로 Publish 한다.
    """

    def __init__(self):
        super().__init__('drawing_pad_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'drawing_path', 10)
        self.path = []          # (x, y) 실제 경로 (Publish 대상)
        self.raw_path = []      # 원본(노이즈 포함) 좌표 시퀀스

        # smoothing 및 노이즈 제거를 위한 파라미터
        self.smooth_queue = deque(maxlen=5) # 최근 5개 좌표로 이동평균
        self.min_dist = 2.5        # 2.5픽셀 이내 진동은 무시
        self.min_publish_dist = 8.0 # 8픽셀 이상 움직이면 path에 추가

        # Tkinter GUI 생성
        self.root = tk.Tk()
        self.root.title("ROS2 Drawing Pad")
        self.canvas = tk.Canvas(self.root, width=480, height=480, bg="white")
        self.canvas.pack()
        self.canvas.bind("<ButtonPress-1>", self.start_draw)
        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.end_draw)

        self.button_frame = tk.Frame(self.root)
        self.button_frame.pack()
        self.publish_button = tk.Button(self.button_frame, text="Publish Path", command=self.publish_path)
        self.publish_button.pack(side=tk.LEFT)
        self.clear_button = tk.Button(self.button_frame, text="Clear", command=self.clear_canvas)
        self.clear_button.pack(side=tk.LEFT)

    def start_draw(self, event):
        pt = (event.x, event.y)
        self.raw_path = [pt]
        self.path = [pt]
        self.smooth_queue.clear()
        self.smooth_queue.append(pt)

    def draw(self, event):
        pt = (event.x, event.y)
        self.smooth_queue.append(pt)
        filtered_pt = moving_average(self.smooth_queue)

        # 미세 진동 무시: 이전 점과 가까우면 return
        if self.raw_path:
            prev_pt = self.raw_path[-1]
            dist = math.hypot(filtered_pt[0] - prev_pt[0], filtered_pt[1] - prev_pt[1])
            if dist < self.min_dist:
                return

        self.raw_path.append(filtered_pt)
        # path에 일정거리마다만 추가
        if not self.path:
            self.path.append(filtered_pt)
        else:
            prev = self.path[-1]
            dist = math.hypot(filtered_pt[0] - prev[0], filtered_pt[1] - prev[1])
            if dist >= self.min_publish_dist:
                self.path.append(filtered_pt)
                self.canvas.create_line(prev[0], prev[1], filtered_pt[0], filtered_pt[1], fill="black", width=2)
        # path가 2개 이상이면 직선 그리기 (화면에 실시간 선 그리기)
        if len(self.path) >= 2:
            x1, y1 = self.path[-2]
            x2, y2 = self.path[-1]
            self.canvas.create_line(x1, y1, x2, y2, fill="black", width=2)

    def end_draw(self, event):
        pt = (event.x, event.y)
        self.smooth_queue.append(pt)
        filtered_pt = moving_average(self.smooth_queue)
        self.raw_path.append(filtered_pt)
        if not self.path or math.hypot(filtered_pt[0] - self.path[-1][0], filtered_pt[1] - self.path[-1][1]) >= self.min_publish_dist:
            self.path.append(filtered_pt)

    def publish_path(self):
        if not self.path:
            self.get_logger().warn("Path is empty, not publishing.")
            return
        msg = Float32MultiArray()
        flat_path = []
        for x, y in self.path:
            x_cm = x / 10.0
            y_cm = y / 10.0
            flat_path.extend([float(x_cm), float(y_cm)])
        msg.data = flat_path
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published path with {len(self.path)} points in cm scale.")
        self.get_logger().info(f"Published path with {msg} points in cm scale.")

    def clear_canvas(self):
        self.canvas.delete("all")
        self.path = []
        self.raw_path = []
        self.smooth_queue.clear()
        self.get_logger().info("Canvas and path cleared.")

    def run(self):
        self._ros_spin()
        self.root.mainloop()

    def _ros_spin(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(10, self._ros_spin)

def main(args=None):
    rclpy.init(args=args)
    node = DrawingNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
