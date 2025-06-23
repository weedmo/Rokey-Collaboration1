#!/usr/bin/env python3
"""
ROS2 + Tkinter 통합 노드 (IntegratedGuiNode)
- drawing_path 토픽으로 좌표 Publish
- drawing_path_resampled, domino_pose_array_GUI 토픽 Subscribe
- GUI에서 경로 그리기 및 보간 결과 시각화
- 인덱스 및 개수 정보를 함께 publish (제한 포함)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from geometry_msgs.msg import PoseArray, Pose
import tkinter as tk
import threading
import math
from collections import deque
from tkinter import messagebox

class IntegratedGuiNode(Node):
    """
    ROS2 + Tkinter GUI 노드 클래스
    """
    def __init__(self):
        super().__init__('integrated_gui_node')

        # --- ROS2 Publisher ---
        self.pub_draw_path = self.create_publisher(Float32MultiArray, 'drawing_path', 10)
        self.pub_index = self.create_publisher(Int32MultiArray, 'domino_pose_array_with_index', 10)
        self.pub_posearray = self.create_publisher(PoseArray, 'domino_pose_array', 10)

        # --- ROS2 Subscriber ---
        self.sub_draw = self.create_subscription(Float32MultiArray, 'drawing_path', self.cb_draw, 10)
        self.sub_resampled = self.create_subscription(Float32MultiArray, 'drawing_path_resampled', self.cb_resampled, 10)
        self.sub_pose = self.create_subscription(PoseArray, 'domino_pose_array_GUI', self.cb_pose, 10)

        # --- 내부 상태 버퍼 ---
        self.latest_draw_path = []
        self.latest_resampled_path = []
        self.latest_posearray = []  # Pose 객체 리스트

        self.selected_indices = []
        self.n_domino = None

        # --- GUI 초기화 ---
        self.root = tk.Tk()
        self.root.title("ROS2 통합 경로 GUI")
        self._setup_gui()
        self._ros_spin()

    def _setup_gui(self):
        """Tkinter GUI 레이아웃 설정"""
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # === 왼쪽 패널 (경로 입력) ===
        left_frame = tk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, padx=10, pady=10)
        tk.Label(left_frame, text="Draw Path (원본 입력)").pack()
        self.left_canvas = tk.Canvas(left_frame, width=480, height=480, bg='white')
        self.left_canvas.pack()
        self.left_canvas.bind("<ButtonPress-1>", self.start_draw)
        self.left_canvas.bind("<B1-Motion>", self.draw)
        self.left_canvas.bind("<ButtonRelease-1>", self.end_draw)
        btn_frame_left = tk.Frame(left_frame)
        btn_frame_left.pack(pady=5)
        tk.Button(btn_frame_left, text="Publish", command=self.left_publish).pack(side=tk.LEFT, padx=2)
        tk.Button(btn_frame_left, text="Clear", command=self.left_clear).pack(side=tk.LEFT, padx=2)

        # === 오른쪽 패널 (결과 시각화 및 조작) ===
        right_frame = tk.Frame(main_frame)
        right_frame.pack(side=tk.LEFT, padx=10, pady=10)
        tk.Label(right_frame, text="Final Path").pack()
        self.right_canvas = tk.Canvas(right_frame, width=480, height=480, bg='white')
        self.right_canvas.pack()
        btn_frame_right = tk.Frame(right_frame)
        btn_frame_right.pack(pady=5)
        tk.Button(btn_frame_right, text="Publish", command=self.right_publish).pack(side=tk.LEFT, padx=2)
        tk.Button(btn_frame_right, text="Clear", command=self.right_clear).pack(side=tk.LEFT, padx=2)

        # === 인덱스 및 개수 입력 ===
        idx_frame = tk.Frame(right_frame)
        idx_frame.pack(pady=2)
        tk.Label(idx_frame, text="인덱스:").pack(side=tk.LEFT)
        self.idx_entry = tk.Entry(idx_frame, width=10)
        self.idx_entry.pack(side=tk.LEFT)
        tk.Label(idx_frame, text="개수:").pack(side=tk.LEFT)
        self.cnt_entry = tk.Entry(idx_frame, width=5)
        self.cnt_entry.pack(side=tk.LEFT)
        self.cnt_entry.insert(0, "10")  # 기본 개수

        # === 드로잉 관련 상태 ===
        self.drawing_points = []
        self.left_last_point = None
        self.raw_path = []
        self.smooth_queue = deque(maxlen=5)
        self.min_dist = 2.5
        self.min_publish_dist = 8.0

    @staticmethod
    def moving_average(point_queue):
        """좌표 이동 평균 필터"""
        n = len(point_queue)
        avg_x = sum([p[0] for p in point_queue]) / n
        avg_y = sum([p[1] for p in point_queue]) / n
        return (avg_x, avg_y)

    # ------------------- 왼쪽 (경로 입력) ------------------- #
    def start_draw(self, event):
        """그리기 시작 이벤트"""
        pt = (event.x, event.y)
        self.raw_path = [pt]
        self.drawing_points = [pt]
        self.left_last_point = pt
        self.smooth_queue.clear()
        self.smooth_queue.append(pt)

    def draw(self, event):
        """그리기 진행 중 (마우스 드래그)"""
        pt = (event.x, event.y)
        self.smooth_queue.append(pt)
        filtered_pt = self.moving_average(self.smooth_queue)
        if self.raw_path:
            prev_pt = self.raw_path[-1]
            dist = math.hypot(filtered_pt[0] - prev_pt[0], filtered_pt[1] - prev_pt[1])
            if dist < self.min_dist:
                return
        self.raw_path.append(filtered_pt)
        if not self.drawing_points:
            self.drawing_points.append(filtered_pt)
        else:
            prev = self.drawing_points[-1]
            dist = math.hypot(filtered_pt[0] - prev[0], filtered_pt[1] - prev[1])
            if dist >= self.min_publish_dist:
                self.drawing_points.append(filtered_pt)
                self.left_canvas.create_line(prev[0], prev[1], filtered_pt[0], filtered_pt[1], fill="black", width=2)
        if len(self.drawing_points) >= 2:
            x1, y1 = self.drawing_points[-2]
            x2, y2 = self.drawing_points[-1]
            self.left_canvas.create_line(x1, y1, x2, y2, fill="black", width=2)
        self.left_last_point = filtered_pt

    def end_draw(self, event):
        """그리기 종료 이벤트"""
        pt = (event.x, event.y)
        self.smooth_queue.append(pt)
        filtered_pt = self.moving_average(self.smooth_queue)
        self.raw_path.append(filtered_pt)
        if not self.drawing_points or math.hypot(filtered_pt[0] - self.drawing_points[-1][0], filtered_pt[1] - self.drawing_points[-1][1]) >= self.min_publish_dist:
            self.drawing_points.append(filtered_pt)
        self.left_last_point = None

    def left_clear(self):
        """왼쪽 캔버스 초기화"""
        self.left_canvas.delete("all")
        self.drawing_points = []
        self.raw_path = []
        self.smooth_queue.clear()

    def left_publish(self):
        """왼쪽 경로 ROS2 토픽 publish"""
        if not self.drawing_points:
            self.get_logger().warn("왼쪽 path가 없습니다.")
            return
        width, height = 480, 480
        path_cm = []
        for x, y in self.drawing_points:
            x_conv = width - x  # 좌우 반전
            x_cm, y_cm = x_conv / 10.0, y / 10.0
            path_cm += [float(x_cm), float(y_cm)]
        msg = Float32MultiArray()
        msg.data = path_cm
        self.pub_draw_path.publish(msg)
        self.get_logger().info(f"왼쪽 path publish ({len(self.drawing_points)} pts)")

    # ------------------- 오른쪽 처리 및 검증 ------------------- #
    def right_clear(self):
        self.right_canvas.delete("all")
        self.selected_indices = []
        self.n_domino = None

    def right_publish(self):
        """오른쪽 index + PoseArray publish (검사 포함)"""
        try:
            idxs = [int(i.strip())-1 for i in self.idx_entry.get().split(',') if i.strip()]
            n_domino = int(self.cnt_entry.get())
        except Exception as e:
            self.get_logger().warn("오른쪽 입력값이 잘못됨")
            return

        # === 유효성 검사: 개수 제한 ===
        if n_domino >= 4:
            self.get_logger().warn("개수 값이 4 이상일 때는 publish할 수 없습니다.")
            messagebox.showwarning("경고", "개수는 3 이하만 입력 가능합니다!")
            return

        # --- PoseArray publish ---
        if self.latest_posearray:
            pa = PoseArray()
            pa.header.stamp = self.get_clock().now().to_msg()
            pa.header.frame_id = "base_link"
            for pose_in in self.latest_posearray:
                pose = Pose()
                pose.position.x = pose_in.position.x
                pose.position.y = pose_in.position.y
                pose.position.z = pose_in.position.z
                pose.orientation.x = pose_in.orientation.x
                pose.orientation.y = pose_in.orientation.y
                pose.orientation.z = pose_in.orientation.z
                pose.orientation.w = pose_in.orientation.w
                pa.poses.append(pose)
            self.pub_posearray.publish(pa)
            self.get_logger().info(f"[오른쪽 publish] PoseArray도 같이 publish ({len(pa.poses)} pts)")

        # --- Index + Count publish ---
        msg = Int32MultiArray()
        msg.data = sum([[idx, n_domino] for idx in idxs], [])  # 평탄화
        self.pub_index.publish(msg)
        self.get_logger().info(f"[오른쪽 publish] index publish ({msg.data})")

        self.selected_indices = idxs
        self.n_domino = n_domino
        self._draw_right_path()

    # ------------------- ROS 콜백 ------------------- #
    def cb_draw(self, msg):
        pts = list(msg.data)
        self.latest_draw_path = [(pts[i], pts[i+1]) for i in range(0, len(pts), 2)]
        self._draw_left_path()

    def cb_resampled(self, msg):
        pts = list(msg.data)
        self.latest_resampled_path = [(pts[i], pts[i+1]) for i in range(0, len(pts), 2)]
        self._draw_right_path()

    def cb_pose(self, msg):
        self.latest_posearray = [p for p in msg.poses]
        self._draw_right_path()

    def _draw_left_path(self):
        """
        좌측 캔버스(left_canvas)에 원본 입력 경로(drawing_path)를 시각화하는 함수
        - 파란색 선으로 경로 그리기
        - 각 포인트 위치에 번호와 원 표시
        - 좌표 스케일링 및 Tkinter 캔버스 좌표계로 변환 포함
        """
        self.left_canvas.delete("all")  # 기존 그림 초기화
        if not self.latest_draw_path:
            return

        W, H = 480, 480  # 캔버스 크기
        pad = 40         # 테두리 여백

        # X, Y 좌표 분리
        xs = [x for x, y in self.latest_draw_path]
        ys = [y for x, y in self.latest_draw_path]

        # 스케일링 계산
        if max(xs) - min(xs) > 1e-5:
            scale_x = (W - 2*pad) / (max(xs) - min(xs) + 1e-5)
        else:
            scale_x = 1.0
        if max(ys) - min(ys) > 1e-5:
            scale_y = (H - 2*pad) / (max(ys) - min(ys) + 1e-5)
        else:
            scale_y = 1.0

        # 좌표계 변환 함수 (좌우반전 포함)
        def tx(x): return W - (pad + int((x - min(xs)) * scale_x))
        def ty(y): return pad + int((y - min(ys)) * scale_y)

        # 선 그리기
        for i in range(1, len(self.latest_draw_path)):
            x1, y1 = self.latest_draw_path[i-1]
            x2, y2 = self.latest_draw_path[i]
            self.left_canvas.create_line(tx(x1), ty(y1), tx(x2), ty(y2), fill='blue', width=2)

        # 점 및 번호 표시
        for i, (x, y) in enumerate(self.latest_draw_path):
            cx, cy = tx(x), ty(y)
            self.left_canvas.create_oval(cx-4, cy-4, cx+4, cy+4, fill='skyblue', outline='black')
            self.left_canvas.create_text(cx, cy-10, text=str(i+1), fill='blue', font=("Arial", 10, "bold"))

    def _draw_right_path(self):
        """
        우측 캔버스(right_canvas)에 보간된 경로 및 최종 PoseArray를 시각화하는 함수
        - 회색: 샘플링된 경로
        - 빨간색: 최종 PoseArray 경로 (orientation 포함)
        - 초록색 화살표: yaw 방향 시각화
        - 선택된 index 강조 및 도미노 개수 표기
        """
        self.right_canvas.delete("all")
        W, H = 480, 480
        pad = 40

        # === 샘플링 경로 시각화 (회색) ===
        if self.latest_resampled_path:
            xs = [x for x, y in self.latest_resampled_path]
            ys = [y for x, y in self.latest_resampled_path]

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

            # 선 그리기
            for i in range(1, len(self.latest_resampled_path)):
                x1, y1 = self.latest_resampled_path[i-1]
                x2, y2 = self.latest_resampled_path[i]
                self.right_canvas.create_line(tx(x1), ty(y1), tx(x2), ty(y2), fill='gray', width=2)

            # 점 및 번호 표시
            for i, (x, y) in enumerate(self.latest_resampled_path):
                cx, cy = tx(x), ty(y)
                self.right_canvas.create_oval(cx-4, cy-4, cx+4, cy+4, fill='#cce6ff', outline='gray90')
                self.right_canvas.create_text(cx, cy-10, text=str(i+1), fill='#7faaff', font=("Arial", 10))

        # === 최종 PoseArray 시각화 (빨간색 + yaw 화살표) ===
        if self.latest_posearray:
            xs = [p.position.x for p in self.latest_posearray]
            ys = [p.position.y for p in self.latest_posearray]

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

            # 빨간 경로 선 연결
            for i in range(1, len(self.latest_posearray)):
                x1, y1 = self.latest_posearray[i-1].position.x, self.latest_posearray[i-1].position.y
                x2, y2 = self.latest_posearray[i].position.x, self.latest_posearray[i].position.y
                self.right_canvas.create_line(tx(x1), ty(y1), tx(x2), ty(y2), fill='red', width=2)

            # 점 및 yaw 방향 시각화
            for i, pose in enumerate(self.latest_posearray):
                x, y = pose.position.x, pose.position.y
                cx, cy = tx(x), ty(y)

                # 선택된 index 강조 (크게 원 + 개수 텍스트)
                if i in getattr(self, 'selected_indices', []):
                    self.right_canvas.create_oval(cx-7, cy-7, cx+7, cy+7, fill='red', outline='black', width=2)
                    if self.n_domino:
                        self.right_canvas.create_text(cx, cy+18, text=f"{self.n_domino}개", fill='red', font=("Arial", 11, "bold"))
                else:
                    self.right_canvas.create_oval(cx-5, cy-5, cx+5, cy+5, fill='orange', outline='black')

                self.right_canvas.create_text(cx, cy-15, text=str(i+1), fill='red', font=("Arial", 11, "bold"))

                # yaw 방향 계산 (좌우반전 포함)
                if i < len(self.latest_posearray) - 1:
                    x2, y2 = self.latest_posearray[i+1].position.x, self.latest_posearray[i+1].position.y
                    yaw = math.atan2(y2 - y, -(x2 - x))
                elif len(self.latest_posearray) > 1:
                    x1, y1 = self.latest_posearray[i-1].position.x, self.latest_posearray[i-1].position.y
                    yaw = math.atan2(y - y1, -(x - x1))
                else:
                    yaw = 0.0

                arrow_len = 25
                ex = cx + arrow_len * math.cos(yaw)
                ey = cy + arrow_len * math.sin(yaw)

                # yaw 방향 화살표 표시
                self.right_canvas.create_line(
                    cx, cy, ex, ey, fill='green', width=2,
                    arrow=tk.LAST, arrowshape=(12, 15, 6)
                )


    # ------------------- ROS + Tkinter 통합 실행 ------------------- #
    def _ros_spin(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(10, self._ros_spin)

# ------------------- 메인 함수 ------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = IntegratedGuiNode()
    node.root.mainloop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
