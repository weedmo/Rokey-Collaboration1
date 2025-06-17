import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
import math
from collections import deque

def catmull_rom_spline(p0, p1, p2, p3, n_points=48):
    """
    Catmull-Rom Spline 보간법으로 4개의 점(p0~p3)을 이용하여 
    곡선을 만들고, 등간격으로 n_points개의 포인트를 샘플링합니다.
    
    Args:
        p0, p1, p2, p3 (tuple): 각 점의 (x, y) 좌표
        n_points (int): 반환할 포인트 개수
    Returns:
        points (list): (x, y) 좌표가 담긴 리스트
    """
    points = []
    for i in range(n_points):
        t = i / (n_points - 1)  # 0~1 사이의 정규화 파라미터
        t2 = t * t
        t3 = t2 * t
        # Catmull-Rom Spline 공식
        x = 0.5 * (
            (2 * p1[0]) +
            (-p0[0] + p2[0]) * t +
            (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2 +
            (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3
        )
        y = 0.5 * (
            (2 * p1[1]) +
            (-p0[1] + p2[1]) * t +
            (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2 +
            (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3
        )
        points.append((x, y))
    return points

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
    마우스로 경로를 그리면, 실시간 smoothing 및 Catmull-Rom Spline으로
    경로를 부드럽게 보간하여, Float32MultiArray 타입으로 Publish 한다.
    """

    def __init__(self):
        """
        노드 초기화, GUI 생성, Publisher 세팅, 필터링 파라미터 등 설정
        """
        super().__init__('drawing_pad_node')
        # ROS2 Publisher: 토픽명 'drawing_path', 메시지 타입 Float32MultiArray
        self.publisher_ = self.create_publisher(Float32MultiArray, 'drawing_path', 10)
        self.path = []          # (x, y) 실제 보간 경로 (Publish 대상)
        self.raw_path = []      # 원본(노이즈 포함) 좌표 시퀀스
        self.prev_smooth = None # 마지막 Catmull-Rom 곡선 샘플링 결과 저장

        # smoothing 및 노이즈 제거를 위한 파라미터
        self.smooth_queue = deque(maxlen=5) # 최근 5개 좌표로 이동평균
        self.min_dist = 2.5        # 2.5픽셀 이내 진동은 무시
        self.min_publish_dist = 8.0 # 8픽셀 이상 움직이면 path에 추가

        # Tkinter GUI 생성
        self.root = tk.Tk()
        self.root.title("ROS2 Drawing Pad")
        self.canvas = tk.Canvas(self.root, width=480, height=480, bg="white")
        self.canvas.pack()
        # 마우스 이벤트와 메소드 연결
        self.canvas.bind("<ButtonPress-1>", self.start_draw)
        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.end_draw)

        # 버튼 UI
        self.button_frame = tk.Frame(self.root)
        self.button_frame.pack()
        self.publish_button = tk.Button(self.button_frame, text="Publish Path", command=self.publish_path)
        self.publish_button.pack(side=tk.LEFT)
        self.clear_button = tk.Button(self.button_frame, text="Clear", command=self.clear_canvas)
        self.clear_button.pack(side=tk.LEFT)

    def start_draw(self, event):
        """
        마우스 클릭(드로잉 시작) 이벤트 콜백
        시작점으로 경로 초기화, smoothing queue 초기화
        """
        pt = (event.x, event.y)
        self.raw_path = [pt]
        self.path = [pt]
        self.prev_smooth = None
        self.smooth_queue.clear()
        self.smooth_queue.append(pt)

    def draw(self, event):
        """
        마우스 드래그(이동) 이벤트 콜백
        - 이동평균을 통한 smoothing
        - Catmull-Rom Spline 보간 (최근 4개 포인트 기준)
        - 거리 기반 path에 추가 및 캔버스 선 그리기
        """
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
        if len(self.raw_path) >= 4:
            # 최근 4개 좌표로 곡선 보간
            p0 = self.raw_path[-4]
            p1 = self.raw_path[-3]
            p2 = self.raw_path[-2]
            p3 = self.raw_path[-1]
            smooth_pts = catmull_rom_spline(p0, p1, p2, p3, n_points=48)
            # 곡선 잇는 선 그리기 (직전 곡선과 이어서)
            if self.prev_smooth is not None:
                x1, y1 = self.prev_smooth[-1]
                x2, y2 = smooth_pts[0]
                self.canvas.create_line(x1, y1, x2, y2, fill="black", width=2)
            # 현재 곡선 내 모든 점 잇기
            for i in range(1, len(smooth_pts)):
                x1, y1 = smooth_pts[i-1]
                x2, y2 = smooth_pts[i]
                self.canvas.create_line(x1, y1, x2, y2, fill="black", width=2)
            self.prev_smooth = smooth_pts

            # 등간격 거리 샘플링하여 path에 추가
            for pt in smooth_pts:
                if not self.path:
                    self.path.append(pt)
                else:
                    prev = self.path[-1]
                    dist = math.hypot(pt[0] - prev[0], pt[1] - prev[1])
                    if dist >= self.min_publish_dist:
                        self.path.append(pt)
        else:
            # 곡선 보간 전(좌표 4개 미만)은 직선으로 연결
            if len(self.raw_path) >= 2:
                x1, y1 = self.raw_path[-2]
                x2, y2 = self.raw_path[-1]
                self.canvas.create_line(x1, y1, x2, y2, fill="black", width=2)
                if not self.path or math.hypot(x2 - self.path[-1][0], y2 - self.path[-1][1]) >= self.min_publish_dist:
                    self.path.append((x2, y2))

    def end_draw(self, event):
        """
        마우스 버튼 뗄 때 호출(드로잉 끝)
        마지막 점 smoothing 및 path에 추가
        """
        pt = (event.x, event.y)
        self.smooth_queue.append(pt)
        filtered_pt = moving_average(self.smooth_queue)
        self.raw_path.append(filtered_pt)
        if not self.path or math.hypot(filtered_pt[0] - self.path[-1][0], filtered_pt[1] - self.path[-1][1]) >= self.min_publish_dist:
            self.path.append(filtered_pt)

    def publish_path(self):
        """
        path를 Float32MultiArray로 변환하여 ROS2 토픽으로 Publish
        (픽셀→cm 단위 변환: 480px=48cm이므로 1/10)
        """
        if not self.path:
            self.get_logger().warn("Path is empty, not publishing.")
            return
        msg = Float32MultiArray()
        flat_path = []

        # 픽셀 좌표를 cm로 변환해 일렬(flatten)로 배열에 저장
        for x, y in self.path:
            x_cm = x / 10.0
            y_cm = y / 10.0
            flat_path.extend([float(x_cm), float(y_cm)])

        msg.data = flat_path
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published path with {len(self.path)} points in cm scale.")
        self.get_logger().info(f"Published path with {msg} points in cm scale.")

    def clear_canvas(self):
        """
        캔버스, 경로 데이터 모두 초기화
        """
        self.canvas.delete("all")
        self.path = []
        self.raw_path = []
        self.prev_smooth = None
        self.smooth_queue.clear()
        self.get_logger().info("Canvas and path cleared.")

    def run(self):
        """
        ROS2 스핀(이벤트 처리)와 Tkinter mainloop를 결합하여 GUI와 ROS가 동시에 동작
        """
        self._ros_spin()
        self.root.mainloop()

    def _ros_spin(self):
        """
        ROS2 이벤트 루프를 Tkinter mainloop와 병행하여 돌리기 위한 내부 메소드
        0.01초마다 spin_once를 호출하여 ROS2 콜백을 처리
        """
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(10, self._ros_spin)  # 10ms마다 다시 호출

def main(args=None):
    """
    엔트리 포인트. ROS2 노드 생성 및 실행
    """
    rclpy.init(args=args)
    node = DrawingNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
