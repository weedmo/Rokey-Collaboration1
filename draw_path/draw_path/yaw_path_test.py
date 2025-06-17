import matplotlib.pyplot as plt
import numpy as np

# 경로 예시
points = np.array([
    [25, 26],
    [30, 40],
    [50, 55],
    [60, 70],
])

def get_yaws(points, first_yaw_mode='from_origin'):
    yaws = []
    N = len(points)
    if first_yaw_mode == 'from_origin':
        # 첫 도미노: (0,0)→첫좌표
        x0, y0 = 0, 0
        x1, y1 = points[0]
        yaw = np.arctan2(y1 - y0, x1 - x0)
        yaws.append(yaw)
    elif first_yaw_mode == 'to_next':
        # 첫 도미노: 첫좌표→두번째좌표
        x1, y1 = points[0]
        x2, y2 = points[1]
        yaw = np.arctan2(y2 - y1, x2 - x1)
        yaws.append(yaw)
    else:
        raise ValueError("mode error")

    # 나머지 도미노들 (현재점→다음점)
    for i in range(1, N - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        yaw = np.arctan2(y2 - y1, x2 - x1)
        yaws.append(yaw)
    return yaws

def plot_dominos(points, yaws, color, label):
    # 첫 도미노는 points[1]부터 시작
    for i, yaw in enumerate(yaws):
        x, y = points[i]
        dx = np.cos(yaw)
        dy = np.sin(yaw)
        # 도미노 몸체 (길이 4, 폭 1)
        length = 4
        width = 1
        # 도미노 중앙에서 yaw방향으로 length만큼 그린다
        px = x - (length/2)*dx
        py = y - (length/2)*dy
        # 직사각형을 yaw 방향으로 회전
        rect = plt.Rectangle(
            (px, py), length, width,
            angle=np.degrees(yaw), color=color, alpha=0.4, label=None if i else label
        )
        plt.gca().add_patch(rect)
        # yaw 방향 화살표
        plt.arrow(x, y, 2*dx, 2*dy, head_width=1.5, head_length=2, fc=color, ec=color)
        # 각도 표시
        plt.text(x, y+2, f"{np.degrees(yaw):.1f}°", fontsize=9, color=color, ha='center')

plt.figure(figsize=(10, 8))
plt.plot(points[:, 0], points[:, 1], 'ko--', label='path')
plt.scatter(points[:, 0], points[:, 1], s=60, c='k')

# Case A: 첫 도미노 yaw = (0,0)→(25,26)
yaws_origin = get_yaws(points, 'from_origin')
plot_dominos(points, yaws_origin, 'blue', "A: (0,0)→1st yaw")

# Case B: 첫 도미노 yaw = (25,26)→(30,40)
yaws_next = get_yaws(points, 'to_next')
plot_dominos(points, yaws_next, 'red', "B: 1st->2nd yaw")

plt.legend(loc='upper left')
plt.axis('equal')
plt.title("compare between 1st yaw and 2nd yaw")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.show()
