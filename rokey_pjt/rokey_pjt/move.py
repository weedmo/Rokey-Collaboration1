# domino movel controller @20241104

import rclpy
import DR_init
import time
from geometry_msgs.msg import PoseArray
from tf_transformations import euler_from_quaternion
import math

# robot config
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VEL, FIXED_ACC, DOWN_ACC = 100, 100, 30
FIXED_Z = 150.0
DOWN_Z = 100.0
FIXED_RX = 41.76
FIXED_RY = -180.0
ON, OFF = 1, 0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movel, 
            movej,
            wait,
            set_digital_output,
            get_tool,
            get_tcp,
            task_compliance_ctrl,
            set_desired_force,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            check_force_condition,
            DR_BASE,
            release_force,
            release_compliance_ctrl,
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing Doosan API: {e}")
        return

    pose_received = False
    pose_array_data = []

    def pose_callback(msg):
        nonlocal pose_received, pose_array_data
        pose_array_data = msg.poses
        pose_received = True
        node.get_logger().info(f"Received {len(pose_array_data)} poses from topic.")

    sub = node.create_subscription(
        PoseArray,
        "/domino_pose_array", 
        pose_callback,
        10
    )
    def grip():
        set_digital_output(1, OFF)
        wait(1.0)
        
    def release():
        set_digital_output(1, ON)
        wait(1.0)

    # === 픽앤플레이스 ===
    def pick(pose):
        movel(pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)

        down_pose = list(pose)
        down_pose[2] -= DOWN_Z
        movel(down_pose, vel=VEL, acc=DOWN_ACC, ref=DR_BASE)
        grip()

        movel(pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)
        
    def place(pose):
        movel(pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)

        go_and_down_pose = list(pose)
        go_and_down_pose[2] -= DOWN_Z
        movel(go_and_down_pose, vel=VEL, acc=DOWN_ACC, ref=DR_BASE)
        task_compliance_ctrl()
        time.sleep(0.1)
        set_desired_force(
            fd=[0, 0, -15, 0, 0, 0],
            dir=[0, 0, 1, 0, 0, 0],
            mod=DR_FC_MOD_REL
        )
        time.sleep(0.1)

        # 힘 조건 만족할 때까지 대기
        start = time.time()                                       
        while not check_force_condition(DR_AXIS_Z, max=10) and time.time() - start < 5.0:
            pass

        # 힘 제어 해제
        release_force()
        time.sleep(0.1)
        release_compliance_ctrl()
        release()

        movel(pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)
    
    def save_pose(pose_array_data):
        # 기준 pose (0번)
        p0 = pose_array_data[0]
        x0 = p0.position.x * 10 + 200
        y0 = p0.position.y * 10 - 225

        # 다음 pose (1번)
        p1 = pose_array_data[1]
        x1 = p1.position.x * 10 + 200
        y1 = p1.position.y * 10 - 225

        # 방향 벡터 계산
        dx = x1 - x0
        dy = y1 - y0

        # yaw 계산 (라디안 단위)
        yaw = math.atan2(dy, dx)

        # 반대 방향으로 30mm 이동
        yaw_reverse = yaw + math.pi
        x_offset = x0 + 30.0 * math.cos(yaw_reverse)
        y_offset = y0 + 30.0 * math.sin(yaw_reverse)

        # z 및 각도 설정
        z = p0.position.z * 10
        qx, qy, qz, qw = p0.orientation.x, p0.orientation.y, p0.orientation.z, p0.orientation.w
        _, _, yaw0 = euler_from_quaternion([qx, qy, qz, qw])
        rz0 = yaw0 * 180.0 / math.pi

        # 기존 포즈
        original_pose = posx(x0, y0, FIXED_Z - DOWN_Z, FIXED_RX, FIXED_RY, rz0)
        reverse_pose = posx(x_offset, y_offset, FIXED_Z, FIXED_RX, FIXED_RY, rz0)

        # --- ✅ 마지막 포즈 기준 정방향으로 50mm 이동 ---
        p_last = pose_array_data[-1]
        x_last = p_last.position.x * 10 + 200
        y_last = p_last.position.y * 10 - 225

        if len(pose_array_data) >= 2:
            p_before_last = pose_array_data[-2]
            dx_last = (p_last.position.x - p_before_last.position.x) * 10
            dy_last = (p_last.position.y - p_before_last.position.y) * 10
        else:
            dx_last, dy_last = 1.0, 0.0  # fallback (정방향이 없을 경우)

        yaw_last = math.atan2(dy_last, dx_last)

        # 정방향 50mm 이동
        x_forward = x_last + 50.0 * math.cos(yaw_last)
        y_forward = y_last + 50.0 * math.sin(yaw_last)

        q = [p_last.orientation.x, p_last.orientation.y, p_last.orientation.z, p_last.orientation.w]
        _, _, yaw_last_angle = euler_from_quaternion(q)
        rz_last = yaw_last_angle * 180.0 / math.pi

        forward_pose = posx(x_forward, y_forward, FIXED_Z, FIXED_RX, FIXED_RY, rz_last)

        # --- ✅ 중간 좌표 계산 (last ↔ forward) ---
        x_mid = (x_last + x_forward) / 2
        y_mid = (y_last + y_forward) / 2
        mid_pose = posx(x_mid, y_mid, FIXED_Z, FIXED_RX, FIXED_RY, rz_last)

        # 로그 출력
        print(f"📍 원래 포즈: {original_pose}")
        print(f"↩️ 반대 방향 30mm 이동: {reverse_pose}")
        print(f"➡️ 정방향 50mm 이동: {forward_pose}")
        print(f"🔸 중간 포즈 (last~forward): {mid_pose}")

        return original_pose, reverse_pose, forward_pose, mid_pose
    

        # 수신될 때까지 기다리기
    while rclpy.ok():
        tool_name = get_tool()
        tcp_name = get_tcp()

        print(f"Tool: {tool_name}, TCP: {tcp_name}")
        if tool_name == "" or tcp_name == "":
            rclpy.shutdown()
            return
        
            # pose가 들어올 때까지 기다림
        while not pose_received and rclpy.ok():
            rclpy.spin_once(node)
            time.sleep(0.1)
        release()
        
        home = posj(0, 0, 90.0, 0, 90, 0)
        movej(home, vel=VEL, acc=FIXED_ACC)
        pick_place = posx(180.35, 271.95, FIXED_Z, 41.76, -180.0, -45.47)
        original_pose, reverse_pose, forward_pose, mid_pose = save_pose(pose_array_data)
        horizontal_pick_place = posx(492.66, 258.77, 165.21, 54.69, -179.53, 57.09)
        
        # 포즈 하나씩 movel 수행
        print(pose_array_data)
        for i, pose in enumerate(pose_array_data):
            x = pose.position.x * 10 + 200
            y = pose.position.y * 10 - 225
            z = pose.position.z * 10 

            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w

            # 오직 yaw만 사용, roll/pitch 무시
            _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
            rz = yaw * 180.0 / 3.141592

            p = posx(x, y, FIXED_Z, FIXED_RX, FIXED_RY, rz)

            print(f"[{i+1}/{len(pose_array_data)}] movel to: {p}")
            
            pick(pick_place)
            place(p)
        
        # 마지막 피날레 
        pick(pick_place)
        place(forward_pose)
        pick(horizontal_pick_place)
        place(mid_pose)
        pick(pick_place)
        mid_pose[2] += 100
        place(mid_pose)
        
        # 쓰러트리기
        movel(reverse_pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)
        grip()
        reverse_pose[2] -= DOWN_Z
        movel(reverse_pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)
        movel(original_pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)
        movej(home, vel=VEL, acc=FIXED_ACC)
        
        rclpy.shutdown()


if __name__ == "__main__":
    main()


