import rclpy
import DR_init
import time
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32MultiArray
from tf_transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_euler
import math
import numpy as np

# robot config
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VEL, ACC, DOWN_ACC = 100, 100, 30
FIXED_Z = 150.0
DOWN_Z = 100.0
FIXED_RX = 41.76
FIXED_RY = 180.0
ON, OFF = 1, 0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("real_move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movel, movej, wait, set_digital_output, get_tool, get_tcp,
            task_compliance_ctrl, set_desired_force, DR_FC_MOD_REL,
            DR_AXIS_Z, check_force_condition, DR_BASE,
            release_force, release_compliance_ctrl,
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        node.get_logger().error(f"Error importing Doosan API: {e}")
        return

    pose_received = False
    index_received = False
    stair_index, stair_height = 0, 0
    pose_array_data = []
    converted_pose_list = []
    qw_list = []
    home = posj(0, 0, 90.0, 0, 90, 0)
    pick_place = posx(170.20, 271.95, FIXED_Z, 41.76, FIXED_RY, -45.47)
    horizontal_pick_place = posx(411.95, 271.49, 110.89, 42.93, FIXED_RY, -43.75)
    put_horizontal = posx(507.49, 272.86, 220.37, 16.43, FIXED_RY, -70.85)

    def pose_callback(msg):
        nonlocal pose_received, pose_array_data, converted_pose_list
        pose_array_data = msg.poses
        converted_pose_list = []
        pose_received = True
        length = len(pose_array_data)
        
        for pose in pose_array_data[:length-2]:
            x = pose.position.x * 10 + 200
            y = pose.position.y * 10 - 225
            z = pose.position.z * 10
            qx, qy, qz, qw = pose.orientation.x* 10, pose.orientation.y * 10, pose.orientation.z * 10, pose.orientation.w * 10
            rx, ry, rz = euler_from_quaternion([qx, qy, qz, qw])
            rx_deg = math.degrees(rx)
            rz_deg = math.degrees(rz)

            converted_pose_list.append([x, y, FIXED_Z, rx_deg, FIXED_RY, rz_deg])
            qw_list.append(qw)
        node.get_logger().info(f"📥 {len(converted_pose_list)}개의 pose를 변환 완료했습니다.")


    def index_callback(msg):
        nonlocal stair_index, stair_height, index_received
        if len(msg.data) >= 2:
            stair_index = msg.data[0]
            stair_height = msg.data[1]
            index_received = True
            node.get_logger().info(f"📩 index: {stair_index}, height: {stair_height}")
        else:
            node.get_logger().warn("❗ index_callback: data 길이가 2보다 작습니다.")

    node.create_subscription(PoseArray, "/domino_pose_array", pose_callback, 10)
    node.create_subscription(Int32MultiArray, "/domino_pose_array_with_index", index_callback, 10)

    def grip():
        set_digital_output(1, OFF)
        wait(1.0)

    def release():
        set_digital_output(1, ON)
        wait(1.0)

    def pick(pose):
        movel(pose, vel=VEL, acc=ACC, ref=DR_BASE)
        down_pose = list(pose)
        down_pose[2] -= DOWN_Z
        movel(down_pose, vel=VEL, acc=DOWN_ACC, ref=DR_BASE)
        grip()
        movel(pose, vel=VEL, acc=ACC, ref=DR_BASE)

    def place(pose):
        movel(pose, vel=VEL, acc=ACC, ref=DR_BASE)
        go_and_down_pose = list(pose)
        go_and_down_pose[2] -= DOWN_Z
        movel(go_and_down_pose, vel=VEL, acc=DOWN_ACC, ref=DR_BASE)
        task_compliance_ctrl()
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.1)
        while not check_force_condition(DR_AXIS_Z, max=10):
            pass
        release_force()
        release_compliance_ctrl()
        release()
        movel(pose, vel=VEL, acc=ACC, ref=DR_BASE)

    def get_forward_pose(index, dist, alpha=0.2):
        """
        주어진 index pose 기준으로, dist만큼 로컬 정방향으로 이동한 pose 반환  
        - 벡터 기반 offset은 기존 방향 유지  
        - 쿼터니언 기반 offset은 yaw + 270도로 반전 + 수직 이동 방향 적용
        """
        n = len(converted_pose_list)
        if index < 0 or index >= n:
            raise IndexError("Index out of range")

        x0, y0, z0, rx_deg, ry_deg, rz_deg = converted_pose_list[index]

        # --- 벡터 방향 기반 offset (그대로 유지) ---
        if index > 0:
            x_prev, y_prev, *_ = converted_pose_list[index - 1]
            dx = x0 - x_prev
            dy = y0 - y_prev
            yaw_vec = math.atan2(dy, dx)
        else:
            yaw_vec = math.radians(rz_deg)

        offset_vec = np.array([
            dist * math.cos(yaw_vec),
            dist * math.sin(yaw_vec),
            0.0
        ])

        # --- 쿼터니언 기반 offset (yaw + 270도 회전) ---
        rx_rad = math.radians(rx_deg)
        ry_rad = math.radians(ry_deg)
        rz_rad = math.radians((rz_deg + 270.0) % 360.0)  # 반전 + 수직 방향

        qx, qy, qz, qw = quaternion_from_euler(rx_rad, ry_rad, rz_rad)
        T = quaternion_matrix([qx, qy, qz, qw])
        local_offset = np.array([dist, 0.0, 0.0, 1.0])
        world_offset = T @ local_offset
        offset_quat = world_offset[:3]

        # --- 혼합 offset ---
        offset = alpha * offset_quat + (1.0 - alpha) * offset_vec

        x_new = x0 + offset[0]
        y_new = y0 + offset[1]
        z_new = z0 + offset[2]

        return [x_new, y_new, z_new, rx_deg, ry_deg, rz_deg]

    def update_all_converted_poses_with_yaw_based_offset(converted_pose_list, dist=0.0, alpha=0.2):

        updated_list = []
        n = len(converted_pose_list)

        for i in range(n):
            try:
                x, y, z, rx, ry, rz = get_forward_pose(i, dist, alpha)
                updated_list.append([x, y, z, rx, ry, rz])
            except Exception as e:
                print(f"❌ pose index {i} 업데이트 실패: {e}")
                updated_list.append(converted_pose_list[i])  # 실패 시 기존 값 유지

        converted_pose_list = updated_list
        print(f"✅ 전체 {n}개의 pose가 yaw 방향 기준으로 업데이트되었습니다.")


    def apply_stair_pose_from_stack_logic(stair_index, stair_height, z_offset_per_layer=0.0):
        """
        stack_stairs()와 동일한 거리 로직을 사용하되,
        도미노 위치를 사전에 계산하여 converted_pose_list에 반영함.
        z 오프셋(z_offset_per_layer)을 각 층에 추가로 적용할 수 있음.
        """
        if stair_height <= 1:
            return

        if not (0 <= stair_index < len(converted_pose_list)):
            print("❗ 유효하지 않은 stair_index입니다.")
            return

        temp_pose_map = {}

        for i in range(stair_height, 0, -1):
            z_layer = FIXED_Z + i * 15.0 + z_offset_per_layer
            if i % 2 == 1:
                distances = [0.0]
            else:
                distances = [-i * 32.0, i * 32.0]  # 거리 넓게 적용

            for dist in distances:
                try:
                    x, y, _, rx, ry, rz = get_forward_pose(stair_index, dist,0.2)
                    temp_pose_map[(i, dist)] = [x, y, z_layer, rx, ry, rz]
                except Exception as e:
                    node.get_logger().error(f"❌ 계단 pose 계산 실패: height={i}, dist={dist}, {e}")

        # converted_pose_list에 덮어쓰기
        sorted_keys = sorted(temp_pose_map.keys(), key=lambda k: (k[0], k[1]))
        for i, key in enumerate(sorted_keys):
            converted_pose_list[i] = temp_pose_map[key]
            

    def stack_stairs(stair_index, stair_height):
        node.get_logger().info(f"🏗 계단 쌓기 시작: index={stair_index}, height={stair_height}")
        for i in range(stair_height, 0, -1):
            z = FIXED_Z + i * 15.0
            if i % 2 == 1:
                distances = [0]  # 홀수 층은 중앙 1개만
            else:
                distances = [-i * 22.0, i * 22.0]  # 짝수 층은 양쪽 2개
            for dist in distances:
                try:
                    x, y, z, rx, ry, rz = get_forward_pose(stair_index, dist,0.2)
                    pose = posx(x, y, z, rx, ry, rz)
                    pose[2] -= 30
                    pick(horizontal_pick_place)
                    place(pose)
                    pick(pick_place)
                    movel(put_horizontal, vel=VEL, acc=ACC, ref=DR_BASE)
                    release()
                    print(f"📦 height={i}, dist={dist}, pose={pose}")
                except Exception as e:
                    node.get_logger().error(f"❌ 오류: {e}")
                
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

        # 반환할 pose들
        original_pose = posx(x0, y0, FIXED_Z - DOWN_Z, FIXED_RX, FIXED_RY, rz0)
        reverse_pose = posx(x_offset, y_offset, FIXED_Z, FIXED_RX, FIXED_RY, rz0)

        print(f"📍 원래 포즈: {original_pose}")
        print(f"↩️ 반대 방향 30mm 이동: {reverse_pose}")

        return original_pose, reverse_pose

    while rclpy.ok():
        tool, tcp = get_tool(), get_tcp()
        print(f"Tool: {tool}, TCP: {tcp}")
        if not tool or not tcp:
            node.get_logger().warn("❗ Tool/TCP 미설정")
            rclpy.shutdown()
            return

        while not pose_received and rclpy.ok():
            #print("📡 pose input 대기 중...")
            rclpy.spin_once(node, timeout_sec=0.1)

        while not index_received and rclpy.ok():
            #print("📡 index input 대기 중...")
            rclpy.spin_once(node, timeout_sec=0.1)
            
        release()
        movej(home, vel=VEL, acc=ACC)
        stack_stairs(stair_index, stair_height)
        original_pose, reverse_pose = save_pose(pose_array_data)
        
        # domino
        if stair_height > 1:
            apply_stair_pose_from_stack_logic(stair_index, stair_height, z_offset_per_layer=20.0)  
        else: 
            update_all_converted_poses_with_yaw_based_offset(converted_pose_list, dist=0.0, alpha=0.2)
            
        for i, pose in enumerate(converted_pose_list):
            p = posx(pose)
            print(f"[{i+1}/{len(converted_pose_list)}] movel to: {p}")
            pick(pick_place)
            place(p)

        movej(home, vel=VEL, acc=ACC)

        # ✅ 피날레: 마지막 포즈 기준 forward, mid, reverse 계산
        last_index = len(converted_pose_list) - 1

        # 정방향 50mm
        forward_pose_vals = get_forward_pose(last_index, 70.0, 0.2)
        forward_pose = posx(forward_pose_vals)

        # 중간 지점: 마지막 포즈와 forward 사이 평균
        x_last, y_last, z_last, rx, ry, rz = converted_pose_list[last_index]
        x_fwd, y_fwd, z_fwd, *_ = forward_pose_vals
        x_mid = (x_last + x_fwd) / 2
        y_mid = (y_last + y_fwd) / 2
        z_mid = (z_last + z_fwd) / 2
        mid_pose = posx(x_mid, y_mid, z_mid + 50, rx, ry, rz)  # 높이 올림

        # ✅ 피날레 동작 수행
        pick(pick_place)
        place(forward_pose)

        pick(horizontal_pick_place)
        place(mid_pose)

        pick(pick_place)
        mid_pose[2] += 50
        place(mid_pose)

        # 쓰러트리기
        movel(reverse_pose, vel=VEL, acc=ACC, ref=DR_BASE)
        grip()
        reverse_pose[2] -= DOWN_Z
        movel(reverse_pose, vel=VEL, acc=ACC, ref=DR_BASE)
        movel(original_pose, vel=VEL, acc=ACC, ref=DR_BASE)
        movej(home, vel=VEL, acc=ACC)

        rclpy.shutdown()


if __name__ == "__main__":
    main()
