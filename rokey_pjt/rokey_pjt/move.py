import rclpy
import DR_init
import time
from geometry_msgs.msg import PoseArray
from tf_transformations import euler_from_quaternion
from DR_common2 import posx, posj

# robot config
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VEL, FIXED_ACC, DOWN_ACC = 60, 60, 30
FIXED_Z = 150.0
DOWN_Z = 150.0
FIXED_RX = 41.76
FIXED_RY = -180.0
ON, OFF = 1, 0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ✅ 함수 정의: 마지막 두 도미노 위에 가로 도미노 1개 놓기
def place_horizontal_domino_on_last_two(pose_array, horizontal_pick_pose):
    """마지막 두 도미노 위 중심에 90도 회전된 도미노를 하나 놓는다."""
    pose1 = pose_array[-2]
    pose2 = pose_array[-1]

    # 중심 좌표 계산
    x_center = (pose1.position.x + pose2.position.x) / 2 * 10 + 200
    y_center = (pose1.position.y + pose2.position.y) / 2 * 10 - 225

    # yaw 계산 + 90도 회전
    _, _, yaw1 = euler_from_quaternion([
        pose1.orientation.x, pose1.orientation.y,
        pose1.orientation.z, pose1.orientation.w
    ])
    _, _, yaw2 = euler_from_quaternion([
        pose2.orientation.x, pose2.orientation.y,
        pose2.orientation.z, pose2.orientation.w
    ])
    yaw_center = (yaw1 + yaw2) / 2 + (3.141592 / 2)
    rz_rotated = yaw_center * 180.0 / 3.141592

    # 높이는 사용자가 설정
    custom_z = FIXED_Z

    # 최종 포즈
    target_pose = posx(x_center, y_center, custom_z, FIXED_RX, FIXED_RY, rz_rotated)
    print(f"[가로 도미노] 위치: {target_pose}")
    
    pick(horizontal_pick_pose)   # <-- 사용자가 따로 입력한 좌표
    place(target_pose)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movel, movej, wait,
            set_digital_output, get_tool, get_tcp,
            task_compliance_ctrl, set_desired_force, DR_FC_MOD_REL,
            DR_AXIS_Z, check_force_condition,
            release_force, release_compliance_ctrl,
            DR_BASE
        )
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
        "/domino_pose_array_resampled",
        pose_callback,
        10
    )

    def grip():
        set_digital_output(1, OFF)
        wait(1.0)

    def release():
        set_digital_output(1, ON)
        wait(1.0)

    def pick(pose):
        movel(pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)
        down_pose = list(pose)
        down_pose[2] -= 100.0
        movel(down_pose, vel=VEL, acc=DOWN_ACC, ref=DR_BASE)
        grip()
        movel(pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)

    def place(pose):
        movel(pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)
        go_and_down_pose = list(pose)
        go_and_down_pose[2] -= 100.0
        movel(go_and_down_pose, vel=VEL, acc=DOWN_ACC, ref=DR_BASE)
        task_compliance_ctrl()
        time.sleep(0.1)
        set_desired_force(
            fd=[0, 0, -15, 0, 0, 0],
            dir=[0, 0, 1, 0, 0, 0],
            mod=DR_FC_MOD_REL
        )
        time.sleep(0.1)
        start = time.time()
        while not check_force_condition(DR_AXIS_Z, max=10) and time.time() - start < 5.0:
            pass
        release_force()
        time.sleep(0.1)
        release_compliance_ctrl()
        release()
        movel(pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)

    while rclpy.ok():
        tool_name = get_tool()
        tcp_name = get_tcp()

        if tool_name == "" or tcp_name == "":
            rclpy.shutdown()
            return

        while not pose_received and rclpy.ok():
            rclpy.spin_once(node)
            time.sleep(0.1)

        release()

        home = posj(0, 0, 90.0, 0, 90, 0)
        movej(home, vel=VEL, acc=FIXED_ACC)

        pick_place = posx(180.35, 271.95, FIXED_Z, 41.76, -180.0, -45.47)

        # ✅ 여기를 사용자가 바꿔주세요 (가로 도미노 pick 좌표)
        horizontal_pick_place = posx(492.66, 258.77, 65.21, 54.69, -179.53, 57.09)  # ← 수정 가능

        for i, pose in enumerate(pose_array_data):
            x = pose.position.x * 10 + 200
            y = pose.position.y * 10 - 225
            z = pose.position.z * 10

            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w

            _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
            rz = yaw * 180.0 / 3.141592

            p = posx(x, y, FIXED_Z, FIXED_RX, FIXED_RY, rz)
            print(f"[{i+1}/{len(pose_array_data)}] placing: {p}")
            pick(pick_place)
            place(p)

        # ✅ 세로 도미노 다 놓은 후 가로 도미노 한 개 놓기
        place_horizontal_domino_on_last_two(pose_array_data, horizontal_pick_place)

        rclpy.shutdown()

if __name__ == "__main__":
    main()
