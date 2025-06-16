# domino movel controller @20241104

import rclpy
import DR_init
import time
from geometry_msgs.msg import PoseArray
from tf_transformations import euler_from_quaternion

# robot config
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VEL, FIXED_ACC, DOWN_ACC = 60, 60, 30
FIXED_Z = 150.0
DOWN_Z = 150.0
FIXED_RX = 180.0
FIXED_RY = 0.0
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
            DR_TOOL,
            
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

    # === 픽앤플레이스 ===
    def pick(pick_pose):
        movel(pick_pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)

        down_pose = list(pick_pose)
        down_pose[2] -= 100.0
        movel(down_pose, vel=VEL, acc=DOWN_ACC, ref=DR_BASE)
        grip()

        movel(pick_pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)
        
    def place(place_pose):
        movel(place_pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)

        go_and_down_pose = list(place_pose)
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

        # 힘 조건 만족할 때까지 대기
        start = time.time()
        while not check_force_condition(DR_AXIS_Z, max=10) and time.time() - start < 5.0:
            pass

        # 힘 제어 해제
        release_force()
        time.sleep(0.1)
        release_compliance_ctrl()
        release()

        movel(place_pose, vel=VEL, acc=FIXED_ACC, ref=DR_BASE)
        
        
        # 수신될 때까지 기다리기
    while rclpy.ok():
        tool_name = get_tool()
        tcp_name = get_tcp()

        print(f"Tool: {tool_name}, TCP: {tcp_name}")
        if tool_name == "" or tcp_name == "":
            rclpy.shutdown()
            return
        
    
        release()
        home = posj(0, 0, 90.0, 0, 90, 0)
        movej(home, vel=VEL, acc=FIXED_ACC)
        pick_place = posx(180.35, 271.95, FIXED_Z, 41.76, -180.0, -45.47)

        p = posx(180.35, 271.95, FIXED_Z, FIXED_RX, FIXED_RY, -45.47)
        
        pick(pick_place)
        place(p)


        rclpy.shutdown()


if __name__ == "__main__":
    main()
