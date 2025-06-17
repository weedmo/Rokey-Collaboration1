# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VEL_UP, ACC_UP = 100, 100
VEL_DOWN, ACC_DOWN = 60, 60 

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move_block", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            # grip
            set_digital_output,
            get_tool,
            get_tcp,
            wait,
            movej,

        )

        from DR_common2 import posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    home = posj(0, 0, 90.0, 0, 90, 0)
    
    # ====================
    # grip
    ON, OFF = 1, 0

    def grip():
        set_digital_output(1, OFF)
        wait(1.0)

    def release():
        set_digital_output(1, ON)
        wait(1.0)

    while rclpy.ok():
        tool_name = get_tool()
        tcp_name = get_tcp()

        print(f"Tool: {tool_name}, TCP: {tcp_name}")
        if tool_name == "" or tcp_name == "":
            rclpy.shutdown()
            return
        release()
        movej(home, vel=VEL_DOWN, acc=ACC_DOWN)
        break  # ← 반복을 원한다면 이 줄을 제거

if __name__ == "__main__":
    main()
