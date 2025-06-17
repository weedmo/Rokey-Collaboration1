# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VEL, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move_gear", namespace=ROBOT_ID)
    
    # publisher -> 조인트 값 계속 쏘는 퍼블리셔
    # subscription -> 시작 입력을 받으면 동작하는 퍼블리셔

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            # grip
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            get_tool,
            get_tcp,
            
            # move
            wait,
            set_tool,
            # set_tcp,
            movej,
            movel,
            
            # force control
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            DR_TOOL,
            release_force,
            
            move_periodic,
            amove_periodic,
            
            amovel,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    JReady = [0, 0, 90, 0, 90, 0] # home 위치
    
    # position
    Global_origin_1 = posx(274.45, -8.19, 91.79, 122.19, 179.98, 122.16) # 가운데 기어
    Global_origin_2 = posx(242.59, 45.09, 91.79, 121.12, 179.98, 121.09)
    Global_origin_3 = posx(248.28, -59.63, 91.79, 119.76, 179.98, 119.72)
    Global_origin_4 = posx(334.53, -4.64, 91.79, 120.59, 179.98, 120.56)

    Global_dst_1 = posx(574.37, -10.65, 91.79, 30.83, 179.96, 30.79) # 가운데 기어
    Global_dst_2 = posx(601.13, 43.15, 91.79, 53.35, 179.97, 53.32)
    Global_dst_3 = posx(607.6, -62.81, 91.79, 32.96, 179.96, 32.93)
    Global_dst_4 = posx(513.1, -15.37, 91.79, 47.71, 179.96, 47.68)
    
    down_pose_little = posx(0.0, 0.0, 25.0, 0.0, 0.0, 0.0) # 조금만 내려가는거, 기어 1에서 사용
    
    # ====================
    # grip
    ON, OFF = 1, 0

    def grip():
        set_digital_output(1, OFF)
        wait(1.0)

    def release():
        set_digital_output(1, ON)
        wait(1.0)
        
    def pick(pose_lists):
        movel(pose_lists, vel=VEL, acc=ACC, ref=DR_BASE)

        down_pose = list(pose_lists)
        down_pose[2] -= 50
        movel(down_pose, vel=VEL, acc=ACC, ref=DR_BASE)
        grip()

        movel(pose_lists, vel=VEL, acc=ACC, ref=DR_BASE)


    def place(pose_lists, use_force=False):
        go_pose = list(pose_lists)
        movel(go_pose, vel=VEL, acc=ACC, ref=DR_BASE)

        # go_and_down_pose = list(go_pose)
        # go_and_down_pose[2] -= 50.0
        # movel(go_and_down_pose, vel=VEL, acc=ACC, ref=DR_BASE)
        # release()

        # movel(go_pose, vel=VEL, acc=ACC, ref=DR_BASE)

        if use_force:
            # 힘 제어 활성화
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

            amove_periodic(
                amp=[0, 0, 0, 0, 0, 15],
                period=1.0,
                atime=0.02,
                repeat=3,
                ref=DR_BASE
            )
            wait(3.0)

            # 힘 제어 해제
            release_force()
            time.sleep(0.1)
            release_compliance_ctrl()

        release()
        wait(1.0)
        movel(go_pose, vel=VEL, acc=ACC, ref=DR_BASE)

    # ====================    

    while rclpy.ok():
        tool_name = get_tool()
        tcp_name = get_tcp()

        print(f"Tool: {tool_name}, TCP: {tcp_name}")
        if tool_name == "" or tcp_name == "":
            rclpy.shutdown()
            return

        movej(JReady, vel=VEL, acc=ACC)
        release()
        
        pick(Global_origin_2)
        place(Global_dst_2, use_force=True)

        pick(Global_origin_3)
        place(Global_dst_3, use_force=True)

        pick(Global_origin_4)
        place(Global_dst_4, use_force=True)

        pick(Global_origin_1) 
        place(Global_dst_1, use_force=True)

        break


if __name__ == "__main__":
    main()
