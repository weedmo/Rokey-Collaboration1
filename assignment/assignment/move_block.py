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
    node = rclpy.create_node("move_block", namespace=ROBOT_ID)
    
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
            get_posx,
            
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
    
    home = posj(0, 0, 90.0, 0, 90, 0)
    pos = posx(249.73, 93.68, 104.84, 30.25, 179.95, 30.24)
    pos2 = posx(359.73, 93.68, 104.84, 30.25, 179.95, 30.24)
    pos3 = posx(249.73, -16.32, 104.84, 30.25, 179.95, 30.24)
    pos4 = posx(359.73, -16.32, 104.84, 30.25, 179.95, 30.24)

    next_pos = posx(551.01, 88.58, 26.82, 29.8, 179.95, 29.79)
    next_pos2 = posx(661.01, 88.58, 26.82, 29.8, 179.95, 29.79)
    next_pos3 = posx(551.01, -21.42, 26.82, 29.8, 179.95, 29.79)
    next_pos4 = posx(661.01, -21.42, 26.82, 29.8, 179.95, 29.79)
    
    # ====================
    # grip
    ON, OFF = 1, 0

    def grip():
        set_digital_output(1, OFF)
        wait(1.0)

    def release():
        set_digital_output(1, ON)
        wait(1.0)
    
    def check(pose):
        movel(pose, vel=VEL, acc=ACC, ref=DR_BASE)
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
        z = list(get_posx())[2]
        
    def pick(pose):
        movel(pose, vel=VEL, acc=ACC, ref=DR_BASE)

        down_pose = list(pose)
        down_pose[2] -= 80
        movel(down_pose, vel=VEL, acc=ACC, ref=DR_BASE)
        grip()

        movel(pose, vel=VEL, acc=ACC, ref=DR_BASE)


    def place(pose, use_force=False):
        movel(pose, vel=VEL, acc=ACC, ref=DR_BASE)

        go_and_down_pose = list(pose)
        go_and_down_pose[2] -= 80.0
        movel(go_and_down_pose, vel=VEL, acc=ACC, ref=DR_BASE)
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

            # 힘 제어 해제
            release_force()
            time.sleep(0.1)
            release_compliance_ctrl()

        release()
        wait(1.0)
        movel(pose, vel=VEL, acc=ACC, ref=DR_BASE)

    # ====================    

    while rclpy.ok():
        
        movej(home, vel=VEL, acc=ACC)
        release()

        break


if __name__ == "__main__":
    main()
