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
            get_current_posx,
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
    pos = posx(488.54, 38.19, 8.08, 168.25, 180.0, 77.3)
    up = posj(16.39, 22.68, 90.57, -13.54, 71.09, 107.35)
    # ====================
    # grip
    ON, OFF = 1, 0

    def grip():
        set_digital_output(1, OFF)
        wait(1.0)

    def release():
        set_digital_output(1, ON)
        wait(1.0)
        
    def pick(pose):
        release()
        movel(pose, vel=VEL_DOWN, acc=ACC_DOWN, ref=DR_BASE)
        grip()
        
        task_compliance_ctrl()
        time.sleep(0.1)
        set_desired_force(
            fd=[0, 0, 15, 0, 0, 0],   # 위로 올라가면서 아래 방향 힘 제어
            dir=[0, 0, 1, 0, 0, 0],
            mod=DR_FC_MOD_REL
        )
        time.sleep(0.1)

        movej(up, vel=VEL_UP, acc=ACC_UP)

    # 4. 힘 제어 종료
    release_force()
    release_compliance_ctrl()


    def place(pose):
        # Step 1: 원래 위치로 이동
        pose[2] += 20
        movel(pose, vel=VEL_DOWN, acc=ACC_DOWN, ref=DR_BASE)
        
        # 1. 힘 제어 활성화
        task_compliance_ctrl()
        time.sleep(0.1)

        # 2. 아래 방향 힘 적용 (로봇 기준: z축 음수 방향)
        set_desired_force(
            fd=[0, 0, -15, 0, 0, 0], 
            dir=[0, 0, 1, 0, 0, 0], 
            mod=DR_FC_MOD_REL
        )
        time.sleep(0.1)

        # 4. 접촉 힘 감지 시까지 대기
        while not check_force_condition(DR_AXIS_Z, max=10):
            pass

        # 5. 힘 제어 해제
        release_force()
        time.sleep(0.1)
        release_compliance_ctrl()

        release()
        wait(1.0)

        # Step 7: 원래 위치로 복귀
        movel(pose, vel=VEL_DOWN, acc=ACC_DOWN, ref=DR_BASE)
    def push(pose):
        # Step 1: 원래 위치로 이동 (접촉 전 준비)
        pose[2] += 40
        movel(pose, vel=VEL_DOWN, acc=ACC_DOWN, ref=DR_BASE)
        grip()

        # Step 2: 힘 제어 활성화
        task_compliance_ctrl()
        time.sleep(0.1)

        # Step 3: 아래 방향 힘 설정 (예: -15N)
        set_desired_force(
            fd=[0, 0, -30, 0, 0, 0], 
            dir=[0, 0, 1, 0, 0, 0], 
            mod=DR_FC_MOD_REL
        )
        time.sleep(0.1)

        # Step 4: 힘 조건 만족 or 타임아웃
        timeout = 10.0  # 최대 5초 대기
        start_time = time.time()
        while not check_force_condition(DR_AXIS_Z, max=30):
            if time.time() - start_time > timeout:
                print("⚠️ 힘 조건 미달로 종료 (timeout)")
                break

        # Step 5: 힘 제어 해제
        release_force()
        time.sleep(0.1)
        release_compliance_ctrl()

        # Step 6: 그리퍼 열기 & 대기
        release()
        wait(1.0)

        # Step 7: 원래 위치로 복귀
        movel(pose, vel=VEL_DOWN, acc=ACC_DOWN, ref=DR_BASE)

        
    while rclpy.ok():
        tool_name = get_tool()
        tcp_name = get_tcp()

        print(f"Tool: {tool_name}, TCP: {tcp_name}")
        if tool_name == "" or tcp_name == "":
            rclpy.shutdown()
            return

        movej(home, vel=VEL_DOWN, acc=ACC_DOWN)
        pick(pos)
        movej(home, vel=VEL_DOWN, acc=ACC_DOWN)
        place(pos)
        push(pos)
        movej(home, vel=VEL_DOWN, acc=ACC_DOWN)
        break  

if __name__ == "__main__":
    main()
