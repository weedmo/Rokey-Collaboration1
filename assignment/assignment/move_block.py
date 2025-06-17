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
    pos = posx(252.87, 95.18, 74.84, 30.25, 179.95, 30.24)
    next_pos = posx(551.11, 91.0, 116.82, 29.8, 179.95, 29.79)
    
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
            mod=DR_FC_MOD_REL)
        time.sleep(0.1)
        while not check_force_condition(DR_AXIS_Z, max=10):
            pass
        pos = get_current_posx()
        contact_z = pos[0][2]
        release_force()
        time.sleep(0.1)
        release_compliance_ctrl()

        print(f"📌 접촉 z 위치: {contact_z}")

        if contact_z is not None:
            retreat_pose = list(pose)
            retreat_pose[2] = contact_z + 80
            movel(retreat_pose, vel=VEL, acc=ACC, ref=DR_BASE)

        return contact_z


        
    def pick(pose):
        release()
        movel(pose, vel=VEL, acc=ACC, ref=DR_BASE)
        grip()
        up_pose = list(pose)
        up_pose[2] += 80
        movel(up_pose, vel=VEL, acc=ACC, ref=DR_BASE)


    def place(pose, use_force=False):
        # Step 1: 원래 위치로 이동
        movel(pose, vel=VEL, acc=ACC, ref=DR_BASE)

        
        # Step 2: 약간만 내려가기
        pre_contact_pose = list(pose)
        pre_contact_pose[2] -= 60.0  # 미리 살짝 내려감
        movel(pre_contact_pose, vel=VEL, acc=ACC, ref=DR_BASE)
        task_compliance_ctrl()
        time.sleep(0.1)
        set_desired_force(
            fd=[0, 0, -15, 0, 0, 0], 
            dir=[0, 0, 1, 0, 0, 0], 
            mod=DR_FC_MOD_REL)
        time.sleep(0.1)
        while not check_force_condition(DR_AXIS_Z, max=10):
            pass
        release_force()
        time.sleep(0.1)
        release_compliance_ctrl()

        release()
        wait(1.0)

        # Step 7: 원래 위치로 복귀
        movel(pose, vel=VEL, acc=ACC, ref=DR_BASE)

    def is_in_range(z, low, high):
        return low <= z < high

    def get_column_by_z(z):
        if z >= 64:
            return 0
        elif z >= 54:
            return 1
        elif z >= 44:
            return 2
        else:
            return -1  # 예외 처리용

    def full_pick_and_place(start_pos, base_place_pos):
        pos = start_pos.copy()
        block_count = 0
        column_counts = [0, 0, 0]  # 열(col) 별로 몇 개 놓았는지 추적

        for x in range(3):
            for y in range(3):
                print(f"\n🔄 {block_count+1}번째 블록 처리 중...")
                grip()

                z = check(pos)  # 힘 감지로 접촉 z 얻기
                if z is None:
                    print("❌ z 감지 실패, 스킵")
                    continue

                # 픽 위치 수정
                pick_pos = pos.copy()
                pick_pos[2] = z - 10
                pick(pick_pos)

                # place 위치 결정
                col = get_column_by_z(z)
                if col == -1 or column_counts[col] >= 3:
                    print(f"⚠ 유효하지 않은 col={col} 또는 이미 3개 배치됨, 스킵")
                    continue

                row = column_counts[col]
                place_pos = base_place_pos.copy()
                place_pos[0] += col * 51.5  # 열 이동 (X)
                place_pos[1] -= row * 51.5  # 행 이동 (Y, 아래로 정렬)
                place_pos[2] = z + 80     # 높이

                place(place_pos)
                column_counts[col] += 1
                block_count += 1

                # 다음 픽 위치로 이동
                pos[1] -= 51.5  # y축 이동

            # y축 다 했으면 x축 이동하고 y축 복원
            pos[0] += 51.5
            pos[1] = start_pos[1]

        movej(home, vel=VEL, acc=ACC)
        print("✅ 모든 작업 완료")

    while rclpy.ok():
        tool_name = get_tool()
        tcp_name = get_tcp()

        print(f"Tool: {tool_name}, TCP: {tcp_name}")
        if tool_name == "" or tcp_name == "":
            rclpy.shutdown()
            return

        movej(home, vel=VEL, acc=ACC)

        # pick and place 수행
        full_pick_and_place(pos, next_pos)

        # 모든 작업 완료 후 종료 또는 반복
        print("🔁 반복을 원하면 [Ctrl+C]를 누르지 말고 계속 진행됩니다.")
        time.sleep(3)  # 혹은 다음 루프 전 대기 시간
        break  # ← 반복을 원한다면 이 줄을 제거

if __name__ == "__main__":
    main()
