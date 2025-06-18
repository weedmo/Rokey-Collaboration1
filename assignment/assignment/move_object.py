import sys
sys.path.append('/home/joonmo/rokey_doosan_ws/src/doosan-robot2/dsr_common2/imp')

import rclpy
import DR_init
import time
import yaml

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VEL, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move_object", namespace=ROBOT_ID)
    
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
    
   # ====================
    # grip
    ON, OFF = 1, 0

    def grip():
        set_digital_output(2, OFF)
        wait(1.0)

    def release():
        set_digital_output(2, ON)
        wait(1.0)
        # wait_digital_input(2)
        
    # ====================
    # YAML 파일 불러오기
    #yaml_path = os.path.join(os.path.dirname(__file__), 'config/object_positions.yaml')
    yaml_path = "/home/joonmo/rokey_doosan_ws/src/Rokey-Collaboration1/assignment/object.yaml"

    try:
        with open(yaml_path, 'r') as f:
            positions = yaml.safe_load(f)
    except Exception as e:
        print(f"YAML load error: {e}")
        return

    # ====================
    # 사용자 입력
    object_name = input("Enter object name (cup, block, lego): ").strip()
    target_box = input("Enter box name (box1, box2): ").strip()

    if object_name not in positions['pick'] or target_box not in positions['place']:
        print("Invalid input.")
        return

    pick_pose = posx(*positions['pick'][object_name])
    before_pose = posx(*positions['before_place'][target_box])
    place_pose = posx(*positions['place'][target_box])

    # ====================
    # 동작
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    
    # home = posj(0, 0, 90.0, 0, 90, 0)
    movej(home, vel=VEL, acc=ACC)
    release()

    print(f"Moving to pick {object_name}...")
    movel(pick_pose, vel=VEL, acc=ACC)
    time.sleep(0.1)
    grip()
    time.sleep(0.1)

    print(f"Moving to {target_box} via before_place...")
    movel(before_pose, vel=VEL, acc=ACC)
    time.sleep(0.1)

    print(f"Placing into {target_box}...")
    movel(place_pose, vel=VEL, acc=ACC)
    time.sleep(0.1)
    release()
    time.sleep(0.1)
    
    movel(before_pose, vel=VEL, acc=ACC)
    time.sleep(0.1)
    
    movej(home, vel=VEL, acc=ACC)

    print("Done.")
    rclpy.shutdown()