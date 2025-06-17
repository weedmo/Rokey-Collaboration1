# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)

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
    
    # ====================
    # grip
    ON, OFF = 1, 0
    
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        # wait_digital_input(2)

    def grip():
        # release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        # wait_digital_input(1)
    # ====================    
    
    while rclpy.ok():
        tool_name = get_tool()
        tcp_name = get_tcp()

        print(f"Tool: {tool_name}, TCP: {tcp_name}")
        if tool_name == "" or tcp_name == "":
            rclpy.shutdown()
            return
        
        

        

    rclpy.shutdown()
if __name__ == "__main__":
    main()
