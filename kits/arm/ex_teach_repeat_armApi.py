# -*- coding: utf-8 -*-
import arm
import mobile_io as mbio


# Set up arm
family_name = "Arm"
module_names = ("J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3")
hrdf = "hrdf/6-DoF_arm.hrdf"
p = arm.ArmParams(family_name, module_names, hrdf)
a = arm.Arm(p)

# Set up our mobile io interface
print('Waiting for Mobile IO device to come online...')
phone_family = "HEBI"
phone_name = "Cal's iPhone"
m = mbio.MobileIO(phone_family, phone_name)
state = m.getState()

abort_flag = False

waypoints = []
flow = []
durrations = []
run_mode = "training"
curr_waypoint_num = 0

while not abort_flag:
    # Update arm and mobile io
    a.update()
    prev_state = state
    state = m.getState()
    diff = m.getDiff(prev_state, state)
    slider3 = state[1][2]
    
    # Check for quit
    if diff[7] == "rising":
        abort_flag = True
        break
    
    if run_mode == "training":
        # B1 add waypoint (stop)
        if diff[0] == "rising":
            print("Waypoint added")
            waypoints.append(a.fbk.position)
            flow.append(False)
            durrations.append(slider3 + 4)
        
        # B1 add waypoint (flow)
        if diff[1] == "rising":
            print("Waypoint added")
            waypoints.append(a.fbk.position)
            flow.append(True)
            durrations.append(slider3 + 4)
        
        # B3 toggle training/playback
        if diff[2] == "rising":
            # Check for more than 2 waypoints
            if len(waypoints) > 1:
                run_mode = "playback"
                curr_waypoint_num = 0
                a.send()
                continue
            else:
                print("At least two waypoints are needed")
        
        # B4 clear waypoints
        if diff[4] == "rising":
            print("Waypoints cleared")
            waypoints = []
            flow = []
            durrations = []
            
    if run_mode == "playback":
        # B3 toggle training/playback
        if diff[2] == "rising":
            run_mode = "training"
            a.cancelGoal()
        
        # When at a waypoint, go to the next one
        if a.at_goal:
            a.setGoal(waypoints, flow=flow, durration=durrations)
            
    
    
    a.send()
        
        
        
        
        