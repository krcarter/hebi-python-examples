#!/usr/bin/env python3

import hebi
import numpy as np
import os
from time import sleep
from hebi.util import create_mobile_io
from hebi import arm as arm_api

# Arm setup
phone_family = "HEBI"
phone_name   = "mobileIO"
arm_family   = "Arm"
hrdf_file    = "hrdf/A-2099-07G.hrdf"
gains_file    = "gains/A-2099-07G.xml"

lookup = hebi.Lookup()
sleep(2)

# Setup MobileIO
print('Waiting for Mobile IO device to come online...')
m = create_mobile_io(lookup, phone_family, phone_name)
if m is None:
  raise RuntimeError("Could not find Mobile IO device")
m.update()

# Setup arm components
arm = arm_api.create([arm_family],
                     names=['J1_base', 'J2A_shoulder1', 'J2B_shoulder2', 'J3_elbow', 'J4_wrist1', 'J5_wrist2', 'J6_wrist3'],
                     lookup=lookup,
                     hrdf_file=hrdf_file)
alt_shoulder_group = lookup.get_group_from_names(arm_family, ['J2B_shoulder1'])
double_shoulder = arm_api.DoubleJointedMirror(2, alt_shoulder_group)
arm.add_plugin(double_shoulder)

arm.load_gains(gains_file)

keep_running = True
pending_goal = False
run_mode = "training"
goal = arm_api.Goal(arm.size)

print("")
print("B1 - Add waypoint (stop)")
print("B2 - Add waypoint (flow)")
print("A3 - Up/down for longer/shorter time to waypoint")
print("B3 - Toggle training/playback")
print("B4 - Clear waypoints")
print("B8 - Quit")
print("")

while keep_running:
  # If there is a goal pending, set it on the arm and clear the flag
  if pending_goal:
    arm.set_goal(goal)
    pending_goal = False

  if not arm.update():
    print("Failed to update arm")
    continue

  if m.update(timeout_ms=0):

    slider3 = m.get_axis_state(3)

    # Check for quit
    if m.get_button_diff(8) == 3: # "ToOn"
      keep_running = False
      break

    if run_mode == "training":
      # B1 add waypoint (stop)
      if m.get_button_diff(1) == 3: # "ToOn"
        print("Stop waypoint added")
        goal.add_waypoint(t=slider3 + 4.0, position=arm.last_feedback.position, velocity=[0,0,0,0,0,0])

      # B2 add waypoint (flow)
      if m.get_button_diff(2) == 3: # "ToOn"
        print("Flow waypoint added")
        goal.add_waypoint(t=slider3 + 4.0, position=arm.last_feedback.position)

      # B3 toggle training/playback
      if m.get_button_diff(3) == 3: # "ToOn"
        # Check for more than 2 waypoints
        if goal.waypoint_count > 1:
          print("Transitioning to playback mode")
          run_mode = "playback"
          pending_goal = True
        else:
          print("At least two waypoints are needed")

      # B4 clear waypoints
      if m.get_button_diff(4) == 3: # "ToOn"
        print("Waypoints cleared")
        goal.clear()

    elif run_mode == "playback":
      # B3 toggle training/playback
      if m.get_button_diff(3) == 3: # "ToOn"
        print("Transitioning to training mode")
        run_mode = "training"
        arm.cancel_goal()

      # replay through the path again once the goal has been reached
      if arm.at_goal:
        arm.set_goal(goal)

  arm.send()
