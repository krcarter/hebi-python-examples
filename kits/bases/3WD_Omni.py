
import hebi
import numpy as np
from time import sleep, time
from hebi.util import create_mobile_io


def load_gains(group, gains_file):
    gains_command = hebi.GroupCommand(group.size)

    try:
        gains_command.read_gains(gains_file)
    except Exception as e:
        print(f'Warning - Could not load gains: {e}')
        return

    # Send gains multiple times
    for i in range(3):
        group.send_command(gains_command)
        sleep(0.1)

class OmniDrive:
    def __init__(self, group):
        self.WHEEL_RADIUS = 0.1524  # m
        self.BASE_RADIUS = 0.24 # m (half of distance between diff drive wheel centers)
        self.maxLinSpeed = 0.5 #m/s
        self.maxRotSpeed = self.maxLinSpeed / (self.BASE_RADIUS/2) # [rad/s]

        self.group = group
        self.base_command = hebi.GroupCommand(group.size)
        self.base_feedback = hebi.GroupFeedback(group.size)
        self.trajectory = None
        self.trajectory_start_time = 0
        self.start_wheel_pos = np.array([0, 0, 0])
        self.color = hebi.Color(0, 0, 0)

    def update(self, t_now):
        if not self.group.get_next_feedback(reuse_fbk=self.base_feedback):
            return False

        if self.trajectory:
            t = min(t_now - self.trajectory_start_time, self.trajectory.duration)
            p, v, _ = self.trajectory.get_state(t)

            self.base_command.position = self.start_wheel_pos + p
            self.base_command.velocity = v

            self.base_command.led.color = self.color
            '''
            print(f"t: {t}")
            print(f"P: {p}")
            print(f"V: {v}")
            '''
            self.group.send_command(self.base_command)

    def build_smooth_velocity_trajectory(self, dx, dy, dtheta, t_now):
    	# Ramp Up Time
        t0 = 0.00 # Start of Trajectory
        t1 = 0.15 # Reaches Desired Velocity
        t2 = 0.90 # Begins to Slow Wheel Down
        t3 = 1.20 # Wheel Zero Velocity
        times = np.array([t0, t1, t2, t3])

        cmd_vels = np.zeros(self.group.size)

        if self.trajectory is None:
            self.start_wheel_pos = self.base_feedback.position
        else:
            t = min(t_now - self.trajectory_start_time, self.trajectory.duration)
            cmd_pos, cmd_vels, _ = self.trajectory.get_state(t)
            self.start_wheel_pos += cmd_pos

        # Create wheel velocity vector
        target_vel_wheels = np.zeros(self.group.size)

        tabletTheta = np.arctan2(dy,dx)

        # Change robot body angle
        target_vel_wheels[0] = dtheta * (self.BASE_RADIUS / self.WHEEL_RADIUS)
        target_vel_wheels[1] = dtheta * (self.BASE_RADIUS / self.WHEEL_RADIUS)
        target_vel_wheels[2] = dtheta * (self.BASE_RADIUS / self.WHEEL_RADIUS)

        # X-Component of the Velocity
        target_vel_wheels[0] += dx / self.WHEEL_RADIUS
        target_vel_wheels[1] += dx / self.WHEEL_RADIUS
        target_vel_wheels[2] -= 2*dx*np.sin(np.deg2rad(60)) / self.WHEEL_RADIUS
        
        # Y-Component of the Velocity
        target_vel_wheels[0] -= dy / self.WHEEL_RADIUS
        target_vel_wheels[1] += dy / self.WHEEL_RADIUS
        target_vel_wheels[2] -= 0

        velocities = np.empty((self.group.size, 4))
        velocities[:, 0] = cmd_vels
        velocities[:, 1] = target_vel_wheels
        velocities[:, 2] = target_vel_wheels
        velocities[:, 3] = np.zeros((1, 3))

        self.build_velocity_trajectory(t_now, times, velocities)

    def build_velocity_trajectory(self, t_now, times, velocities):

        # start position 0, unconstrained waypoints afterwards
        p = np.empty((self.group.size, 4))
        p.fill(np.nan)
        p[:, 0] = 0

        a = np.zeros((self.group.size, 4))

        self.trajectory = hebi.trajectory.create_trajectory(times, p, velocities, a)
        self.trajectory_start_time = t_now

if __name__ == "__main__":

    lookup = hebi.Lookup()
    sleep(2)

    # Base setup
    base_family = "Rosie"
    module_names = ['W1', 'W2', 'W3']

    # Create group
    group = lookup.get_group_from_names([base_family], module_names)
    if group is None:
        raise RuntimeError(f"Could not find Omni modules: {module_names} in family '{base_family}'")
    load_gains(group, "gains/omni-drive-wheel-gains.xml")

    base = OmniDrive(group)

    # mobileIO setup
    phone_name = "mobileIO"

    print('Waiting for Mobile IO device to come online...')
    m = create_mobile_io(lookup, base_family, phone_name)
    if m is None:
        raise RuntimeError("Could not find Mobile IO device")
    else:
        print('Mobile IO Found')
    m.set_led_color("blue")
    m.clear_text()  # Clear any text from previous programs
    m.update()

    # Demo Variables
    abort_flag = False

    # Print Instructions
    instructions = """A1 - Rotational Control
A7 - X-Direction Control
A8 - Y-Direction Control
B8 - Quit
"""

    m.add_text(instructions)

    #######################
    ## Main Control Loop ##
    #######################

    while not abort_flag:

        if not m.update():
            print("Failed to get feedback from MobileIO")
            continue

        # B8 - Quit
        if m.get_button_diff(8) == 1:  # "ToOn"
            # Reset text & color, and quit
            m.clear_text()
            m.set_led_color("transparent")
            abort_flag = True
            break

        dtheta = pow(m.get_axis_state(1), 3) * 4.0
        dx = pow(m.get_axis_state(7), 3)
        dy = pow(m.get_axis_state(8), 3)

        if base.update(time()) == False:
            print("Failed to get feedback from Actuators")
        else:
            base.build_smooth_velocity_trajectory(dx, dy, dtheta, time())