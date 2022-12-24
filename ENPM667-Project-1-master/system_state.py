"""system_state.py - contains a class definition of the attributes of the system at any given time"""


class SystemState:
    def __init__(self):
        # Instantaneous attributes
        self.body_x = 0
        self.body_y = 0
        self.body_z = 0

        self.yaw = 0
        self.pitch = 0
        self.roll = 0

        self.joint_positions = [0, 0, 0, 0, 0]

        # First derivative
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.rotational_velocity_yaw = 0
        self.rotational_velocity_pitch = 0
        self.rotational_velocity_roll = 0
        self.joint_velocities = [0, 0, 0, 0, 0]

        # Second derivative
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.rot_accel_yaw = 0
        self.rot_accel_pitch = 0
        self.rot_accel_roll = 0
        self.joint_accelerations = [0, 0, 0, 0, 0]


    def __str__(self):
        return str([self.body_x, self.body_y, self.body_z, self.yaw, self.pitch, self.roll, self.joint_positions, self.vx, self.vy, self.vz, self.rotational_velocity_yaw, self.rotational_velocity_pitch, self.rotational_velocity_roll, self.joint_velocities, self.ax, self.ay, self.az, self.rot_accel_yaw, self.rot_accel_pitch, self.rot_accel_roll, self.joint_accelerations])