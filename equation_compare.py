import pybullet as p
import pybullet_data
import time
import numpy as np
from controller import Controller
import matplotlib.pyplot as plt

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the ground plane
plane = p.loadURDF("plane.urdf")

# Set environment
p.setGravity(0, 0, -9.81)
p.changeDynamics(plane, -1, lateralFriction=1.0)
timeStep = 1.0 / 100.0
p.setTimeStep(timeStep)

# Load the robot URDF
robot = p.loadURDF("two_wheel_robot.urdf", [0, 0, 0.05], useFixedBase=False, flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_IMPLICIT_CYLINDER)
robot_control = Controller(1.0, 0.2, 0.2, 0.2, 0.0166667, 0.014166667, 0.004166667, 0.00025, 0.05, timeStep)

# Get the joint indices
num_joints = p.getNumJoints(robot)
for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    print(f"Joint {i}: {info[1].decode('utf-8')}")

for i in range(1,3):
    p.changeDynamics(robot, i, lateralFriction=1.0, angularDamping = 1.0)
    p.setJointMotorControl2(robot, i, p.VELOCITY_CONTROL, force=0)
    p.resetJointState(robot, i, targetValue=0, targetVelocity=0)
    p.enableJointForceTorqueSensor(robot, i, 1)

# Control parameters
left_torque = 0
right_torque = 0
threshold = 0.005

# Initialize variables for calculating accelerations
previous_wheel_velocities = np.array([[0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0]])
previous_base_velocity_robot = np.zeros(10)
previous_yaw_velocity = np.zeros(10)
previous_pitch_velocity = np.zeros(10)

#Parameter
linear_speed_slider = p.addUserDebugParameter("Linear Speed (X)", -1, 1, 0)  # Min, Max, Default
angular_speed_slider = p.addUserDebugParameter("Angular Speed (Yaw)", -0.5, 0.5, 0)  # Min, Max, Default
start = p.addUserDebugParameter("Start", 1, 0, 0)

# For compare
calculate_velo = [[], [], []]
real_velo = [[], [], []]
current_cal_velo = [0, 0, 0]
time_stamp = []
current_time = 0.0
prev_state = 0
f = [[], [], [], [], [], []]

# Simulation loop with feedback
while True:
    # Apply torque control
    p.setJointMotorControl2(robot, 1, p.TORQUE_CONTROL, force=left_torque)
    p.setJointMotorControl2(robot, 2, p.TORQUE_CONTROL, force=right_torque)

    # Step simulation
    p.stepSimulation()
    time.sleep(timeStep)

    # Robot base orientation and Convert linear velocity to robot's local frame
    base_position, base_orientation = p.getBasePositionAndOrientation(robot)
    base_linear_velocity, base_angular_velocity = p.getBaseVelocity(robot)
    rotation_matrix = p.getMatrixFromQuaternion(base_orientation)
    rotation_matrix = np.array(rotation_matrix).reshape(3, 3)
    linear_velocity_robot = np.dot(rotation_matrix.T, base_linear_velocity)  # Transform velocity
    x = base_position[0]
    y = base_position[1]
    x_velocity = linear_velocity_robot[0]  # Forward velocity in robot's local frame

    # Get yaw from quaternion
    _, _, yaw = p.getEulerFromQuaternion(base_orientation)
    yaw_velocity = base_angular_velocity[2]  # Angular velocity around Z-axis

    # Body tilt around y-axis (pitch)
    _, pitch, _ = p.getEulerFromQuaternion(base_orientation)
    pitch_velocity = base_angular_velocity[1]

    # Wheel positions and velocities
    wheel_feedback = []
    for i in range(1, 3):  # Assuming joints 1 and 2 are wheels
        joint_state = p.getJointState(robot, i)
        q = joint_state[0]  # Position
        qdot = joint_state[1]  # Velocity
        wheel_feedback.append((q, qdot))
        print(joint_state[2], left_torque, right_torque)
        print(joint_state[3])

    linear_speed = p.readUserDebugParameter(linear_speed_slider)
    angular_speed = p.readUserDebugParameter(angular_speed_slider)
    is_start = p.readUserDebugParameter(start)

    if is_start > 0:
        linear_angle = robot_control.linearVControl(linear_speed, wheel_feedback[0][1], wheel_feedback[1][1])
        turn_torque = robot_control.angularVControl(angular_speed, wheel_feedback[0][1], wheel_feedback[1][1])
    else:
        linear_angle = robot_control.linearVControl(0, wheel_feedback[0][1], wheel_feedback[1][1])
        turn_torque = robot_control.angularVControl(0, wheel_feedback[0][1], wheel_feedback[1][1])
    ff_torque = robot_control.feedForward(pitch, linear_angle)
    balance_torque = robot_control.balanceControl(pitch, pitch_velocity, linear_angle)
    new_left_torque = ff_torque + balance_torque - turn_torque
    new_right_torque = ff_torque + balance_torque + turn_torque
    left_torque = max(min(left_torque + threshold, new_left_torque), left_torque - threshold)
    right_torque = max(min(right_torque + threshold, new_right_torque), right_torque - threshold)

    if is_start > 0:
        if current_time < 10.0:
            contact_l = p.getContactPoints(robot, plane, 1, -1)
            lfx_l = 0
            lfy_l = 0
            lfz_l = 0
            for point in contact_l:
                lfx_l += point[11][0] * point[10] + point[13][0] * point[12]
                lfy_l += point[11][1] * point[10] + point[13][1] * point[12]
                lfz_l += point[11][2] * point[10] + point[13][2] * point[12]
            f[0].append(lfx_l)
            f[1].append(lfy_l)
            f[2].append(lfz_l)

            contact_r = p.getContactPoints(robot, plane, 2, -1)
            lfx_r = 0
            lfy_r = 0
            lfz_r = 0
            for point in contact_r:
                lfx_r += point[11][0] * point[10] + point[13][0] * point[12]
                lfy_r += point[11][1] * point[10] + point[13][1] * point[12]
                lfz_r += point[11][2] * point[10] + point[13][2] * point[12]
            f[3].append(lfx_r)
            f[4].append(lfy_r)
            f[5].append(lfz_r)

            # acceleration = robot_control.compare([pitch, pitch_velocity], left_torque, right_torque, yaw_velocity, x_velocity, (np.cos(yaw)*lfx_l + np.sin(yaw)*lfy_l)*robot_control.r, (np.cos(yaw)*lfx_r + np.sin(yaw)*lfy_r)*robot_control.r)
            acceleration = robot_control.compare([pitch, pitch_velocity], left_torque, right_torque, [x_velocity, pitch_velocity, yaw_velocity], [wheel_feedback[0][1], wheel_feedback[1][1]])
            x_accel = acceleration[0][0]
            pitch_accel = acceleration[1][0]
            yaw_accel = acceleration[2][0]

            current_cal_velo[0] = Controller.rk4(current_cal_velo[0], lambda x: acceleration[0][0], timeStep)
            current_cal_velo[1] = Controller.rk4(current_cal_velo[1], lambda x: acceleration[1][0], timeStep)
            current_cal_velo[2] = Controller.rk4(current_cal_velo[2], lambda x: acceleration[2][0], timeStep)

            calculate_velo[0].append(current_cal_velo[0])
            calculate_velo[1].append(current_cal_velo[1])
            calculate_velo[2].append(current_cal_velo[2])
            real_velo[0].append(x_velocity)
            real_velo[1].append(pitch_velocity)
            real_velo[2].append(yaw_velocity)
            time_stamp.append(current_time)

            current_time += timeStep
        else:
            p.disconnect()
            components = ['X Velocity', 'Pitch Velocity', 'Yaw Velocity']
            fig, axs = plt.subplots(3, 2, figsize=(12, 10))

            for i in range(3):
                axs[i, 0].plot(time_stamp, real_velo[i], label=f"Real {components[i]}")
                axs[i, 0].set_xlabel("Time (s)")
                axs[i, 0].set_ylabel(f"{components[i]} (m/s or rad/s)")
                axs[i, 0].set_title(f"Real {components[i]}")
                axs[i, 0].legend()
                axs[i, 0].grid()

                axs[i, 1].plot(time_stamp, calculate_velo[i], label=f"Calculated {components[i]}")
                axs[i, 1].set_xlabel("Time (s)")
                axs[i, 1].set_ylabel(f"{components[i]} (m/s or rad/s)")
                axs[i, 1].set_title(f"Calculated {components[i]}")
                axs[i, 1].legend()
                axs[i, 1].grid()

            plt.tight_layout()
            plt.show()

            # components = ['fx_l', 'fy_l', 'fz_l', 'fx_r', 'fy_r', 'fz_r']
            # fig, axs = plt.subplots(3, 2, figsize=(12, 10))

            # for i in range(6):
            #     axs[i//2, i%2].plot(time_stamp, f[i], label=f"{components[i]}")
            #     axs[i//2, i%2].set_xlabel("Time (s)")
            #     axs[i//2, i%2].set_ylabel(f"{components[i]}")
            #     axs[i//2, i%2].set_title(f"{components[i]}")
            #     axs[i//2, i%2].legend()
            #     axs[i//2, i%2].grid()

            # plt.tight_layout()
            # plt.show()
