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
timeStep = 1.0 / 500.0
p.setTimeStep(timeStep)

# Load the robot URDF
robot = p.loadURDF("two_wheel_robot.urdf", [0, 0, 0.05], useFixedBase=False)
robot_control = Controller(1.0, 0.2, 0.2, 0.2, 0.0166667, 0.014166667, 0.004166667, 0.00025, 0.05, timeStep)

# Get the joint indices
num_joints = p.getNumJoints(robot)
for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    print(f"Joint {i}: {info[1].decode('utf-8')}")

for i in range(1,3):
    p.changeDynamics(robot, i, lateralFriction=1.0)
    p.setJointMotorControl2(robot, i, p.VELOCITY_CONTROL, force=0)
    p.resetJointState(robot, i, targetValue=0, targetVelocity=0)

# Control parameters
left_torque = 0
right_torque = 0

# Initialize variables for calculating accelerations
previous_wheel_velocities = np.array([[0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0]])
previous_base_velocity_robot = np.zeros(10)
previous_yaw_velocity = np.zeros(10)
previous_pitch_velocity = np.zeros(10)

# For compare
calculate_velo = [[], [], []]
real_velo = [[], [], []]
current_cal_velo = [0, 0, 0]
time_stamp = []
current_time = 0.0
is_compare = True
prev_state = 0

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

    # Print feedback
    # print(f"Left_torque: {left_torque:.4f}, Right_torque: {right_torque:.4f}")
    # print(f"Forward Velocity: {x_velocity:.4f} m/s")
    # print(f"Body Tilt: {pitch:.4f}, Body Tilt Velocity: {pitch_velocity:.4f}")
    # print(f"Robot Angle: {yaw:.4f}, Robot Rotation velocity: {yaw_velocity:.4f}")
    # print(f"Left Wheel: Position (q): {wheel_feedback[0][0]:.4f}, Velocity (qdot): {wheel_feedback[0][1]:.4f}")
    # print(f"Right Wheel: Position (q): {wheel_feedback[1][0]:.4f}, Velocity (qdot): {wheel_feedback[1][1]:.4f}")
    # print("------")

    ff_torque = robot_control.feedForward(pitch)
    balance_torque = robot_control.balanceControl(pitch, pitch_velocity)
    linear_torque = robot_control.linearVControl(1, wheel_feedback[0][1], wheel_feedback[1][1])
    turn_torque = robot_control.angularVControl(1, wheel_feedback[0][1], wheel_feedback[1][1])
    left_torque = ff_torque + balance_torque + linear_torque + turn_torque
    right_torque = ff_torque + balance_torque + linear_torque - turn_torque

    acceleration = robot_control.compare([pitch, pitch_velocity], left_torque, right_torque)
    x_accel = acceleration[0][0]
    pitch_accel = acceleration[1][0]
    yaw_accel = acceleration[2][0]

    if is_compare and current_time < 2.0:
        calculate_velo[0].append(current_cal_velo[0])
        calculate_velo[1].append(current_cal_velo[1])
        calculate_velo[2].append(current_cal_velo[2])
        real_velo[0].append(x_velocity)
        real_velo[1].append(pitch_velocity)
        real_velo[2].append(yaw_velocity)
        time_stamp.append(current_time)

        # current_cal_velo = [x_velocity, pitch_velocity, yaw_velocity]
        current_cal_velo[0] -= acceleration[0][0] * (timeStep)
        current_cal_velo[1] -= acceleration[1][0] * (timeStep)
        current_cal_velo[2] -= acceleration[2][0] * (timeStep)
        current_time += timeStep
    elif is_compare:
        p.disconnect()
        print(real_velo[0][0], calculate_velo[0][0])
        components = ['X Velocity', 'Pitch Velocity', 'Yaw Velocity']
        fig, axs = plt.subplots(3, 2, figsize=(12, 10))  # 3 rows, 2 columns

        for i in range(3):  # Loop through 3 components
            # Plot real velocity on the left column
            axs[i, 0].plot(time_stamp, real_velo[i], label=f"Real {components[i]}")
            axs[i, 0].set_xlabel("Time (s)")
            axs[i, 0].set_ylabel(f"{components[i]} (m/s or rad/s)")
            axs[i, 0].set_title(f"Real {components[i]}")
            axs[i, 0].legend()
            axs[i, 0].grid()

            # Plot calculated velocity on the right column
            axs[i, 1].plot(time_stamp, calculate_velo[i], label=f"Calculated {components[i]}")
            axs[i, 1].set_xlabel("Time (s)")
            axs[i, 1].set_ylabel(f"{components[i]} (m/s or rad/s)")
            axs[i, 1].set_title(f"Calculated {components[i]}")
            axs[i, 1].legend()
            axs[i, 1].grid()

        plt.tight_layout()  # Adjust spacing between subplots
        plt.show()
        is_compare = False
