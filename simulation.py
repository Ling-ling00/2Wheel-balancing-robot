import pybullet as p
import pybullet_data
import time
import numpy as np
from controller import Controller
import matplotlib.pyplot as plt

x_velocity_data = []
pitch_velocity_data = []
yaw_velocity_data = []
simulation_time_data = []
left_torque_data = []
right_torque_data = []
x_position_data = []
y_position_data = []

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the ground plane
plane = p.loadURDF("plane.urdf")

# Set environment
p.setGravity(0, 0, -9.81)
p.changeDynamics(plane, -1, lateralFriction=1.0)
timeStep = 1.0 / 100.0
start_time = time.time()
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

# Control parameters
left_torque = 0
right_torque = 0
threshold = 0.005

# Initialize variables for calculating accelerations
previous_wheel_velocities = np.array([[0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0]])
previous_base_velocity_robot = np.zeros(10)
previous_yaw_velocity = np.zeros(10)
previous_pitch_velocity = np.zeros(10)

# Parameter
linear_speed_slider = p.addUserDebugParameter(" Linear Speed (X)", -1, 1, 0)  # Min, Max, Default
angular_speed_slider = p.addUserDebugParameter(" Angular Speed (Yaw)", -0.5, 0.5, 0)  # Min, Max, Default
x_slider = p.addUserDebugParameter(" X Position", -3, 3, 0)  # Min, Max, Default
y_slider = p.addUserDebugParameter(" Y Position", -3, 3, 0)  # Min, Max, Default
mode_button = p.addUserDebugParameter(" Switch mode", 1, 0, 0)  # Min, Max, Default

# Simulation loop with feedback
while True:
    # Apply torque control
    p.setJointMotorControl2(robot, 1, p.TORQUE_CONTROL, force=left_torque)
    p.setJointMotorControl2(robot, 2, p.TORQUE_CONTROL, force=right_torque)

    # Step simulation
    p.stepSimulation()
    time.sleep(timeStep)
    current_time = time.time() - start_time
    
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
    print(f"Left_torque: {left_torque:.4f}, Right_torque: {right_torque:.4f}")
    print(f"Forward Velocity: {x_velocity:.4f} m/s")
    print(f"Body Tilt: {pitch:.4f}, Body Tilt Velocity: {pitch_velocity:.4f}")
    print(f"Robot Angle: {yaw:.4f}, Robot Rotation velocity: {yaw_velocity:.4f}")
    print(f"Left Wheel: Position (q): {wheel_feedback[0][0]:.4f}, Velocity (qdot): {wheel_feedback[0][1]:.4f}")
    print(f"Right Wheel: Position (q): {wheel_feedback[1][0]:.4f}, Velocity (qdot): {wheel_feedback[1][1]:.4f}")
    print(f"X: {x}, Y: {y}")
    print("------")

    linear_speed = p.readUserDebugParameter(linear_speed_slider)
    angular_speed = p.readUserDebugParameter(angular_speed_slider)
    target_x = p.readUserDebugParameter(x_slider)
    target_y = p.readUserDebugParameter(y_slider)
    mode = p.readUserDebugParameter(mode_button)

    if mode%2 == 1:
        v, w = robot_control.positionControl(target_x, target_y, x, y, yaw)
        linear_angle = robot_control.linearVControl(v, wheel_feedback[0][1], wheel_feedback[1][1])
        turn_torque = robot_control.angularVControl(w, wheel_feedback[0][1], wheel_feedback[1][1])
    else:
        linear_angle = robot_control.linearVControl(linear_speed, wheel_feedback[0][1], wheel_feedback[1][1])
        turn_torque = robot_control.angularVControl(angular_speed, wheel_feedback[0][1], wheel_feedback[1][1])
        position_flag = 0
    ff_torque = robot_control.feedForward(pitch, linear_angle)
    balance_torque = robot_control.balanceControl(pitch, pitch_velocity, linear_angle)
    new_left_torque = ff_torque + balance_torque - turn_torque
    new_right_torque = ff_torque + balance_torque + turn_torque
    left_torque = max(min(left_torque + threshold, new_left_torque), left_torque - threshold)
    right_torque = max(min(right_torque + threshold, new_right_torque), right_torque - threshold)

     # Record data
    simulation_time_data.append(current_time)
    x_velocity_data.append(x_velocity)
    yaw_velocity_data.append(yaw_velocity)
    pitch_velocity_data.append(pitch_velocity)
    left_torque_data.append(left_torque)
    right_torque_data.append(right_torque)
    x_position_data.append(x)
    y_position_data.append(y)

    # Stop simulation after 50 seconds
    if current_time >= 100:
        break

p.disconnect()

fig, axs = plt.subplots(5, 1, figsize=(20, 25), constrained_layout=True)

# Plot x_velocity_data
# Plot x_velocity (desired vs actual)
axs[0].plot(simulation_time_data, x_velocity_data, label='x_velocity', linewidth=2)
axs[0].set_title('Forward Velocity')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Velocity (m/s)')
axs[0].legend()
axs[0].grid()

# Plot pitch_velocity_data
axs[1].plot(simulation_time_data, pitch_velocity_data, label='pitch_velocity', linewidth=2,)
axs[1].set_title('Pitch Velocity (pitch_velocity) Over Time')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Velocity (rad/s)')
axs[1].legend()
axs[1].grid()

# Plot yaw_velocity (desired vs actual)
axs[2].plot(simulation_time_data, yaw_velocity_data, label='yaw_velocity', linewidth=2)
axs[2].set_title('Yaw Velocity')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylabel('Velocity (rad/s)')
axs[2].legend()
axs[2].grid()

# Plot left_torque_data
axs[3].plot(simulation_time_data, left_torque_data, label='left_torque', linewidth=2, linestyle='-.')
axs[3].set_title('Left Torque (left_torque) Over Time')
axs[3].set_xlabel('Time (s)')
axs[3].set_ylabel('Torque (Nm)')
axs[3].legend()
axs[3].grid()

# Plot right_torque_data
axs[4].plot(simulation_time_data, right_torque_data, label='right_torque', linewidth=2, linestyle='-.')
axs[4].set_title('Right Torque (right_torque) Over Time')
axs[4].set_xlabel('Time (s)')
axs[4].set_ylabel('Torque (Nm)')
axs[4].legend()
axs[4].grid()

# Show the plot
plt.show()

fig, axs = plt.subplots(2, 1, figsize=(20, 25), constrained_layout=True)
# Plot x_velocity (desired vs actual)
axs[0].plot(simulation_time_data, x_position_data, label='x_position', linewidth=2)
axs[0].set_title('X Position')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('distance (m)')
axs[0].legend()
axs[0].grid()

# Plot pitch_velocity_data
axs[1].plot(simulation_time_data, y_position_data, label='y_position', linewidth=2,)
axs[1].set_title('Y Position')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('distance (m)')
axs[1].legend()
axs[1].grid()

plt.show()