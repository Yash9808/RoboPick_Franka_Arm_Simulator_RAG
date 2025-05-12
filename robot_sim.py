import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import tempfile
import time

# Set up the simulation in DIRECT mode
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Load robot and environment
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0], useFixedBase=True)
cube_id = p.loadURDF("cube_small.urdf", basePosition=[0.6, 0, 0.02])
p.changeVisualShape(cube_id, -1, rgbaColor=[0, 0, 0, 1])

# Identify joint indices
def get_joint_indices(robot):
    arm, fingers = [], []
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        name = info[1].decode("utf-8")
        joint_type = info[2]
        if "finger" in name:
            fingers.append(i)
        elif joint_type == p.JOINT_REVOLUTE:
            arm.append(i)
    return arm, fingers

arm_joints, finger_joints = get_joint_indices(robot_id)
trajectory = []  # Stores joint trajectory for 3D plot

# Render camera view
def capture_view():
    width, height = 640, 640
    view_matrix = p.computeViewMatrix(cameraEyePosition=[1.2, 0, 1],
                                      cameraTargetPosition=[0, 0, 0.4],
                                      cameraUpVector=[0, 0, 1])
    proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                               aspect=1.0,
                                               nearVal=0.1,
                                               farVal=3.1)
    _, _, px, _, _ = p.getCameraImage(width=width, height=height,
                                      viewMatrix=view_matrix,
                                      projectionMatrix=proj_matrix)
    rgb_array = np.reshape(px, (height, width, 4))[:, :, :3]

    fig, ax = plt.subplots()
    ax.imshow(rgb_array.astype(np.uint8))
    ax.axis("off")
    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".png")
    plt.savefig(tmp.name, bbox_inches='tight')
    plt.close()
    return tmp.name

# Move robot to given joint angles
def move_robot(joint_angles, grip=0.04, steps=100):
    if len(joint_angles) != 7:
        return "❌ Invalid joint input."

    for step in range(steps):
        blend = [(1 - step / steps) * p.getJointState(robot_id, j)[0] + (step / steps) * t
                 for j, t in zip(arm_joints, joint_angles)]
        for j, val in zip(arm_joints, blend):
            p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, targetPosition=val)
        for f in finger_joints:
            p.setJointMotorControl2(robot_id, f, p.POSITION_CONTROL, targetPosition=grip)
        p.stepSimulation()
        pos = p.getLinkState(robot_id, arm_joints[-1])[0]
        trajectory.append(pos)
        time.sleep(0.005)

# Execute pick and place
def perform_pick_and_place():
    approach = [0.0, -0.5, 0.3, -1.5, 0.0, 1.4, 0.6]
    pick = [0.0, -0.4, 0.3, -1.4, 0.0, 1.2, 0.6]
    place = [0.5, -0.4, 0.2, -1.8, 0.0, 1.4, 0.6]

    move_robot(approach)
    move_robot(pick, grip=0.0)  # Close gripper
    move_robot([a + 0.1 for a in pick], grip=0.0)
    move_robot(place, grip=0.0)
    move_robot(place, grip=0.04)  # Open gripper

# Plot 3D trajectory
def plot_trajectory():
    if not trajectory:
        return None

    traj = np.array(trajectory)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], label="End Effector Path")
    ax.scatter(traj[-1, 0], traj[-1, 1], traj[-1, 2], color='red', label='Final Position')
    ax.set_title("3D Trajectory")
    ax.legend()
    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".png")
    plt.savefig(tmp.name)
    plt.close()
    return tmp.name

# Main control function
def robot_chatbot(user_input: str):
    user_input = user_input.lower().strip()
    if "pick" in user_input and "place" in user_input:
        trajectory.clear()
        perform_pick_and_place()
        return "✅ Pick and place executed.", capture_view()

    try:
        values = [float(v.strip()) for v in user_input.split(",")]
        if len(values) == 7:
            trajectory.clear()
            move_robot(values)
            return "✅ Moved to specified joint angles.", capture_view()
    except Exception:
        pass

    return "🤖 Command not recognized. Try 'pick', 'place', or provide 7 joint angles.", capture_view()

