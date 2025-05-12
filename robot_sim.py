# robot_sim.py
import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

#p.connect(p.GUI)
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)
cube_id = p.loadURDF("cube_small.urdf", [0.6, 0, 0.02])

arm_joints = [i for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot, i)[2] == p.JOINT_REVOLUTE]
finger_joints = [9, 10]  # Panda gripper fingers
end_effector_index = 11  # Tip link index for trajectory

trajectory = []

approach_pose = [0.0, 0.5, -0.6, -1.5, 0.0, 2.0, 1.5]
place_pose = [-0.5, 0.4, 0.3, -1.2, 0.0, 1.8, 1.4]

def record_trajectory():
    pos = p.getLinkState(robot, end_effector_index)[0]
    trajectory.append(pos)

def plot_trajectory():
    if not trajectory:
        return
    xs, ys, zs = zip(*trajectory)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xs, ys, zs, marker='o')
    ax.set_title("End-Effector Trajectory")
    os.makedirs("assets", exist_ok=True)
    plt.savefig("assets/trajectory.png")
    plt.close()

def move_to_angles(joint_values):
    current = [p.getJointState(robot, j)[0] for j in arm_joints]
    steps = 100
    for i in range(steps):
        blend = [(1 - i/steps)*c + (i/steps)*t for c, t in zip(current, joint_values)]
        for j, v in zip(arm_joints, blend):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, targetPosition=v)
        for _ in range(2):
            p.stepSimulation()
            record_trajectory()
            time.sleep(0.01)

def pick():
    move_to_angles(approach_pose)
    move_to_angles([j - 0.05 for j in approach_pose])
    for _ in range(50):
        for fj in finger_joints:
            p.setJointMotorControl2(robot, fj, p.POSITION_CONTROL, targetPosition=0.0)
        p.stepSimulation()
        record_trajectory()
        time.sleep(0.01)
    move_to_angles([j + 0.1 for j in approach_pose])

def place():
    move_to_angles(place_pose)
    for _ in range(50):
        for fj in finger_joints:
            p.setJointMotorControl2(robot, fj, p.POSITION_CONTROL, targetPosition=0.04)
        p.stepSimulation()
        record_trajectory()
        time.sleep(0.01)


def get_image():
    view_matrix = p.computeViewMatrix([1.5, 0, 1], [0, 0, 0.5], [0, 0, 1])
    proj_matrix = p.computeProjectionMatrixFOV(60, 1, 0.1, 3.1)
    _, _, img, _, _ = p.getCameraImage(640, 640, view_matrix, proj_matrix)
    return img[:, :, :3] if isinstance(img, np.ndarray) else None



def robot_chatbot(command, chat_history=None):
    if isinstance(command, str) and "pick" in command.lower():
        return pick()
    elif isinstance(command, str) and "place" in command.lower():
        return place()
    try:
        joint_values = [float(x.strip()) for x in command.split(",")]
        if len(joint_values) == 7:
            return move_to_joint_angles(joint_values)
        else:
            return None, "❌ Please provide 7 joint values."
    except Exception as e:
        return None, f"Error parsing command: {e}"


