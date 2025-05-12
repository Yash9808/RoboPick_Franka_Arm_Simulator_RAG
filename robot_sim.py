# robot_sim.py
import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import tempfile
import time

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0], useFixedBase=True)
cube_id = p.loadURDF("cube_small.urdf", basePosition=[0.6, 0, 0.02])
p.changeVisualShape(cube_id, -1, rgbaColor=[0, 0, 0, 1])

def get_joints():
    arm, fingers = [], []
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        name = info[1].decode()
        if "finger" in name:
            fingers.append(i)
        elif info[2] == p.JOINT_REVOLUTE:
            arm.append(i)
    return arm, fingers

arm_joints, finger_joints = get_joints()
current_joint_angles = [0] * 7
grip = 0.02

def render(joints, grip):
    for idx, val in zip(arm_joints, joints):
        p.setJointMotorControl2(robot, idx, p.POSITION_CONTROL, targetPosition=val)
    for fj in finger_joints:
        p.setJointMotorControl2(robot, fj, p.POSITION_CONTROL, targetPosition=grip)
    for _ in range(20): p.stepSimulation()

    view_matrix = p.computeViewMatrix([1.5, 0, 1], [0, 0, 0.5], [0, 0, 1])
    proj_matrix = p.computeProjectionMatrixFOV(60, 1, 0.1, 3.1)
    #_, _, img, _, _ = p.getCameraImage(640, 640, view_matrix, proj_matrix)
    #rgb = np.reshape(img, (640, 640, 4))[:, :, :3]

    width, height = 1024, 1024
    _, _, img, _, _ = p.getCameraImage(width, height, view_matrix, proj_matrix)
    rgb = np.reshape(img, (height, width, 4))[:, :, :3]


    fig, ax = plt.subplots()
    ax.imshow(rgb)
    ax.axis("off")
    tmp = tempfile.NamedTemporaryFile(suffix=".png", delete=False)
    plt.savefig(tmp.name, bbox_inches='tight')
    plt.close()

    return tmp.name

def robot_chatbot(input_text, history):
    global current_joint_angles, grip

    if "pick" in input_text.lower():
        for _ in range(50):
            for fj in finger_joints:
                p.setJointMotorControl2(robot, fj, p.POSITION_CONTROL, targetPosition=0.0)
            p.stepSimulation()
            time.sleep(0.01)
        grip = 0.0
        response = "🟢 Object picked."

    elif "place" in input_text.lower():
        for _ in range(50):
            for fj in finger_joints:
                p.setJointMotorControl2(robot, fj, p.POSITION_CONTROL, targetPosition=0.04)
            p.stepSimulation()
            time.sleep(0.01)
        grip = 0.04
        response = "🟢 Object placed."

    elif "joint" in input_text.lower():
        try:
            parts = input_text.lower().replace("joint", "").replace("angles", "").strip()
            new_angles = [float(x.strip()) for x in parts.split(",")]
            if len(new_angles) != 7:
                response = "⚠️ Please provide exactly 7 joint angles."
            else:
                current_joint_angles = new_angles
                response = f"✅ Moved to new joint angles."
        except:
            response = "❌ Failed to parse joint angles."

    else:
        response = "🤖 Command not recognized. Try 'pick', 'place', or provide 7 joint angles."

    img_path = render(current_joint_angles, grip)
    return response, img_path
