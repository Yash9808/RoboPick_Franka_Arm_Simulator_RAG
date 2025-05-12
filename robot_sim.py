import pybullet as p
import pybullet_data
import numpy as np
import tempfile
import matplotlib.pyplot as plt
import os

class RobotSimulator:
    def __init__(self):
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.loadURDF("plane.urdf")
        self.robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)
        self.cube = p.loadURDF("cube_small.urdf", [0.6, 0, 0.02])
        self.arm_joints = [i for i in range(p.getNumJoints(self.robot)) if p.getJointInfo(self.robot, i)[2] == p.JOINT_REVOLUTE]
        self.fingers = [9, 10]

    def move_joints(self, joint_angles):
        for idx, angle in zip(self.arm_joints, joint_angles):
            p.setJointMotorControl2(self.robot, idx, p.POSITION_CONTROL, angle)
        for _ in range(50):
            p.stepSimulation()

    def control_gripper(self, open_grip=True):
        val = 0.04 if open_grip else 0.0
        for f in self.fingers:
            p.setJointMotorControl2(self.robot, f, p.POSITION_CONTROL, targetPosition=val)
        for _ in range(50):
            p.stepSimulation()

    def pick(self):
        self.control_gripper(open_grip=False)

    def place(self):
        self.control_gripper(open_grip=True)

    def render(self):
        width, height = 512, 512
        view = p.computeViewMatrix([1.5, 0, 1], [0, 0, 0.5], [0, 0, 1])
        proj = p.computeProjectionMatrixFOV(60, width / height, 0.1, 3.1)
        _, _, img, _, _ = p.getCameraImage(width, height, view, proj)
        rgb = np.reshape(img, (height, width, 4))[:, :, :3]

        tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".png")
        plt.imsave(tmp.name, rgb.astype(np.uint8))
        return tmp.name
