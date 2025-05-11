import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import tempfile
import gradio as gr
import time

# Setup PyBullet
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0], useFixedBase=True)

# Add a cube to pick (black color)
cube_id = p.loadURDF("cube_small.urdf", basePosition=[0.6, 0, 0.02])
p.changeVisualShape(cube_id, -1, rgbaColor=[0, 0, 0, 1])  # Change the cube color to black

# Get joint indices
def get_panda_joints(robot):
    arm, fingers = [], []
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        name = info[1].decode()
        joint_type = info[2]
        if "finger" in name and joint_type == p.JOINT_PRISMATIC:
            fingers.append(i)
        elif joint_type == p.JOINT_REVOLUTE:
            arm.append(i)
    return arm, fingers

arm_joints, finger_joints = get_panda_joints(robot)

# Add 3D debug labels to the robot's joints
debug_labels = []
def add_joint_labels():
    global debug_labels
    for i in debug_labels:
        p.removeUserDebugItem(i)
    debug_labels.clear()
    for idx in arm_joints:
        link_state = p.getLinkState(robot, idx)
        pos = link_state[0]
        lbl = f"J{arm_joints.index(idx)+1}"
        text_id = p.addUserDebugText(lbl, pos, textColorRGB=[1, 0, 0], textSize=1.2)
        debug_labels.append(text_id)

# Render image and info
def render_sim(joint_values, gripper_val):
    # Apply joint controls
    for idx, tgt in zip(arm_joints, joint_values):
        p.setJointMotorControl2(robot, idx, p.POSITION_CONTROL, targetPosition=tgt)

    # Open/close gripper symmetrically
    if len(finger_joints) == 2:
        p.setJointMotorControl2(robot, finger_joints[0], p.POSITION_CONTROL, targetPosition=gripper_val)
        p.setJointMotorControl2(robot, finger_joints[1], p.POSITION_CONTROL, targetPosition=gripper_val)

    for _ in range(10): p.stepSimulation()

    # Refresh joint labels
    add_joint_labels()

    # Camera
    width, height = 1280, 1280  # Increased resolution for clarity
    view_matrix = p.computeViewMatrix([1.5, 0, 1], [0, 0, 0.5], [0, 0, 1])
    proj_matrix = p.computeProjectionMatrixFOV(60, width / height, 0.1, 3.1)
    _, _, img, _, _ = p.getCameraImage(width, height, view_matrix, proj_matrix)
    rgb = np.reshape(img, (height, width, 4))[:, :, :3]

    # Image to file
    fig, ax = plt.subplots(figsize=(5, 5))
    ax.imshow(rgb.astype(np.uint8))
    ax.axis("off")
    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".png")
    plt.savefig(tmp.name, bbox_inches='tight', dpi=200)  # Increased DPI for sharper image
    plt.close()

    # Text output
    joint_text = "Joint Angles:\n" + "\n".join([f"J{i+1} = {v:.2f} rad" for i, v in enumerate(joint_values)])
    joint_text += f"\nGripper = {gripper_val:.3f} m"
    return tmp.name, joint_text

# Smooth motion to input angles
def move_to_input_angles(joint_str):
    try:
        target_angles = [float(x.strip()) for x in joint_str.split(",")]
        if len(target_angles) != 7:
            return None, "❌ Please enter exactly 7 joint angles."

        # Get current joint positions
        current = [p.getJointState(robot, idx)[0] for idx in arm_joints]
        steps = 100
        for i in range(steps):
            blend = [(1 - i/steps) * c + (i/steps) * t for c, t in zip(current, target_angles)]
            for idx, val in zip(arm_joints, blend):
                p.setJointMotorControl2(robot, idx, p.POSITION_CONTROL, targetPosition=val)
            p.stepSimulation()
            time.sleep(0.01)

        current_grip = p.getJointState(robot, finger_joints[0])[0]
        return render_sim(target_angles, current_grip)

    except Exception as e:
        return None, f"Error: {str(e)}"

# Simulated Retrieval Function: Fetch task-related info from an external source (simulated here)
def retrieve_task_data(task):
    # Simulating a retrieval step where the task data is fetched (e.g., from a database)
    if task == "pick_up_object":
        return {"object": "cube", "position": [0.6, 0, 0.02], "size": "small"}
    elif task == "pick_up_box":
        return {"object": "box", "position": [0.7, 0, 0.02], "size": "medium"}
    else:
        return {"object": "unknown", "position": [0, 0, 0], "size": "unknown"}

# Perform Pick and Place Task using Retrieval-Aided Generation (RAG)
def pick_and_place(position_angles, approach_angles, place_angles):
    # Retrieve task data (this would usually be an API call or database query)
    task_data = retrieve_task_data("pick_up_object")
    object_position = task_data["position"]
    object_size = task_data["size"]
    print(f"Task: {task_data['object']} at position {object_position}, size: {object_size}")
    
    # Parse the angles from user input
    position_angles = [float(x.strip()) for x in position_angles.split(",")]
    approach_angles = [float(x.strip()) for x in approach_angles.split(",")]
    place_angles = [float(x.strip()) for x in place_angles.split(",")]

    # Move towards the position
    move_to_input_angles(approach_angles)

    # Close gripper to pick
    for _ in range(50):
        for fj in finger_joints:
            p.setJointMotorControl2(robot, fj, p.POSITION_CONTROL, targetPosition=0.0)
        p.stepSimulation()
        time.sleep(0.01)

    # Lift the object
    move_to_input_angles([x + 0.1 for x in approach_angles])  # Example lift motion

    # Move to the place position
    move_to_input_angles(place_angles)

    # Open gripper to place
    for _ in range(50):
        for fj in finger_joints:
            p.setJointMotorControl2(robot, fj, p.POSITION_CONTROL, targetPosition=0.04)  # Open gripper
        p.stepSimulation()
        time.sleep(0.01)

    return render_sim(place_angles, 0.04)  # Final joint angles after placement

# Gradio UI
with gr.Blocks(title="Franka Arm Control with 7 DoF and Gripper Options") as demo:
    gr.Markdown("## 🤖 Franka 7-DOF Control\nUse the sliders to manipulate the robot arm.")
    
    # Gripper selection
    gripper_selector = gr.Dropdown(["Two-Finger", "Suction"], value="Two-Finger", label="Select Gripper")
    gripper_feedback = gr.Textbox(label="Gripper Status", interactive=False)
    gripper_selector.change(fn=switch_gripper, inputs=gripper_selector, outputs=gripper_feedback)

    # 7 DoF sliders
    joint_sliders = []
    with gr.Row():
        for i in range(4):
            joint_sliders.append(gr.Slider(-3.14, 3.14, value=0, label=f"Joint {i+1}"))
    with gr.Row():
        for i in range(4, 7):
            joint_sliders.append(gr.Slider(-3.14, 3.14, value=0, label=f"Joint {i+1}"))
        gripper = gr.Slider(0.0, 0.04, value=0.02, step=0.001, label="Gripper Opening")

    # Outputs
    with gr.Row():
        img_output = gr.Image(type="filepath", label="Simulation View")
        text_output = gr.Textbox(label="Joint States")

    # Live update
    def live_update(*vals):
        joints = list(vals[:-1])
        grip = vals[-1]
        return render_sim(joints, grip)

    for s in joint_sliders + [gripper]:
        s.change(fn=live_update, inputs=joint_sliders + [gripper], outputs=[img_output, text_output])

    # Reset button
    def reset():
        return render_sim([0]*7, 0.02)

    gr.Button("🔄 Reset Robot").click(fn=reset, inputs=[], outputs=[img_output, text_output])

    # Joint angles input box for user to input manually
    gr.Markdown("### 🧾 Enter Joint Angles (comma-separated)")
    joint_input = gr.Textbox(label="Joint Angles (7 values in radians)", placeholder="e.g. 0.0, -0.5, 0.3, -1.2, 0.0, 1.5, 0.8")
    gr.Button("▶️ Move to Angles").click(fn=move_to_input_angles, inputs=joint_input, outputs=[img_output, text_output])

    # Pick and Place Inputs
    gr.Markdown("### 🧾 Enter Joint Angles for Pick and Place")
    position_input = gr.Textbox(label="Position Angles (7 values in radians)", placeholder="e.g. 0.0, -0.5, 0.3, -1.2, 0.0, 1.5, 0.8")
    approach_input = gr.Textbox(label="Approach Angles (7 values in radians)", placeholder="e.g. 0.0, -0.3, 0.3, -1.5, 0.0, 1.0, 0.6")
    place_input = gr.Textbox(label="Place Angles (7 values in radians)", placeholder="e.g. 0.5, -0.4, 0.2, -1.8, 0.0, 1.4, 0.6")
    gr.Button("🤖 Perform Pick and Place").click(fn=pick_and_place, inputs=[position_input, approach_input, place_input], outputs=[img_output, text_output])

demo.launch(debug=True)
