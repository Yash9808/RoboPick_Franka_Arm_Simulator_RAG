import gradio as gr
from robot_sim import robot_chatbot

chat_history = []

def handle_message(user_input):
    global chat_history
    response, (sim_img_path, traj_img_path) = robot_chatbot(user_input)
    traj_img_path = traj_img_path or ""  # Avoid None
    chat_history.append((user_input, response))
    return chat_history, sim_img_path, traj_img_path

with gr.Blocks() as demo:
    chatbot = gr.Chatbot()
    with gr.Row():
        user_input = gr.Textbox(label="Enter Command")
    with gr.Row():
        sim_view = gr.Image(label="Simulation View", type="filepath")
        traj_view = gr.Image(label="3D Trajectory", type="filepath")
    send_btn = gr.Button("Send")

    send_btn.click(handle_message, inputs=user_input, outputs=[chatbot, sim_view, traj_view])
    user_input.submit(handle_message, inputs=user_input, outputs=[chatbot, sim_view, traj_view])

demo.launch()
