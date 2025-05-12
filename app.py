# app.py
import gradio as gr
from robot_sim import robot_chatbot

chat_history = []

def handle_message(user_input, history):
    img_path, response = robot_chatbot(user_input)
    if response is None:
        response = "🤖 Something went wrong."
    history.append((user_input, response))
    return history, img_path

with gr.Blocks() as demo:
    gr.Markdown("## 🤖 Robotic RAG Chatbot Interface")
    chatbot = gr.Chatbot()
    msg = gr.Textbox(label="Enter Command", placeholder="e.g. pick or 0.0, -0.5, 0.3, -1.2, 0.0, 1.5, 0.8")
    output_img = gr.Image(type="filepath", label="Simulation View")
    send = gr.Button("Send")

    send.click(fn=handle_message, inputs=[msg, chatbot], outputs=[chatbot, output_img])

demo.launch()
