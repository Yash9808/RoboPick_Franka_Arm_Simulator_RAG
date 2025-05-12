import gradio as gr
from robot_sim import robot_chatbot

with gr.Blocks() as demo:
    chatbot = gr.Chatbot()
    with gr.Row():
        with gr.Column():
            msg = gr.Textbox(label="Type Command (e.g., 'Pick', 'Place', or joint angles')")
            send = gr.Button("Send")
        output_img = gr.Image(type="filepath", label="Robot View")

    def handle_message(user_input, chat_history):
        response, img_path = robot_chatbot(user_input, chat_history)
        chat_history.append((user_input, response))
        return chat_history, "", img_path

    send.click(handle_message, inputs=[msg, chatbot], outputs=[chatbot, msg, output_img])

demo.launch()
