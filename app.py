import gradio as gr
from robot_sim import RobotSimulator
from action_parser import parse_user_input

robot = RobotSimulator()

def chat_command_handler(msg, history):
    result = parse_user_input(msg)
    if result["action"] == "move":
        robot.move_joints(result["angles"])
        img = robot.render()
        return [(msg, "✅ Moved to position."), (None, img)]
    elif result["action"] == "pick":
        robot.pick()
        img = robot.render()
        return [(msg, "🟡 Picked the object."), (None, img)]
    elif result["action"] == "place":
        robot.place()
        img = robot.render()
        return [(msg, "🔵 Placed the object."), (None, img)]
    else:
        return [(msg, f"❌ {result['msg']}")]

gr.ChatInterface(chat_command_handler, title="🤖 Robotic Arm Controller").launch()
