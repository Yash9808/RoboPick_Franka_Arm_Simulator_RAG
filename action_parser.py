import re

def parse_action(command: str):
    # Normalize and extract 7 float values (joint angles)
    angle_match = re.findall(r'-?\d+\.\d+', command)
    if len(angle_match) == 7:
        return {
            "type": "move",
            "angles": [float(x) for x in angle_match]
        }

    # Handle specific keywords
    cmd = command.lower()
    if "pick" in cmd:
        return {"type": "pick"}
    if "place" in cmd:
        return {"type": "place"}

    return {"type": "unknown", "message": "Command not recognized. Try 'pick', 'place', or provide 7 joint angles."}
