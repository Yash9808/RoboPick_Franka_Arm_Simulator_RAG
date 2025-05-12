def parse_user_input(msg: str):
    msg = msg.lower()
    if "pick" in msg:
        return {"action": "pick"}
    elif "place" in msg:
        return {"action": "place"}
    elif "move to" in msg:
        # Example: move to 0.0, -0.5, 0.3, -1.2, 0.0, 1.5, 0.8
        try:
            angles = [float(x) for x in msg.split("move to")[1].split(",")]
            if len(angles) == 7:
                return {"action": "move", "angles": angles}
        except:
            return {"action": "error", "msg": "Failed to parse joint angles."}
    return {"action": "error", "msg": "Unknown command."}
