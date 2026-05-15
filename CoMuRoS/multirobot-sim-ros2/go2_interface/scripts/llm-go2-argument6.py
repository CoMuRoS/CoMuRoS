import time
import sys
import openai
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from useful import (
    stand, move_linear, rotate_in_place, soft_turn, 
    goto_position, rotate_to_yaw_angle, circle, 
    square_trajectory_no_yaw
)
# from go2_commander import 
import math
from dataclasses import dataclass
import traceback

# ------------------------- Network Interface Detection -------------------------
def get_active_network_interface():
    """Detects the first active network interface (excluding loopback)."""
    for iface, addrs in psutil.net_if_addrs().items():
        if iface != "lo":  # Exclude loopback
            return iface
    return "eth0"  # Default to eth0 if no active interface found

network_interface = get_active_network_interface()

# ------------------------- Robot Task Options -------------------------
@dataclass
class TestOption:
    """Defines available robot actions and corresponding Python function calls."""
    name: str
    id: int
    description: str
    example_code: str

option_list = [
    TestOption(name="stand", id=1, description="Make the robot transition to a standing position.", example_code="stand(sport_client)"),
    TestOption(name="stand_down", id=2, description="Make the robot transition to a sitting or lying position.", example_code="sport_client.StandDown()"),
    TestOption(name="move forward", id=3, description="Move the robot forward.", example_code="move_linear(sport_client, 0.3, 0, 2)"),
    TestOption(name="move lateral", id=4, description="Move the robot laterally to the left.", example_code="move_linear(sport_client, 0, 0.3, 2)"),
    TestOption(name="move rotate", id=5, description="Rotate the robot on the spot.", example_code="rotate_in_place(sport_client, 0.5, 2)"),
    TestOption(name="stop_move", id=6, description="Stop all robot motion.", example_code="sport_client.StopMove()"),
    TestOption(name="switch_gait 0", id=7, description="Switch to gait type 0.", example_code="sport_client.SwitchGait(0)"),
    TestOption(name="switch_gait 1", id=8, description="Switch to gait type 1.", example_code="sport_client.SwitchGait(1)"),
    TestOption(name="balanced stand", id=9, description="Balance stand.", example_code="sport_client.BalanceStand()"),
    TestOption(name="recovery", id=10, description="Recover to a safe standing posture.", example_code="sport_client.RecoveryStand()"),
    TestOption(name="move back", id=11, description="Move the robot backward.", example_code="move_linear(sport_client, -0.3, 0, 2)"),
    TestOption(name="move right", id=12, description="Move the robot to the right.", example_code="move_linear(sport_client, 0, -0.3, 2)"),
    TestOption(name="turn left", id=14, description="Rotate counterclockwise.", example_code="rotate_in_place(sport_client, -0.5, 2)"),
    TestOption(name="turn right", id=15, description="Rotate clockwise.", example_code="rotate_in_place(sport_client, 0.5, 2)"),
    TestOption(name="goto_position", id=16, description="Move to a specific position.", example_code="goto_position(sport_client, 1, 1, 5)"),
    TestOption(name="rotate_to_yaw_angle", id=17, description="Rotate to a yaw angle.", example_code="rotate_to_yaw_angle(sport_client, 1.57, 5)"),
    TestOption(name="soft_turn", id=18, description="Perform a soft turn while moving forward.", example_code="soft_turn(sport_client, 0.1, 0.4, 5)"),
    TestOption(name="circle", id=19, description="Move in a circular path.", example_code="circle(sport_client, radius=0.5, duration=20)"),
    TestOption(name="square_trajectory", id=20, description="Move in a square trajectory.", example_code="square_trajectory_no_yaw(sport_client, side_length=2, speed=0.3)"),
    TestOption(name="damp", id=0, description="Reduce movement to a damped state.", example_code="sport_client.Damp()"),
]

# ------------------------- OpenAI Code Generation -------------------------
def generate_python_code(user_prompt):
    examples = "\n\n".join(
        [f"Function: {opt.name}\nExample Code:\n{opt.example_code}" for opt in option_list]
    )

    system_message = (
        "You are a Python code generator for controlling a quadruped robot. "
        "Only use predefined functions:\n" + examples
    )

    try:
        response = openai.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": system_message},
                {"role": "user", "content": user_prompt},
            ],
            max_tokens=500, temperature=0.2,
        )

        generated_code = response.choices[0].message.content.strip()
        if "```python" in generated_code:
            return generated_code.split("```python")[1].split("```")[0].strip(), "Generated successfully."
        return None, None
    except Exception as e:
        print(f"Error generating Python code: {e}")
        return None, None

# ------------------------- ROS 2 Task Completion Publisher -------------------------
# class TaskStatusPublisher(Node):
#     def __init__(self):
#         super().__init__('task_status_publisher')
#         self.status_publisher = self.create_publisher(String, '/chat/status', 10)
#         self.input_publisher = self.create_publisher(String, '/chat/input', 10)

#     def publish_completion_message(self):
#         """Publishes 'quadruped task COMPLETED' to /chat/status and /chat/input."""
#         msg = String()
#         msg.data = "quadruped task COMPLETED"

#         self.status_publisher.publish(msg)
#         self.input_publisher.publish(msg)

#         self.get_logger().info("Published 'quadruped task COMPLETED' to /chat/status and /chat/input")

# ------------------------- Execute Generated Python Code -------------------------
def execute_python_code(code, user_prompt):
    safe_globals = {
        "sport_client": sport_client,
        "stand": stand,
        "stand_down": lambda sc: sc.StandDown(),
        "move_linear": move_linear,
        "rotate_in_place": rotate_in_place,
        "soft_turn": soft_turn,
        "goto_position": goto_position,
        "rotate_to_yaw_angle": rotate_to_yaw_angle,
        "circle": circle,
        "square_trajectory_no_yaw": square_trajectory_no_yaw,
    }
    try:
        if "stop" not in user_prompt.lower() :

            # with open("data1.txt", "w") as g:
            #     g.write(f"Quadruped (status) : {user_prompt} : IN PROGRESS\n")  # Added newline for clarity

            exec(code, safe_globals)
            print("Data is written on data file")
            
            with open("data.txt", "a") as f:
                f.write("Quadruped (status) : TASKS COMPLETE\n")  # Added newline for clarity
                f.flush()

            # with open("data1.txt", "w") as g:
            #     g.write(f"Quadruped (status) : {user_prompt} : TASKS COMPLETE\n")  # Added newline for clarity
        else :
            exec(code, safe_globals)

    except Exception as e:
        print(f"Error executing Python code: {e}")
    traceback.print_exc()  # Print detailed error traceback

# ------------------------- Main Execution -------------------------
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} userPrompt")
        sys.exit(-1)

    user_prompt = sys.argv[1]

    ChannelFactoryInitialize(0, network_interface)
    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    generated_code, explanation = generate_python_code(user_prompt)

    if not generated_code or "sport_client" not in generated_code:
        print("Invalid or empty code.")
        sys.exit(1)

    print("\nGenerated Python Code:\n", generated_code)
    execute_python_code(generated_code, user_prompt)
