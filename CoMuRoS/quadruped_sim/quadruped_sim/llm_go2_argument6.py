#!/usr/bin/env python3
import time
import sys
import openai
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# import psutil
# from unitree_sdk2py.core.channel import ChannelFactoryInitialize
# from unitree_sdk2py.go2.sport.sport_client import SportClient

from useful import (
    stand, move_linear, rotate_in_place, soft_turn, 
    goto_position, rotate_to_yaw_angle, circle, 
    square_trajectory_no_yaw
)

import math
from dataclasses import dataclass
import traceback


@dataclass
class TestOption:
    """Defines available robot actions and corresponding Python function calls."""
    name: str
    id: int
    description: str
    example_code: str

# option_list = [
#     TestOption(name="stand", id=1, description="Make the robot transition to a standing position.", example_code="stand()"),
#     TestOption(name="stand_down", id=2, description="Make the robot transition to a sitting or lying position.", example_code="sport_client.StandDown()"),
#     TestOption(name="move forward", id=3, description="Move the robot forward.", example_code="move_linear(sport_client, 0.3, 0, 2)"),
#     TestOption(name="move lateral", id=4, description="Move the robot laterally to the left.", example_code="move_linear(sport_client, 0, 0.3, 2)"),
#     TestOption(name="move rotate", id=5, description="Rotate the robot on the spot.", example_code="rotate_in_place(sport_client, 0.5, 2)"),
#     TestOption(name="stop_move", id=6, description="Stop all robot motion.", example_code="sport_client.StopMove()"),
#     TestOption(name="switch_gait 0", id=7, description="Switch to gait type 0.", example_code="sport_client.SwitchGait(0)"),
#     TestOption(name="switch_gait 1", id=8, description="Switch to gait type 1.", example_code="sport_client.SwitchGait(1)"),
#     TestOption(name="balanced stand", id=9, description="Balance stand.", example_code="sport_client.BalanceStand()"),
#     TestOption(name="recovery", id=10, description="Recover to a safe standing posture.", example_code="sport_client.RecoveryStand()"),
#     TestOption(name="move back", id=11, description="Move the robot backward.", example_code="move_linear(sport_client, -0.3, 0, 2)"),
#     TestOption(name="move right", id=12, description="Move the robot to the right.", example_code="move_linear(sport_client, 0, -0.3, 2)"),
#     TestOption(name="turn left", id=14, description="Rotate counterclockwise.", example_code="rotate_in_place(-0.5, 2)"),
#     TestOption(name="turn right", id=15, description="Rotate clockwise.", example_code="rotate_in_place(0.5, 2)"),
#     TestOption(name="goto_position", id=16, description="Move to a specific position.", example_code="goto_position( 1, 1, 5)"),
#     TestOption(name="rotate_to_yaw_angle", id=17, description="Rotate to a yaw angle.", example_code="rotate_to_yaw_angle(sport_client, 1.57, 5)"),
#     TestOption(name="soft_turn", id=18, description="Perform a soft turn while moving forward.", example_code="soft_turn(sport_client, 0.1, 0.4, 5)"),
#     TestOption(name="circle", id=19, description="Move in a circular path.", example_code="circle(sport_client, radius=0.5, duration=20)"),
#     TestOption(name="square_trajectory", id=20, description="Move in a square trajectory.", example_code="square_trajectory_no_yaw(sport_client, side_length=2, speed=0.3)"),
#     TestOption(name="damp", id=0, description="Reduce movement to a damped state.", example_code="sport_client.Damp()"),
# ]

option_list = [
    TestOption(name="sit", id=0,
               description="Make the robot sit (lower body Z to -0.1).",
               example_code="controller.sit()"),

    TestOption(name="stand", id=1,
               description="Make the robot stand (raise body Z to 0.0).",
               example_code="controller.stand()"),

    TestOption(name="go_to_position", id=2,
               description="Move the robot to a specific X, Y, Z position using open-loop timer-based control.",
               example_code="controller.go_to_position(1.0, 1.0, 0.0)"),

    TestOption(name="move_forward", id=3,
               description="Command the robot to move forward with a fixed velocity for 2 seconds.",
               example_code="controller.move_forward()"),

    TestOption(name="move_backward", id=4,
               description="Command the robot to move backward with a fixed velocity for 2 seconds.",
               example_code="controller.move_backward()"),

    TestOption(name="stop", id=5,
               description="Immediately stop all robot movement and cancel current motion.",
               example_code="controller.stop()"),
]


class QuadrupedSimulator(Node):
    def __init__(self):
        super().__init__('quadruped_simulator')
        self.publisher_1 = self.create_publisher(String, '/chat/task_status', 10)


    def generate_python_code(self, user_prompt):
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


    def execute_python_code(self, code, user_prompt):
        safe_globals = {
            # "sport_client": sport_client,
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

                exec(code, safe_globals)
                # print("Data is written on data file")
                
                # with open("data.txt", "a") as f:
                #     f.write("Quadruped (status) : TASKS COMPLETE\n")  
                #     f.flush()
                msg = String()
                msg.data = "Quadruped (status) : TASKS COMPLETE"
                self.publisher_1.publish(msg)

            else :
                exec(code, safe_globals)

        except Exception as e:
            print(f"Error executing Python code: {e}")
        traceback.print_exc()  

def main(args=None):
    
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} userPrompt")
        sys.exit(-1)

    user_prompt = sys.argv[1]

    rclpy.init(args=args)
    node =  QuadrupedSimulator()


    generated_code, explanation = node.generate_python_code(user_prompt)


    if not generated_code:
        print("Invalid or empty code.")
        sys.exit(1)

    print("\nGenerated Python Code:\n", generated_code)
    node.execute_python_code(generated_code, user_prompt)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


