import rclpy
import openai
import sys
import traceback
from dataclasses import dataclass
from sjtu_drone_control.drone_position_control import DronePositionControl

# ------------------------- Define Task Options -------------------------
@dataclass
class TestOption:
    name: str
    id: int
    description: str
    example_code: str

option_list = [
    # TestOption(name="sit", id=0,
    #            description="Make the robot sit (lower body Z to -0.1).",
    #            example_code="controller.sit()"),
    # TestOption(name="stand", id=1,
    #            description="Make the robot stand (raise body Z to 0.0).",
    #            example_code="controller.stand()"),
    TestOption(name="search/find operation", id=0,
               description="Move the drone to start surveillance search/find operation.",
               example_code="drone.start_surveillance()"),
    # TestOption(name="move_forward", id=3,
    #            description="Move forward for 2 seconds.",
    #            example_code="controller.move_forward()"),
    # TestOption(name="move_backward", id=4,
    #            description="Move backward for 2 seconds.",
    #            example_code="controller.move_backward()"),
    # TestOption(name="stop", id=5,
    #            description="Stop all movement.",
    #            example_code="controller.stop()"),
]

# ------------------------- Code Generation from Prompt -------------------------
def generate_python_code(user_prompt):
    examples = "\n\n".join(
        [f"Function: {opt.name}\nExample Code:\n{opt.example_code}" for opt in option_list]
    )

    system_message = (
        "You are a Python code generator for controlling a Quadcopter Drone. "
        "Only use the functions provided in the following list. Do NOT invent new functions.\n\n"
        f"{examples}\n"
        "Only generate valid code using `drone` object to access those methods. USE only the above methods , see example code to for execution."
        "Dont assume random locations in case of controller.go_to_position() . In case of ambiguity of X , Y and Z coordinates for go_to_location function you can call controller.stop() directly, instead of controller.go_to_position(1.0,1.0,0.0) or controller.go_to_position()"
    )

    try:
        response = openai.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": system_message},
                {"role": "user", "content": user_prompt},
            ],
            max_tokens=500,
            temperature=0.2,
        )

        code = response.choices[0].message.content.strip()
        if "```python" in code:
            return code.split("```python")[1].split("```")[0].strip(), "Generated successfully."
        return code, "Generated successfully."
    except Exception as e:
        print(f"Error generating code: {e}")
        return None, None

# ------------------------- Execute the Code -------------------------
def execute_generated_code(code, drone, user_prompt):
    safe_globals = {
        "drone": drone
    }

    try:
        if "stop" not in user_prompt.lower():
            exec(code, safe_globals)
            print("✅ Code executed.")

            print("Data is written on data file")
            with open("/home/vipul/ws/mia_ws/src/sjtu_drone/sjtu_drone_control/sjtu_drone_control/data.txt", "a") as f:
                f.write("Drone (status) : TASKS COMPLETE\n")  # Added newline for clarity
                f.flush()
        else:
            exec(code, safe_globals)
            print("Robot stopped.")
    except Exception as e:
        print(f"Error executing code: {e}")
        traceback.print_exc()

# ------------------------- Main Execution -------------------------
def main():
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} \"user instruction here\"")
        sys.exit(-1)

    user_prompt = sys.argv[1]

    rclpy.init()
    drone = DronePositionControl()

    code, explanation = generate_python_code(user_prompt)
    if not code:
        print("No valid code generated.")
        sys.exit(1)

    print("\nGenerated Code:\n" + code)
    execute_generated_code(code, drone, user_prompt)

    rclpy.spin(drone)
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()