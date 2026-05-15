import rclpy
from go2_commander2 import Go2Interface  # Import your class
import openai
import sys
import traceback
from dataclasses import dataclass
from std_msgs.msg import String
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
    TestOption(name="stand", id=1,
               description="Make the robot stand (raise body Z to 0.0).",
               example_code="controller.stand()"),
    TestOption(name="go_to_position", id=2,
               description="Move the robot to a specific (x, y, z) position .",
               example_code="controller.go_to_position(1.0, 1.0, 0.0)"),
    TestOption(name="move_forward", id=3,
               description="Move forward for 2 seconds.",
               example_code="controller.move_forward()"),
    TestOption(name="move_backward", id=4,
               description="Move backward for 2 seconds.",
               example_code="controller.move_backward()"),
    TestOption(name="stop", id=5,
               description="Stop all movement.",
               example_code="controller.stop()"),
    TestOption(name="go_to_object", id=6,
               description="Goto an specified object by passing the particular object as a parameter",
               example_code="controller.go_to_object('bedroom')"),
    # TestOption(name="success", id=7,
    #            description="This function should always be called after complete code generation",
    #            example_code="controller.success()"),
]

# ------------------------- Code Generation from Prompt -------------------------
def generate_python_code(user_prompt):
    examples = "\n\n".join(
        [f"Function: {opt.name}\nExample Code:\n{opt.example_code}" for opt in option_list]
    )

    system_message = (
        "You are a Python code generator for controlling a quadruped robot. "
        "Only use the functions provided in the following list. Do NOT invent new functions.\n\n"
        f"{examples}\n"
        "Only generate valid code using `controller.` object to access those methods.\n"
        "For go_to_position() -> Dont assume any parameters before hand , if parameters are not clearly defined for any function you can give controller.stop() command instead of randomly calling controller.go_to_position(1.0, 1.0, 0.0) or controller.go_to_position(0.0, 0.0, 0.0) or controller.go_to_position()\n"
        # "When invoking go_to_object(), the model must carefully analyze the full prompt and issue the command exclusively for the final intended object or destination mentioned. The target passed to go_to_object() must reflect the ultimate goal location or endpoint of the task, not intermediate objects or waypoints."
        "Dont assume that the position is defined elsewhere ."
        "Only following objects can be used in go_to_object"
        "Dont always call controller.stop() always after completing any task as Quadruped stops automatically."
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
def execute_generated_code(code, controller, user_prompt):
    safe_globals = {
        "controller": controller
    }

    try:
        if "stop" not in user_prompt.lower():
            exec(code, safe_globals)
            print("✅ Code executed.")
        else:
            exec(code, safe_globals)
            print("Robot stopped.")
    except Exception as e:
        print(f"Error executing code: {e}")
        traceback.print_exc()

def tasks_completed():
    with open("/home/vipul/ws/himura_ws/src/quadruped_sim/quadruped_sim/data.txt", "a") as f:
        f.write("Quadruped (status) : TASKS COMPLETE\n")  # Added newline for clarity
        f.flush()

# ------------------------- Main Execution -------------------------
def main():
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} \"user instruction here\"")
        sys.exit(-1)

    user_prompt = sys.argv[1]

    rclpy.init()
    controller = Go2Interface()

    code, explanation = generate_python_code(user_prompt)
    if not code:
        print("No valid code generated.")
        sys.exit(1)

    print("\nGenerated Code:\n" + code)
    execute_generated_code(code, controller, user_prompt)
    tasks_completed()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main() 