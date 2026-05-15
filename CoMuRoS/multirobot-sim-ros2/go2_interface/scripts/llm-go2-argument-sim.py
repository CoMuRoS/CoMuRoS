import rclpy
from go2_commander import Go2Interface  # Import your class
import openai
import sys
import traceback
from dataclasses import dataclass

# ------------------------- Define Task Options -------------------------
@dataclass
class TestOption:
    name: str
    id: int
    description: str
    example_code: str

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
               description="Move forward for 2 seconds.",
               example_code="controller.move_forward()"),
    TestOption(name="move_backward", id=4,
               description="Move backward for 2 seconds.",
               example_code="controller.move_backward()"),
    TestOption(name="stop", id=5,
               description="Stop all movement.",
               example_code="controller.stop()"),
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
        "Only generate valid code using `controller.` object to access those methods."
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
        print(f"❌ Error generating code: {e}")
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

            print("Data is written on data file")
            with open("data.txt", "a") as f:
                f.write("Quadruped (status) : TASKS COMPLETE\n")  # Added newline for clarity
                f.flush()
        else:
            exec(code, safe_globals)
            print("⛔ Robot stopped.")
    except Exception as e:
        print(f"❌ Error executing code: {e}")
        traceback.print_exc()

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
        print("❌ No valid code generated.")
        sys.exit(1)

    print("\n🧠 Generated Code:\n" + code)
    execute_generated_code(code, controller, user_prompt)

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
