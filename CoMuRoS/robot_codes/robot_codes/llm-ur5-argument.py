import rclpy
from ur5_commander import UR5Commander # Import your class
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

ns = ""

option_list = [
    TestOption(name="pick an object", id=0,
               description="Make the UR5 perform the pick routine by passing argument compulsorily. There are two types of UR5 , If UR5 Chef is mentioned call commander.pick_routine() ; Else if UR5 Helper is mentioned call commander.pick_routine()",
               example_code=f"commander.pick_routine()"),
    TestOption(name="place an object", id=1,
               description="Make the UR5 perform the place routine by passing argument compulsorily. There are two types of UR5 , If UR5 Chef is mentioned call commander.place_routine() ; Else if UR5 Helper is mentioned call commander.place_routine()",
               example_code=f"commander.place_routine()"),
]

# ------------------------- Code Generation from Prompt -------------------------
def generate_python_code(user_prompt):
    examples = "\n\n".join(
        [f"Function: {opt.name}\nExample Code:\n{opt.example_code}" for opt in option_list]
    )

    system_message = (
        "You are a Python code generator for controlling UR5 Arms with Vacuum Gripper. "
        "Only use the functions provided in the following list. Do NOT invent new functions.\n\n"
        f"{examples}\n"
        "Only generate valid code using `commander.` object to access those methods.\n"

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
def execute_generated_code(code, commander, user_prompt, ns):
    safe_globals = {
        "commander": commander
    }

    try:
        if "stop" not in user_prompt.lower():
            exec(code, safe_globals)
            print("✅ Code executed.")

            print("Data is written on data file")
            if ns == "arm1":
                with open("/home/vipul/ws/mia_ws/src/robot_codes/robot_codes/data1.txt", "a") as f:
                    f.write("UR5 Chef (status) : TASKS COMPLETE\n")  # Added newline for clarity 
                    f.flush()
            elif ns == "arm2":
                with open("/home/vipul/ws/mia_ws/src/robot_codes/robot_codes/data2.txt", "a") as f:
                    f.write("UR5 Helper (status) : TASKS COMPLETE\n")  # Added newline for clarity
                    f.flush()
        else:
            exec(code, safe_globals)
            print("⛔ Robot stopped.")
    except Exception as e: 
        print(f"❌ Error executing code: {e}")
        traceback.print_exc()

# ------------------------- Main Execution -------------------------
def main():
    if len(sys.argv) < 3:
        print(f"Usage: python3 {sys.argv[0]} \"<instruction>\" <namespace>")
        print("Example: python3 script.py \"pick and place\" arm1")
        sys.exit(-1)

    user_prompt = sys.argv[1]
    ns = sys.argv[3]

    rclpy.init()
    print(ns)
    commander = UR5Commander(ns)

    code, explanation = generate_python_code(user_prompt)
    if not code:
        print("❌ No valid code generated.")
        sys.exit(1)

    print("\n🧠 Generated Code:\n" + code)
    execute_generated_code(code, commander, user_prompt, ns)

    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
