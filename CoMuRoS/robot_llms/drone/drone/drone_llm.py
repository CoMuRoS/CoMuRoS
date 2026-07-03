#!/usr/bin/env python3
import json
import time

import openai
import base64
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage
from dataclasses import dataclass
import os
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup
)
from ament_index_python.packages import get_package_share_directory

from robot_interface.srv import GotoPoseDrone
from openai import OpenAI

ROBOT_NAME   = 'drone'
ROBOT_TYPE   = "Quadrotor UAV"
NODE_NAME    = "drone_llm_node"
PACKAGE_NAME = "drone"

api_key = os.getenv("OPENAI_API_KEY")


class TaskCancelledException(Exception):
    """Custom exception to signal task cancellation"""
    pass


# Define available actions
@dataclass
class TestOption:
    name: str
    id: int
    description: str
    example_code: str

option_list = [
    TestOption(
        name="Hover",
        id=0,
        description="This is used to make the drone hover or it can be used to goto the specified position",
        example_code="node.hover(x=1.0, y=2.0, z=3.0, yaw_deg=0.0)"
    ),
    TestOption(
        name="DescribeScene",
        id=1,
        description="This is used to describe the current scene view of the drone given a prompt. ALL QUESTIONS about the scene MUST be handled in one unified function without repeating it. All doubts must be solved so add all the questions related to the doubt.",
        example_code="node.describe_scene(prompt='in which table is child is sitting?')"
    ),
]

class RobotLLMNode(Node):
    """
    A hub node that:
      - Listens to chat topics (/chat/output, /chat/history, /chat/task_status)
      - Publishes task status updates (/chat/task_status) and robot-specific status (<robot_name>_task_status)
      - Mirrors/consumes robot state messages on /robot_states (expects JSON strings)
    """

    _instance = None

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        # ---- Parameters ----
        self.declare_parameter('robot_name', ROBOT_NAME)
        self.robot_name: str = self.get_parameter('robot_name').value
        robot_task_topic = f'{self.robot_name}_task_status'

        RobotLLMNode._instance = self

        self.current_time = f"Hours: {00}, Minutes: {10}, Seconds: {00}"
        self.robot_task = ""
        self.robot_states = {}

        self.Visionclient = OpenAI(api_key=api_key)
        self.latest_image_b64 = None

        # ========== CANCELLATION MECHANISM ==========
        self._task_cancelled = False  # Simple boolean flag
        # ============================================

        # GROUPS
        self.single_group = MutuallyExclusiveCallbackGroup()
        self.seq_group = MutuallyExclusiveCallbackGroup()
        self.multi_group = ReentrantCallbackGroup()

        # ---- Publishers ----
        self.pub_task_status = self.create_publisher(String, '/chat/task_status', 10)
        self.pub_input_msg = self.create_publisher(String, '/chat/input', 10)
        self.pub_robot_states = self.create_publisher(String, '/robot_states', 10)
        self.pub_robot_task = self.create_publisher(String, robot_task_topic, 10)

        self._goto_client = self.create_client(GotoPoseDrone, '/r3/goto_pose', callback_group=self.multi_group)
        self._cancel_goto_pub = self.create_publisher(Bool, '/r3/cancel_goto_pose_goal', 10)

        # ---- Subscriptions ----
        self.sub_robot_states = self.create_subscription(
            String, '/robot_states', self.on_robot_states, 10,
            callback_group=self.seq_group
        )
        self.current_time_sub = self.create_subscription(
            String, '/current_time', self.on_current_time, 10,
            callback_group=self.seq_group
        )
        self.sub_tasks_json = self.create_subscription(
            String, '/task_manager/tasks_json', self.on_tasks_json, 10,
            callback_group=self.multi_group
        )
        self.sub_chat_output = self.create_subscription(
            String, '/chat/output', self.on_chat_output, 10,
            callback_group=self.single_group
        )

        # self.sub_chat_history = self.create_subscription(
        #     String, '/chat/history', self.on_chat_history, 10
        # )
        self.sub_chat_task_status = self.create_subscription(
            String, '/chat/task_status', self.on_chat_task_status, 10
        )

        # ---- Timer Callbacks ----
        # self.timer_period = 1.0  # seconds
        # self.robot_task_status_callback = self.create_timer(
        #     self.timer_period, self.robot_task_status_update,
        #     callback_group=self.multi_group
        # )

        self.create_subscription(
            CompressedImage,
            "/r3/bottom_camera/color/image_raw/compressed",
            self.image_callback, 10,
            callback_group=self.single_group
        )

        self.get_logger().info(
            f'RobotLLMNode started for robot="{self.robot_name}". '
            f'Publishing robot task status on "{robot_task_topic}".'
        )

        directry = "data"
        package_path = get_package_share_directory(PACKAGE_NAME)

        script_name = self.robot_name+"_chat_history.txt"
        self.history_file = os.path.join(package_path, directry, script_name)

        script_name = self.robot_name+'_task_history.txt'
        self.robot_task_history = os.path.join(package_path, directry, script_name)

        self.clear_files()
        self.robot_has_no_current_task()

    @classmethod
    def get_instance(cls):
        """Safely get or create the singleton node."""
        if cls._instance is None:
            raise RuntimeError("RobotLLMNode has not been created yet! Did you run the node?")
        return cls._instance

    # ========== CANCELLATION HELPERS ==========
    def check_cancelled(self):
        """Check if task has been cancelled. Raise exception if true."""
        if self._task_cancelled:
            self.get_logger().warn("Task cancellation detected!")
            raise TaskCancelledException("Task was cancelled")

    def reset_cancellation(self):
        """Clear the cancellation flag (call before starting new task)"""
        self._task_cancelled = False
    # ==========================================

    def clear_files(self) -> None:
        """Clear the chat history file on startup."""
        if os.path.exists(self.history_file):
            with open(self.history_file, "w") as file:
                file.write("")
            self.get_logger().info("Cleared chat history file on startup.")
        else:
            with open(self.history_file, "w") as file:
                file.write("")
            self.get_logger().warn(f"Chat history file not found. Created new file: {self.history_file}")

        if os.path.exists(self.robot_task_history):
            with open(self.robot_task_history, "w") as file:
                file.write("")
            self.get_logger().info("Cleared the robot task history file on startup.")
        else:
            self.get_logger().warn(f"Robot Task history file not found: {self.robot_task_history}")
            with open(self.robot_task_history, "w") as file:
                file.write("")
            self.get_logger().warn(f"Robot task history file not found. Created new file: {self.robot_task_history}")

    # -------------------- Callbacks --------------------

    def image_callback(self, msg: CompressedImage):
        """Store latest image in Base64."""
        try:
            self.latest_image_b64 = base64.b64encode(msg.data).decode("utf-8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def on_robot_states(self, msg: String) -> None:
        """Handle robot state updates (expects JSON string in msg.data)."""
        try:
            data = json.loads(msg.data)
            rs = data.get("robot_states")
            if isinstance(rs, dict):
                self.robot_states = rs
                self.get_logger().debug(f"robot_states keys: {list(self.robot_states.keys())}")
            else:
                self.get_logger().warning("/robot_states missing or not a dict; ignoring payload.")
        except json.JSONDecodeError:
            self.get_logger().error(f"/robot_states not JSON: {msg.data}")

    def on_tasks_json(self, msg: String) -> None:
        """Handle tasks JSON from task manager."""
        self.get_logger().debug(f'Received /task_manager/tasks_json: {msg.data}')
        try:
            data = json.loads(msg.data)
            robot_tasks = {
                key.lower().replace(" ", "_"): value
                for key, value in data.get("robot_tasks", {}).items()
            }

            robot_name = self.robot_name
            robot_names = list(robot_tasks.keys())
            for name in robot_names:
                if self.robot_name in name:
                    robot_name = name
                    break

            robot_task = robot_tasks.get(robot_name, "").strip()
            self.get_logger().debug(f"{self.robot_name} task fetched [1]")

            if not robot_task:
                robot_task = robot_tasks.get(self.robot_name+'_task', "").strip()
                self.get_logger().debug(f"{self.robot_name+'_task'} task fetched [2]")

            if not robot_task:
                robot_task = robot_tasks.get(self.robot_name+'_tasks', "").strip()
                self.get_logger().debug(f"{self.robot_name+'_tasks'} task fetched [3]")

            if not robot_task:
                robot_task = f"No {self.robot_name} task found."
                self.get_logger().debug(robot_task)
                return
            
            elif "stop" in robot_task.lower():
                self.get_logger().info(f"{self.robot_name} task: {robot_task}")
                self.robot_task_interrupted(robot_task)
                self.stop_tasks()
                msg.data = f'{self.robot_name.capitalize()} (status): STOP TASKS COMPLETED'
                self.pub_task_status.publish(msg)
                return

            self.robot_task = robot_task
            
            self.get_logger().info(f"{self.robot_name} task: {robot_task}")
            
            self.get_logger().info("Robot Task in Progress ..")
            self.robot_task_in_progress(robot_task)

            # ========== RESET CANCELLATION BEFORE NEW TASK ==========
            self.reset_cancellation()
            # =========================================================

            self.get_logger().info("Executing Robot Task ..")
            try:
                self.execute_task(robot_task)
                self.get_logger().info(f'On Task Json Task executed')
            except TaskCancelledException:
                self.get_logger().warn("Task execution was cancelled")
                self.robot_task_interrupted(robot_task)

            # status.data = f'{self.robot_name}: received {len(tasks) if isinstance(tasks, list) else 1} task set(s)'
        except json.JSONDecodeError:
            self.get_logger().warn(
                f'Received /task_manager/tasks_json with invalid JSON; raw: {msg.data}'
            )
            # status.data = f'{self.robot_name}: received invalid tasks JSON'
        # self.pub_task_status.publish(status)

    def on_chat_output(self, msg: String) -> None:
        """Handle raw chat output and save it to the history file."""
        self.get_logger().debug(f'Received /chat/output: {msg.data}')

        timestamp = self.current_time
        parts = msg.data.split("|", 1)

        if len(parts) == 2:
            role, content = parts[0].strip(), parts[1].strip()
            self.chat_entry = f"[Time: {timestamp}] {role.capitalize()}: {content}"
        else:
            role, content = "Task Manager", msg.data
            self.chat_entry = f"[Time: {timestamp}] {role}:\n{content}"

        with open(self.history_file, "a") as file:
            file.write(self.chat_entry + "\n")

    # def on_chat_history(self, msg: String) -> None:
    #     """Handle chat history stream."""
    #     # self.get_logger().debug(f'Received /chat/history len={len(msg.data)}')

    def on_chat_task_status(self, msg: String) -> None:
        """Observe task status changes (external)."""
        self.get_logger().debug(f'Observed /chat/task_status: {msg.data}')
        with open(self.history_file, "a") as file:
            file.write(msg.data + "\n")

    def on_current_time(self, msg: String) -> None:
        """Update current time from /current_time topic."""
        self.get_logger().debug(f'Received /current_time: {msg.data}')
        self.current_time = msg.data

    # -------------------- Helpers (optional) --------------------

    def update_robot_state(self, incoming: dict, robot_name: str | None = None) -> None:
        """
        Merge a partial robot-state update into self.robot_states.

        - self.robot_states may be:
            * None
            * a dict without "robot_states" (e.g., only meta fields like date)
            * a dict that already has "robot_states"
            * (optionally) a bare robot-states dict from older code; we wrap it.

        - Ensures top-level "robot_states" key exists.
        - Ensures per-robot dict exists.
        - Adds new keys and updates changed ones.
        """

        # Default robot_name to this node's robot name if not given
        if robot_name is None:
            robot_name = self.robot_name

        if self.robot_states is None:
            self.robot_states = {}
            self.get_logger().info("Created top-level robot_states container dict")

        if "robot_states" not in self.robot_states:
            if all(isinstance(v, dict) for v in self.robot_states.values()) and len(self.robot_states) > 0:
                # Wrap existing as robot_states
                self.robot_states = {"robot_states": self.robot_states}
                self.get_logger().debug("Wrapped existing dict under 'robot_states'")
            else:
                # Start fresh robot_states dict, keep other meta fields as-is
                self.robot_states["robot_states"] = {}
                self.get_logger().info("Created 'robot_states' key in top-level dict")

        robots_dict = self.robot_states["robot_states"]

        # 3) Ensure the robot entry exists
        if robot_name not in robots_dict or not isinstance(robots_dict[robot_name], dict):
            robots_dict[robot_name] = {}
            self.get_logger().info(f"Created new robot entry '{robot_name}'")

        saved_state = robots_dict[robot_name]

        # 4) Merge incoming keys into that robot's state
        for key, new_val in incoming.items():
            if key not in saved_state:
                saved_state[key] = new_val
                self.get_logger().debug(f"{robot_name}: Added '{key}' = '{new_val}'")
            else:
                old_val = saved_state[key]
                if old_val != new_val:
                    saved_state[key] = new_val
                    self.get_logger().debug(f"{robot_name}: Updated '{key}' from '{old_val}' → '{new_val}'")

        # 5) Publish the updated robot_states
        msg = String()
        msg.data = json.dumps(self.robot_states)
        self.pub_robot_states.publish(msg)

        # 6) Log the updated root structure (just the robot_states part to keep it readable)
        self.get_logger().debug(f"robot_states now: {self.robot_states['robot_states']}")

        return
    
    # -------------------- Task Status Methods --------------------

    def robot_task_in_progress(self, robot_task) -> None:
        # self.task_completed = False
        # self.task_in_progress = True
        # self.task_interrupted = False
        if not robot_task:
            robot_task = self.robot_task

        task_in_progress_msg = f"{self.robot_name.capitalize()} (status) : {robot_task} : TASK IN PROGRESS"
        self.get_logger().info(task_in_progress_msg)
        self.robot_task_status_update(task_in_progress_msg)
        return

    def robot_task_completed(self, robot_task) -> None:
        # self.task_completed = True
        # self.task_in_progress = False
        # self.task_interrupted = False
        if not robot_task:
            robot_task = self.robot_task

        task_completed_msg = f"{self.robot_name.capitalize()} (status) : {robot_task} : TASK COMPLETED"
        self.get_logger().info(task_completed_msg)
        self.robot_task_status_update(task_completed_msg)
        return

    def robot_task_interrupted(self, robot_task) -> None:
        # self.task_completed = False
        # self.task_in_progress = False
        # self.task_interrupted = True
        if not robot_task:
            robot_task = self.robot_task

        task_interrupted_msg = f"{self.robot_name.capitalize()} (status) : {robot_task} : TASK INTERRUPTED"
        self.get_logger().info(task_interrupted_msg)
        self.robot_task_status_update(task_interrupted_msg)
        return

    def robot_has_no_current_task(self) -> None:
        # self.task_completed = False
        # self.task_in_progress = False
        # self.task_interrupted = False
        
        robot_task = ""
        no_task_msg = f"{self.robot_name.capitalize()} (status) : {robot_task} : NO CURRENT TASK"
        self.get_logger().info(no_task_msg)
        self.robot_task_status_update(no_task_msg)
        return

    def robot_task_status_update(self, status_msg: str) -> None:
        msg = String()
        msg.data = status_msg

        # if self.task_in_progress:
        #     msg.data = self.task_in_progress_msg
        # elif self.task_completed:
        #     msg.data = self.task_completed_msg
        # elif self.task_interrupted:
        #     msg.data = self.task_interrupted_msg
        # else:
        #     msg.data = f"{self.robot_name.capitalize()} (status) : No current task."

        self.pub_robot_task.publish(msg)

        try:
            with open(self.robot_task_history, "a") as file:
                file.write(f"{status_msg}\n")
        except FileNotFoundError as e:
            self.get_logger().warn(f"Robot Task history file not found: {e}")

        self.get_logger().info("Robot task status updated... ")
        return

    def tasks_completed(self, task) -> None:
        msg = String()
        msg.data = f"{self.robot_name.capitalize()} (status): ALL TASKS COMPLETED"
        self.pub_task_status.publish(msg)
        return

    def stop_tasks(self) -> None:
        """Stop all robot tasks by setting cancellation flag."""
        self.get_logger().info("Stopping all robot tasks...")

        # ========== SET CANCELLATION FLAG ==========
        self._task_cancelled = True
        # ===========================================

        msg = Bool()
        msg.data = True
        self._cancel_goto_pub.publish(msg)

        self.get_logger().info("Cancel request sent to /goto/cancel")
        self.get_logger().info("Cancel request sent to /find/cancel")
        self.get_logger().info("All tasks of the robot have been stopped.")

    # -------------------- Helper Methods --------------------

    def read_chat_history(self) -> str:
        """Read the entire chat history from the persistent file."""
        self.get_logger().debug(f"Attempting to read chat history from: {self.history_file}")

        if not os.path.exists(self.history_file):
            self.get_logger().debug(f"Chat history file not found: {self.history_file}")
            return "No previous chat history."

        if not os.path.isfile(self.history_file):
            self.get_logger().warn(f"Chat history path exists but is not a file: {self.history_file}")
            return "No previous chat history."

        try:
            with open(self.history_file, "r", encoding="utf-8") as file:
                history = file.read().strip()

            if not history:
                self.get_logger().debug("Chat history file is empty.")
                return "No previous chat history."

            self.get_logger().info("Successfully loaded chat history.")
            return history

        except PermissionError:
            self.get_logger().error(f"Permission denied when reading chat history file: {self.history_file}")
            return "Error: Unable to read chat history (permission denied)."
        except OSError as e:
            self.get_logger().error(f"OS error while reading chat history file: {e}")
            return "Error: Failed to read chat history due to system issue."
        except Exception as e:
            self.get_logger().error(f"Unexpected error reading chat history: {type(e).__name__}: {e}")
            return "Error: Failed to load chat history."

    def generate_action_prompt(self, prompt: str, task: str) -> str:
        """Generate action prompt for LLM."""
        self.get_logger().info(f"Generating code...")

        try:
            response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {'role': 'system', 'content': prompt},
                    {'role': 'user', 'content': task}
                ],
                max_tokens=500,
                temperature=0.5,
            )
            self.get_logger().debug(f"LLM Response: {response}")

            raw = response.choices[0].message.content.strip()
            self.get_logger().info(f"Content: {raw}")
            
            if "```python" in raw:
                parts = raw.split("```python")
                explaination = parts[0].strip()
                code = parts[1].split("```")[0].strip()
                return code, explaination

            return "", ""
            
        except openai.Timeout:
            self.get_logger().error("OpenAI request TIMED OUT (no response in time)")
        except openai.AuthenticationError:
            self.get_logger().error("OpenAI authentication failed — check your API key!")
        except openai.RateLimitError:
            self.get_logger().error("OpenAI rate limit hit — slow down or upgrade plan")
        except openai.APIError as e:
            self.get_logger().error("OpenAI API error: %s", e)
        except Exception as e:
            self.get_logger().error("Unexpected error in LLM call: %s: %s", type(e).__name__, e)

        self.get_logger().warn("Returning (None, None) due to LLM failure")
        return None, None

    def execute_task(self, task: str) -> None:
        """Execute the given robot task using LLM decision-making."""
        chat_history = self.read_chat_history()

        try:
            self.get_logger().info("Building system action messages")
            available_actions = "\n".join(
                [f"Function Name: {opt.name} \nFunction Description: {opt.description} (e.g., {opt.example_code})" 
                 for opt in option_list]
            )

            self.get_logger().debug(f"Action message: {available_actions}")
            self.get_logger().info("Building system messages")
            
            prompt = (
                f"You are a robot control system controlling a {ROBOT_TYPE} named '{self.robot_name}'. "
                "You must generate python code to perform the task. "
                "Based on the given task, generate code using available actions."
                f"Recent Tasks (History): {chat_history} "
                f"Current States of All Robots: {self.robot_states} "
                f"Available Actions: {available_actions} "
                "Using the class reference name same as the example is important. "
                "Use the name 'node' to refer to the RobotLLMNode instance. "
                "Task Specific Rules: "
                "   'food1' is the name of the food from stall 1 "
                "   'food2' is the name of the food from stall 2 "
                "   'food3' is the name of the food from stall 3 "
                "   Tables 1, 2, 3, and 4 are present in the restaurant environment. "
                "   remember this name to pick the food from stall or table "
            )

            self.get_logger().debug(f"Prompt message: {prompt}")

        except Exception as e:
            self.get_logger().warning(f"{e}")

        code, explanation = self.generate_action_prompt(prompt, task)
        
        if code:
            self.get_logger().info(f"Generated Code:\n{code}")
            if explanation:
                self.get_logger().info(f"Explanation:\n{explanation}")

            self.get_logger().info("Calling execute_python_code...")
            execute_python_code(code, node=self)

            self.get_logger().info("Robot Task Completed.")
            self.robot_task_completed(task)
            self.tasks_completed(task)
        else:
            self.get_logger().error("Failed to generate valid code for the task.")
            self.robot_task_interrupted(task)

    def goto_service(self, x: float, y: float, z: float, yaw_deg: float) -> bool:
        """Call Drone GoTo service and wait (non-blocking, no nested spin)."""
        # ========== CHECK CANCELLATION ==========
        self.check_cancelled()
        # ========================================

        self.get_logger().info(f"Sending drone goto goal: x={x}, y={y}, z={z}, yaw={yaw_deg}°")

        if not self._goto_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("GotoPoseDrone service NOT available!")
            return False

        req = GotoPoseDrone.Request()
        req.x = x
        req.y = y
        req.z = z
        req.yaw_deg = yaw_deg

        future = self._goto_client.call_async(req)
        self.get_logger().info("Waiting for drone service response...")

        deadline = time.time() + 1000000.0
        while rclpy.ok() and not future.done():
            # ========== CHECK CANCELLATION IN LOOP ==========
            self.check_cancelled()
            # ================================================
            time.sleep(0.05)
            if time.time() > deadline:
                self.get_logger().error("Drone goto service call TIMED OUT.")
                return False

        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Drone goto service call failed: {e}")
            return False

        if not res.accepted:
            self.get_logger().warn(f"Drone goto request was NOT accepted: {res.message}")
            return False

        if res.success:
            self.get_logger().info(f"Drone goto SUCCESS: {res.message}")
            return True
        else:
            self.get_logger().warn(f"Drone goto FAILED: {res.message}")
            return False

    def query_callback(self, prompt: str) -> bool:
        # ========== CHECK CANCELLATION ==========
        self.check_cancelled()
        # ========================================

        if self.latest_image_b64 is None:
            self.get_logger().warn("No image received yet. Cannot query VLM.")
            return False

        self.get_logger().info("Sending image to VLM...")

        self.system_prompt = (
            "The table numbers are from left to right 1 to 4. "
            "The stall numbers are from left to right 1 to 3. "
            "You are a vision-based event detection assistant for a Drone. "
            "Your job is to analyze the image from the drone's bottom camera and answer the user's questions based on the visual content. IN SHORT, CONCISE ANSWERS ONLY. "
        )

        try:
            response = self.Visionclient.chat.completions.create(
                model="gpt-4.1",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{self.latest_image_b64}"
                                }
                            }
                        ]
                    }
                ]
            )

            answer = response.choices[0].message.content

            self.get_logger().info(f"VLM Answer: {answer}")
            answer = f"Drone (msg) | {answer}"
            self.pub_input_msg.publish(String(data=answer))
            return True

        except Exception as e:
            self.get_logger().error(f"OpenAI VLM request failed: {e}")
            return False

    # —————————————————————— LLM FUNCTIONS ——————————————————————

    def describe_screen(self, prompt: str = "What is in front of the drone?"):
        """Describe screen with cancellation support and state updates."""
        self.get_logger().info(f'Describing screen with prompt: "{prompt}"...')
        
        # Update state: Task started
        self.update_robot_state({
            "current_task": "describe_screen",
            "task_status": "in_progress",
            "vision_query": prompt,
            "query_stage": "started"
        })
        
        # Check if image available
        if self.latest_image_b64 is None:
            self.get_logger().warn("No image received yet. Cannot query VLM.")
            self.update_robot_state({
                "task_status": "failed",
                "failure_reason": "no_image_available",
                "query_stage": "failed"
            })
            self.robot_task_interrupted(f"describe screen with prompt: {prompt}")
            return
        
        # Update state: Processing image with VLM
        self.update_robot_state({
            "task_status": "querying_vlm",
            "query_stage": "processing_image",
            "image_available": True
        })

        success = self.query_callback(prompt)
        
        if success:
            self.get_logger().info(f"Screen description completed.")
            
            # Update state: Task completed
            self.update_robot_state({
                "task_status": "completed",
                "query_stage": "completed",
                "last_query": prompt,
                "query_successful": True,
                "completion_timestamp": time.time()
            })
            
            self.robot_task_completed(f"describe screen with prompt: {prompt}")
        else:
            self.get_logger().error("Failed to get VLM response.")
            
            # Update state: Task failed
            self.update_robot_state({
                "task_status": "failed",
                "query_stage": "vlm_error",
                "failure_reason": "vlm_query_failed",
                "query_successful": False
            })
            
            self.robot_task_interrupted(f"describe screen with prompt: {prompt}")
        return

    def hover(self, x: float = 0.0, y: float = 0.0, z: float = 2.0, yaw_deg: float = 0.0):
        """Hover with cancellation support and state updates."""
        self.get_logger().info(f"Hovering at position x={x}, y={y}, z={z}, yaw={yaw_deg}...")
        
        # Update state: Task started
        self.update_robot_state({
            "current_task": "hover",
            "task_status": "in_progress",
            "hover_stage": "started",
            "target_position": {"x": x, "y": y, "z": z, "yaw": yaw_deg}
        })
        
        # Update state: Moving to hover position
        self.update_robot_state({
            "task_status": "moving_to_position",
            "hover_stage": "navigating",
            "target_coords": {"x": x, "y": y, "z": z, "yaw_deg": yaw_deg}
        })

        success = self.goto_service(x=x, y=y, z=z, yaw_deg=yaw_deg)
        
        if success:
            self.get_logger().info(f"Hovering at position x={x}, y={y}, z={z} completed.")
            
            # Update state: Hovering successfully
            self.update_robot_state({
                "task_status": "completed",
                "hover_stage": "hovering",
                "current_position": {"x": x, "y": y, "z": z, "yaw": yaw_deg},
                "hover_successful": True,
                "hovering_at": f"({x}, {y}, {z})",
                "completion_timestamp": time.time()
            })
            
            self.robot_task_completed(f"hover at x={x}, y={y}, z={z}")
        else:
            self.get_logger().error(f"Failed to reach hover position.")
            
            # Update state: Hover failed
            self.update_robot_state({
                "task_status": "failed",
                "hover_stage": "navigation_failed",
                "failure_reason": "could_not_reach_target_position",
                "hover_successful": False,
                "attempted_position": {"x": x, "y": y, "z": z, "yaw": yaw_deg}
            })
            
            self.robot_task_interrupted(f"hover at x={x}, y={y}, z={z}")
        return

def execute_python_code(code: str, node=None):
    """Execute generated Python code safely with cancellation support."""
    print("Inside the execute python code function")

    if node is None:
        node = RobotLLMNode.get_instance()
        if node is None:
            print("CRITICAL: Could not get node instance!")
            return

    node.get_logger().info(f"Executing generated Python code: {code}")

    try:
        # ========== PASS TaskCancelledException TO EXEC CONTEXT ==========
        exec(code, {"__builtins__": {}}, {
            "node": node,
            "TaskCancelledException": TaskCancelledException
        })
        # =================================================================
        node.get_logger().info("Code executed successfully")
    except TaskCancelledException:
        node.get_logger().warn("Code execution cancelled")
        raise  # Re-raise so execute_task can handle it
    except TypeError as e:
        pass
    except Exception as e:
        node.get_logger().error("Failed to execute generated code: %s", e)


def main(args=None):
    rclpy.init(args=args)
    node = RobotLLMNode()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()