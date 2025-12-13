#!/usr/bin/env python3

import json
import time
import subprocess
import openai
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from dataclasses import dataclass
import os
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup
)
from ament_index_python.packages import get_package_share_directory

from robot_interface.srv import GotoPoseHolonomic


ROBOT_NAME   = 'delivery_bot'
ROBOT_TYPE   = "Differential Drive Robot"
NODE_NAME    = "delivery_bot_llm_node"
PACKAGE_NAME = "delivery_bot"


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
        name="DeliverFood",
        id=0,
        description="This is used to deliver food items from a stall to a table.",
        example_code="node.deliver_food(stall_number=1, table_number=3)"
    ),
    TestOption(
        name='ClearTable',
        id=1,
        description='This is used to clear the plates from table and drop them in the sink. From the history, check which food items were delivered to the table and clear only those food items. from the history check the name of the food which was kept at the table and send that also',
        example_code='node.clear_table(table_number=2, food_name="food1")'
    )
]

TableLocation = {
    1 : [0.0, -1.3, 0.0],
    2 : [3.0, -1.3, 0.0],
    3 : [6.0, -1.3, 0.0],
    4 : [9.0, -1.3, 0.0],
}

StallLocation = {
    1 : [0.0, 0.0, 0.0],
    2 : [4.0, 0.0, 0.0],
    3 : [8.0, 0.0, 0.0],
}

home_pose = [0.0, 0.0, 0.0]

sink_pose = [-1.85, -0.5, 0.0]

food = ['food1', 'food2', 'food3']

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

        # ========== CANCELLATION MECHANISM ==========
        self._task_cancelled = False  # Simple boolean flag
        # ============================================

        # GROUPS
        self.single_group = MutuallyExclusiveCallbackGroup()
        self.seq_group = MutuallyExclusiveCallbackGroup()
        self.multi_group = ReentrantCallbackGroup()

        # ---- Publishers ----
        self.pub_task_status = self.create_publisher(String, '/chat/task_status', 10)
        self.pub_robot_states = self.create_publisher(String, '/robot_states', 10)
        self.pub_robot_task = self.create_publisher(String, robot_task_topic, 10)

        self._goto_client = self.create_client(GotoPoseHolonomic, '/r2/goto_pose', callback_group=self.multi_group)
        self._cancel_goto_pub = self.create_publisher(Bool, '/r2/cancel_goto_pose_goal', 10)

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

        # 1) Ensure top-level container exists
        if self.robot_states is None:
            self.robot_states = {}
            self.get_logger().info("Created top-level robot_states container dict")

        # 2) Make sure we have a "robot_states" key at top-level
        if "robot_states" not in self.robot_states:
            # Heuristic: if current dict already *looks* like it only contains robots,
            # then wrap it under "robot_states" instead of losing it.
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
                self.get_logger().debug(f"code: {code}")
                self.get_logger().debug(f"explaination: {explaination}")
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
                "You can generate python code to perform actions. "
                "Based on the given task, you need to choose the appropriate action from the available options. "
                f"Recent Tasks (History): {chat_history} "
                f"Current States of All Robots: {self.robot_states} "
                f"Available Actions: {available_actions} "
                "Using the class reference name same as the example is important. "
                "Use the name 'node' to refer to the RobotLLMNode instance. "
                "'food1' is the name of the food from stall 1 "
                "'food2' is the name of the food from stall 2 "
                "'food3' is the name of the food from stall 3 "
                "remember this name to pick the food from stall or table "
                # "IMPORTANT: Add 'node.check_cancelled()' at the start of loops and between long operations. "
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

    def goto_service(self, x: float, y: float, yaw_deg: float) -> bool:
        """Call GoToPoseDiffDrive service (non-blocking, no nested spin)."""
        # ========== CHECK CANCELLATION ==========
        self.check_cancelled()
        # ========================================

        self.get_logger().info(f"Sending goto goal: x={x}, y={y}, yaw={yaw_deg}°")

        if not self._goto_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("GotoPoseDiffDrive service NOT available!")
            return False

        req = GotoPoseHolonomic.Request()
        req.x = x
        req.y = y
        req.yaw_deg = yaw_deg

        future = self._goto_client.call_async(req)
        self.get_logger().info("Waiting for goto service response...")

        deadline = time.time() + 900000.0
        while rclpy.ok() and not future.done():
            # ========== CHECK CANCELLATION IN LOOP ==========
            self.check_cancelled()
            # ================================================
            time.sleep(0.05)
            if time.time() > deadline:
                self.get_logger().error("GotoPoseDiffDrive service call TIMED OUT.")
                return False

        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return False

        if res.accepted:
            self.get_logger().info(f"Goto accepted: {res.message}")
            return True
        else:
            self.get_logger().warn(f"Goto rejected: {res.message}")
            return False

    def teleport(self, name, x, y, z):
        """Teleport object in simulation."""
        # ========== CHECK CANCELLATION ==========
        self.check_cancelled()
        # ========================================

        self.get_logger().info(f"Teleport Object: {name}")

        req = (
            f'name: "{name}", '
            f'position: {{x: {x}, y: {y}, z: {z}}}, '
            'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'
        )

        cmd = [
            'ign', 'service', '-s', '/world/food_court/set_pose',
            '--reqtype', 'ignition.msgs.Pose',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '1000',
            '--req', req
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True)

        self.get_logger().info(f"Teleport result: {result.stdout}")
        return result.returncode == 0

    # —————————————————————— LLM FUNCTIONS ——————————————————————
    
    def deliver_food(self, stall_number, table_number):
        """Deliver food with cancellation support."""
        self.get_logger().info('Delivering food ...')
        table_pose = TableLocation.get(table_number)
        stall_pose = StallLocation.get(stall_number)

        if table_pose is None:
            self.get_logger().error(f"Invalid table number: {table_number}")
            return 

        if stall_pose is None:
            self.get_logger().error(f"Invalid stall number: {stall_number}")
            return 

        sx, sy, syaw = stall_pose
        hx, hy, hyaw = table_pose
        home_pose_x, home_pose_y, home_pose_yaw = home_pose

        food_name = food[stall_number - 1]

        # ========== CHECK CANCELLATION ==========
        self.check_cancelled()
        # ========================================

        result = self.goto_service(sx, sy+0.5, syaw) 
        self.get_logger().info('Robot Near Stall')
        time.sleep(2.0)

        result = self.teleport(name=food_name, x=sx, y=sy+0.5, z=0.4)
        self.get_logger().info('Food Picked from Stall')
        time.sleep(2.0)

        result = self.goto_service(hx, hy, hyaw)
        self.get_logger().info('Robot Near Table')
        time.sleep(2.0)

        result = self.teleport(name=food_name, x=hx+0.1, y=hy-0.7, z=0.6)
        self.get_logger().info("Food Delivered to table")
        time.sleep(2.0)

        result = self.goto_service(home_pose_x, home_pose_y, home_pose_yaw)
        result = self.get_logger().info('Robot Went Home')

    def clear_table(self, table_number, food_name):
        """Clear table with cancellation support."""
        self.get_logger().info('Clearing Table ...')
        table_pose = TableLocation.get(table_number)

        self.get_logger().info(f'Clear the food item: {food_name} from table number: {table_number}')

        if table_pose is None:
            self.get_logger().error(f"Invalid table number: {table_number}")
            return 

        table_x, table_y, table_yaw = table_pose
        sink_x, sink_y, sink_yaw = sink_pose
        home_pose_x, home_pose_y, home_pose_yaw = home_pose

        # ========== CHECK CANCELLATION ==========
        self.check_cancelled()
        # ========================================

        result = self.goto_service(table_x, table_y, table_yaw)
        self.get_logger().info('Robot Near Table')
        time.sleep(2.0)

        result = self.teleport(name=food_name, x=table_x, y=table_y, z=0.4)
        self.get_logger().info('Food Picked from Table')
        time.sleep(2.0)

        result = self.goto_service(sink_x, sink_y, sink_yaw)
        self.get_logger().info('Robot Near Sink')
        time.sleep(2.0)

        result = self.teleport(name=food_name, x=-2.5, y=-0.5, z=0.6)
        self.get_logger().info('Food Dropped in Sink')  
        time.sleep(2.0)

        result = self.goto_service(home_pose_x, home_pose_y, home_pose_yaw)
        self.get_logger().info('Robot Went Home')


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