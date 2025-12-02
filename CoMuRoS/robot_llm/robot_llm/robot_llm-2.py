#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time

import openai
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dataclasses import dataclass
import os
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup
)
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from pick_object_interface.action import PickObject
from action_msgs.msg import GoalStatus

from concurrent.futures import Future
from typing import Optional

ROBOT_TYPE = 'Robotic Arm'

# Define available actions
@dataclass
class TestOption:
    name: str
    id: int
    description: str
    example_code: str

option_list = [
    TestOption(
        name="Pick green object",
        id=0,
        description="This is used pick or show the green object or gear",
        example_code="node.pick_green_object()"
    ),
    TestOption(
        name="Pick brown object",
        id=1,
        description="This is used pick or show the brown object or gear",
        example_code="node.pick_brown_object()"
    ),
    TestOption(
        name="Pick grey object",
        id=2,
        description="This is used pick or show the grey object or gear",
        example_code="node.pick_grey_object()"
    )
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
        super().__init__('robot_llm_node')

        # ---- Parameters ----
        self.declare_parameter('robot_name', 'lerobot1')
        self.robot_name: str = self.get_parameter('robot_name').value
        robot_task_topic = f'{self.robot_name}_task_status'

        RobotLLMNode._instance = self

        self.current_time = f"Hours: {00}, Minutes: {10}, Seconds: {00}"
        self.robot_task = ""
        self.robot_states = {}


        self._pick_future: Optional[Future] = None
        self._current_goal_handle = None           # THIS WAS MISSING!

        # GROUPS
        self.single_group = MutuallyExclusiveCallbackGroup()   # For single-threaded/exclusive ops, like chat
        self.seq_group = MutuallyExclusiveCallbackGroup()      # New: For sequential execution of robot_states and current_time
        self.multi_group = ReentrantCallbackGroup()            # For parallel ops, consolidated into one for simplicity

        # ---- Publishers ----
        self.pub_task_status = self.create_publisher(String, '/chat/task_status', 10)
        self.pub_robot_states = self.create_publisher(String, '/robot_states', 10)
        self.pub_robot_task = self.create_publisher(String, robot_task_topic, 10)

        self._action_client = ActionClient(self, PickObject, 'pick_object')

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
        # self.sub_chat_task_status = self.create_subscription(
        #     String, '/chat/task_status', self.on_chat_task_status, 10
        # )

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

        package_name = "robot_llm"
        directry = "data"
        package_path = get_package_share_directory(package_name)

        script_name = "chat_history.txt"
        self.history_file = os.path.join(package_path, directry, script_name)

        script_name = 'robot_task_history.txt'
        self.robot_task_history = os.path.join(package_path, directry, script_name)

        self.clear_files()
        self.robot_has_no_current_task()

    @classmethod
    def get_instance(cls):
        """Safely get or create the singleton node."""

        if cls._instance is None:
            raise RuntimeError("RobotLLMNode has not been created yet! Did you run the node?")
        return cls._instance

    def clear_files(self) -> None:
        """Clear the chat history file on startup."""

        if os.path.exists(self.history_file):
            with open(self.history_file, "w") as file:
                file.write("")  # Clear the file contents
            self.get_logger().info("Cleared chat history file on startup.")
        else:
            self.get_logger().warn(f"Chat history file not found: {self.history_file}")
        
        if os.path.exists(self.robot_task_history):
            with open(self.robot_task_history, "w") as file:
                file.write("")  # Clear the files
            self.get_logger().info("Cleared the robot task history file on startup.")
        else:
            self.get_logger().warn(f"Robot Task history file not found: {self.robot_task_history}")


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
            if not robot_task:
                robot_task = f"No {self.robot_name} task found."
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

            self.get_logger().info("Excuting Robot Task ..")
            self.execute_task(robot_task)
            self.get_logger().info(f'On Task Json Task executed')

            # status.data = f'{self.robot_name}: received {len(tasks) if isinstance(tasks, list) else 1} task set(s)'
        except json.JSONDecodeError:
            self.get_logger().warn(
                f'Received /task_manager/tasks_json with invalid JSON; raw: {msg.data}'
            )
            # status.data = f'{self.robot_name}: received invalid tasks JSON'
        # self.pub_task_status.publish(status)

    def on_chat_output(self, msg: String) -> None:
        """Handle raw chat output and savs it to the history file."""
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

    # def on_chat_task_status(self, msg: String) -> None:
    #     """Observe task status changes (external)."""
    #     self.get_logger().debug(f'Observed /chat/task_status: {msg.data}')

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

    def robot_task_in_progress(self, robot_task) -> None:
        # self.task_completed = False
        # self.task_in_progress = True
        # self.task_interrupted = False

        if not robot_task :
            robot_task =self.robot_task

        task_in_progress_msg = f"{self.robot_name.capitalize()} (status) : {robot_task} : TASK IN PROGRESS"
        self.get_logger().info(task_in_progress_msg)

        self.robot_task_status_update(task_in_progress_msg)

        return

    def robot_task_completed(self, robot_task) -> None:
        # self.task_completed = True
        # self.task_in_progress = False
        # self.task_interrupted = False

        if not robot_task :
            robot_task =self.robot_task

        task_completed_msg = f"{self.robot_name.capitalize()} (status) : {robot_task} : TASK COMPLETED"
        self.get_logger().info(task_completed_msg)

        self.robot_task_status_update(task_completed_msg)

        return

    def robot_task_interrupted(self, robot_task) -> None:
        # self.task_completed = False
        # self.task_in_progress = False
        # self.task_interrupted = True

        if not robot_task :
            robot_task =self.robot_task

        task_interrupted_msg = f"{self.robot_name.capitalize()} (status) : {robot_task} : TASK INTERRUPTED"
        self.get_logger().info(task_interrupted_msg)

        self.robot_task_status_update(task_interrupted_msg)
        
        return

    def robot_has_no_current_task(self) -> None:
        # self.task_completed = False
        # self.task_in_progress = False
        # self.task_interrupted = False
        
        robot_task=""

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
        """Stop all robot tasks (stub function)."""
        self.get_logger().info("Stopping all robot tasks...")

        # self._cancel_requested = True

        # if self._current_goal_handle:
        #     self.get_logger().info("Canceling current pick action...")
        #     self._current_goal_handle.cancel_goal_async()  # ← REAL CANCEL

        self.cancel_current_pick()

        self.get_logger().info("All tasks of the robot has been stopped.")

    def read_chat_history(self) -> str:
        """
        Read the entire chat history from the persistent file.

        Returns:
            str: The full chat history as a string, or a user-friendly message
                if the file is missing or empty.
        """
        
        self.get_logger().debug(f"Attempting to read chat history from: {self.history_file}")

        if not os.path.exists(self.history_file):
            self.get_logger().debug(f"Chat history file not found: {self.history_file}")
            return "No previous chat history."

        if not os.path.isfile(self.history_file):
            self.get_logger().warn(f"Chat history path exists but is not a file: {self.history_file}")
            return "No previous chat history."

        try:
            # Use UTF-8 encoding explicitly and handle potential I/O errors
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
        """Generate action prompt for LLM (stub function)."""
        self.get_logger().info(f"Generating code...")

        try:
            response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        'role': 'system',
                        'content': prompt
                    },
                    {
                        'role': 'user',
                        'content': task
                    }
                ],
                max_tokens=500,
                temperature=0.5,
            )
            self.get_logger().info(f"LLM Response: {response}")

            raw = response.choices[0].message.content.strip()
            # code = response.choices[0].message['content'].strip()
            self.get_logger().info(f"Content: {raw}")
            if "```python" in raw:
                parts = raw.split("```python")
                explaination = parts[0].strip()
                code = parts[1].split("```")[0].strip()
                # self.get_logger().info(f"code: {code}")
                # self.get_logger().info(f"explaination: {explaination}")
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
                [f"Function Name: {opt.name} \nFunction Description: {opt.description} (e.g., {opt.example_code})" for opt in option_list]
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
                # f"Task to be performed: {task} "
            )

            self.get_logger().debug(f"Prompt message: {prompt}")

        except Exception as e:
            self.get_logger().warning(f"{e}")

        code, explanation = self.generate_action_prompt(prompt, task)
        
        if code:
            self.get_logger().info(f"Generated Code:\n{code}")
            if explanation:
                self.get_logger().info(f"Explanation:\n{explanation}")

            # Execute the generated code
            self.get_logger().info("Calling execute_python_code...")
            # execute_python_code(code)
            execute_python_code(code, node=self)

            self.get_logger().info("Robot Task Completed.")
            self.robot_task_completed(task)
            self.tasks_completed(task)
        else:
            self.get_logger().error("Failed to generate valid code for the task.")
            self.robot_task_interrupted(task) ## Needs to be handled better If it became an EVENT it could be better

    def pick_object(self, object_name: str = "red_gear") -> bool:
        """BLOCKS until pick finishes. Returns True/False. SAFE with MultiThreadedExecutor."""
        self.get_logger().info(f"Picking '{object_name}'...")

        # self.cancel_current_pick() # Cancel any previous goal first

        # Reject if already busy
        if self._pick_future is not None and not self._pick_future.done():
            self.get_logger().warn("Already picking — rejecting new pick")
            return False

        # Create future for this pick
        self._pick_future = Future()

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            self._pick_future.set_result(False)
            self._pick_future = None
            return False

        goal_msg = PickObject.Goal()
        goal_msg.object_name = object_name

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

        # BLOCK HERE — but it's safe because another thread will run the callback
        try:
            success = self._pick_future.result(timeout=60.0)  # waits up to 60s
            return success
        except TimeoutError:
            self.get_logger().error("Pick action timed out after 60s")
            self._pick_future = None
            return False

    # —————————————————————— CALLBACKS (run in executor's threads) ——————————————————————
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by server")
            if self._pick_future:
                self._pick_future.set_result(False)
                self._pick_future = None
            return

        # THIS LINE WAS MISSING — THIS IS THE ENTIRE FIX
        self._current_goal_handle = goal_handle
        self.get_logger().info("Goal accepted — handle saved for cancellation")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        try:
            result_msg = future.result()           # has .status and .result
        except Exception as e:
            self.get_logger().error(f"Result retrieval failed: {e}")
            if self._pick_future:
                self._pick_future.set_result(False)
                self._pick_future = None
            self._current_goal_handle = None
            return

        status = getattr(result_msg, "status", None)
        result = getattr(result_msg, "result", None)

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(getattr(result, "status_msg", "Succeeded"))
            success = True
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal was canceled by request.")
            success = False
        else:
            self.get_logger().warn(f"Goal finished with status={status}")
            success = False

        # Clear only after terminal result
        self._current_goal_handle = None

        if self._pick_future and not self._pick_future.done():
            self._pick_future.set_result(success)
        self._pick_future = None

    def feedback_callback(self, feedback_msg):
        try:
            data = json.loads(feedback_msg.feedback.json_state)
            self.update_robot_state(data)
        except json.JSONDecodeError:
            pass

    def cancel_current_pick(self):
        self.get_logger().info("In process of cancelling request")
        if self._current_goal_handle is None:
            self.get_logger().debug("No active goal to cancel")
            return

        self.get_logger().info("CANCELLING current pick action...")
        cancel_future = self._current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._on_cancel_done)

    def _on_cancel_done(self, future):
        """Called when the server responds to our cancel request."""
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"Cancel request failed: {e}")
            return

        # If the server accepted the cancel, it will push a terminal result soon.
        goals_canceling = getattr(resp, "goals_canceling", None)
        if goals_canceling and len(goals_canceling) > 0:
            self.get_logger().info("Cancel accepted; waiting for final result...")

            # Re-attach result callback defensively (handles races).
            try:
                if self._current_goal_handle is not None:
                    result_future = self._current_goal_handle.get_result_async()
                    result_future.add_done_callback(self._get_result_callback)
            except Exception as e:
                self.get_logger().error(f"Failed to reattach result callback after cancel: {e}")
        else:
            rc = getattr(resp, "return_code", None)
            # 0=OK (some distros), 1=REJECTED, 2=UNKNOWN_GOAL_ID, 3=GOAL_TERMINATED
            self.get_logger().warn(f"Cancel not accepted or too late (return_code={rc}).")
            # If the goal already terminated, _get_result_callback should run (or already ran).
            # If not, ensure we don't get stuck:
            if self._pick_future and not self._pick_future.done():
                self._pick_future.set_result(False)
                self._pick_future = None


    def _cancel_watchdog_once(self):
        # One-shot timer – cancel itself
        for t in list(self.timers):  # if you don't keep a handle, ignore this
            t.cancel()
        if self._current_goal_handle is not None:
            # still no result? log it
            self.get_logger().warn("Cancel watchdog fired: no result yet from server.")
        if self._pick_future and not self._pick_future.done():
            self._pick_future.set_result(False)
        self._pick_future = None

    # —————————————————————— YOUR LLM FUNCTIONS (now perfect) ——————————————————————
    def pick_green_object(self) -> bool:
        self.get_logger().info("Picking green object...")
        success = self.pick_object("green object")
        if success:
            self.robot_task_completed("pick green object")
        else:
            self.robot_task_interrupted("pick green object")
        return 

    def pick_brown_object(self) -> bool:
        self.get_logger().info("Picking brown object...")
        success = self.pick_object("brown object")
        if success:
            self.robot_task_completed("pick brown object")
        else:
            self.robot_task_interrupted("pick brown object")
        return 

    def pick_grey_object(self) -> bool:
        self.get_logger().info("Picking grey object...")
        success = self.pick_object("grey object")
        if success:
            self.robot_task_completed("pick grey object")
        else:
            self.robot_task_interrupted("pick grey object")
        return 
        

def execute_python_code(code: str, node=None):
    """
    Execute generated Python code safely.
    node: the actual running RobotLLMNode instance (pass it explicitly!)
    """

    print("Inside the execute python code function")

    if node is None:
        # Fallback — but you should never hit this
        node = RobotLLMNode.get_instance()
        if node is None:
            print("CRITICAL: Could not get node instance!")
            return
    
    node.get_logger().info(f"Executing generated Python code:{code}")
    # node.get_logger().debug("Code to execute:\n%s", code)

    # node.pick_brown_object()
    try:
        exec(code, {"__builtins__": {}}, {"node": node})
        node.get_logger().info("Code executed successfully")
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
