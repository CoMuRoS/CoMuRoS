#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from std_msgs.msg import String 
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
import re
import os
import json
import ast
from queue import Queue
# import requests
import openai
from openai import OpenAI
from google.genai import types
from ollama import chat, ChatResponse
from google import genai


openai.api_key = os.environ.get("OPENAI_API_KEY", "")
gemini_api_key = os.environ.get("GEMINI_API_KEY", "")
xai_api_key = os.environ.get("XAI_API_KEY") 


class TaskManager(Node):
    """
    A single node that:
     1) Subscribes to /chat/output to see the entire conversation (no truncation).
     2) On new Human commands, calls GPT to parse tasks for *only* the mentioned robots
        (others keep their existing tasks).
     3) On "Event:" lines, calls GPT again to do a full re-plan. 
        - We set all robots to 'stop what you are doing' first.
        - GPT can specify new tasks or even multi-step sequences.
     4) Publishes the final plan (robot tasks + optional sequence) as JSON to /task_manager/tasks_json
        and also as text to /chat/input so the GUI sees it.

    GPT Prompt Highlights:
     - Robots can help each other by pausing/resuming tasks.
     - GPT may provide multi-step 'Sequence:' lines. 
     - We parse and store them in self.sequence_of_tasks, then publish them in the JSON.
    """

    def __init__(self):
        super().__init__("task_manager")

        self.robot_state = None

        self.declare_parameter("config_file", "robot_config_assmble_help")
        cfg_file_name = self.get_parameter("config_file").get_parameter_value().string_value
        cfg_file = cfg_file_name + '.json'
        self.config_file = cfg_file
        print(f"Config file : {cfg_file}")


        self.declare_parameter("model", 4)
        self.model = self.get_parameter("model").get_parameter_value().integer_value
        self.get_logger().info(f"model number: {self.model}")
        self.model_name = ""

        package_share = get_package_share_directory("chatty")

        cfg_path = os.path.join(package_share, "config", cfg_file)

        with open(cfg_path, 'r') as f:
            self.robot_config = json.load(f)

        history_current_file = "chat_history_current.txt"
        self.history_current = os.path.join(package_share, "data", history_current_file)

        self.robot_state_pub = self.create_publisher(String,'/robot_states',10)
        self.set_robot_states()
        # self.system_prompt = self.build_system_prompt()
        self.system_prompt = None
        # print("SYSTEM PROMPT : \n")
        # print(self.system_prompt)
        
        # Optional multi-step or overall plan
        self.sequence_of_tasks = ""
        self.task_status = "INIT"
        
        self.conversation_log = [] # Full chat log

        self.multirobot_completion_status = False
        self.master_status = False 
        self.multirobot_list = []
        self.singlerobot_list = []

        self.team_list = []
        self.single_tasks_dict = {}
        self.current_time = f"Hours: {00}, Minutes: {00}, Seconds: {00}"

        self.subscription = self.create_subscription(String, "/chat/output", self.on_chat_output, 10)
        self.input_sub = self.create_subscription(String, "/chat/input" , self.on_chat_input, 10)
        self.status_pub = self.create_subscription(String, "/chat/task_status", self.on_status_callback,10)   
        self.robot_state_sub = self.create_subscription(String, "/robot_states", self.on_robot_state_callback,10)
        self.time_sub = self.create_subscription(String, "/current_time", self.on_time_callback, 10)

        self.create_dynamic_subscribers()

        self.pub_tasks_json = self.create_publisher(String, "/task_manager/tasks_json", 10)
        self.pub_chat_input = self.create_publisher(String, "/chat/input", 10)

        # self.publisher_ = self.create_publisher(Bool, '/close_tasks', 10)

        self.get_logger().info("[TaskManager] Node initialized.")

    def set_robot_states(self):
        for name in self.robot_config["robot_names"]:
            setattr(self, f"{name}_state", "Starting position")
        robot_states = self.robot_config['robot_states']
        robot_states = {"robot_states": robot_states}
        msg = String()
        msg.data = json.dumps(robot_states)
        self.robot_state_pub.publish(msg)
        self.get_logger().info(f"[TaskManager] Initialized robot states for: {self.robot_config['robot_states']}")

    def create_dynamic_subscribers(self):
        print("Creating dynamic subscribers")
        for robot_name in self.robot_config["robot_names"]:
            topic = f"/{robot_name}_task_status"
            callback = self.generate_callback(robot_name)
            
            # Dynamically create a subscriber attribute like self.quadruped_state_sub
            sub_attr_name = f"{robot_name}_state_sub"
            setattr(self, sub_attr_name,
                    self.create_subscription(String, topic, callback, 10))

            self.get_logger().info(f"Created subscriber for {robot_name} on topic {topic}\n")

    def generate_callback(self, robot_name):
        def callback(msg):
            # Dynamically store the status as self.<robot_name>_status
            setattr(self, f"{robot_name}_state", msg.data)

            # Optionally log only if the status changes
            prev_status = getattr(self, f"{robot_name}_last_logged", None)
            if msg.data != prev_status:
                self.get_logger().info(f"[{robot_name.upper()} STATUS]: {msg.data}")
                setattr(self, f"{robot_name}_last_logged", msg.data)

        return callback

    def on_time_callback(self, msg:String) :
        # self.get_logger().info(f"[TaskManager] Time Callback : {msg.data}")
        self.current_time = msg.data


    def load_history(self):
        """Loads previous chat history from file."""

        if os.path.exists(self.history_current):
            try:
                with open(self.history_current, "r") as file:
                    self.chat_log = file.read().splitlines()
                # self.get_logger().info(f"[TaskManager] Loaded {len(self.chat_log)} previous messages.")
            except Exception as e:
                self.get_logger().error(f"[TaskManager] Failed to load history: {e}")


    # --------------------------------------------------------------------------
    #  BUILD SYSTEM PROMPT
    # --------------------------------------------------------------------------
    def build_system_prompt(self):

        self.load_history()

        cfg = self.robot_config
        try :

            # --- Dynamic: Robot Capabilities ---
            capability = "1. **Robot Capabilities:**\n"
            for name in cfg['robot_names']:
                title = name.replace('_', ' ').title()
                desc  = cfg['robot_capabilities'].get(name, '').strip()
                capability += f"   - **{title}**: {desc}\n"


            # --- Dynamic: Robot Morphology ---
            morphology = "2. **Robot Morphology and Location:**\n"
            for name in cfg['robot_names']:
                title = name.replace('_', ' ').title()
                desc  = cfg['robot_morphology'].get(name, '').strip()
                morphology += f"   - **{title}**:{desc}\n"


            # --- Dynamic: Task‐Specific & Replanning Rules ---
            rules = "3. **Task Allocation Specific Rules:**\n"
            for rule in cfg['task_specific_rules']:
                rules += f"   - {rule}\n"

            replanning_rules = ""
            for rule in cfg['task_replanning_rules']:
                replanning_rules += f"   - {rule}\n"

            # --- Assemble entire system prompt ---
            header = (
                "You are a Task Manager AI for a team of robots. "
                "Your job is to **ensure that all assigned valid tasks are completed efficiently** "
                "by distributing work among the robots based on their capabilities.\n\n"
                "### Task Execution Principles:\n"
            )

            # --- Dynamic: Robot Task Status ---
            status_parts = []
            for name in cfg['robot_names']:
                try:
                    val = getattr(self, f"{name}_state")
                except AttributeError:
                    val = "UNKNOWN"
                status_parts.append(f"{name}: {val}\n")
            status = "   ".join(status_parts)

            self.robot_tasks = {name: "" for name in cfg['robot_names']}

            # --- Static remainder of your prompt (replanning, output format, etc.) ---
            static = (
            "If conversation is general and does not involve any task assignment just reply politely and shortest possible answer.\n"
            "0. **Task Replanning Rules:**\n"
            "     - If the event directly affects or changes the state of an object, location, or condition involved in an active task, it is RELEVANT.\n"
            "     - Events can be triggered by robots, humans, or external factors.\n"
            "     - IMPORTANT : When an event is RELEVANT, you may choose one of the approaches based on the replanning : Independent Task(for tasks that can be done parallely) or Plan(for tasks that can only be done sequentially) or Mixed Tasks(for tasks that can not be done only using Plan or Independent), \n\n"
            
            "     0 a. If Plan option then you MUST begin the Plan by issuing `STOP` to ALL robots INVOLVED, including those with IN PROGRESS or COMPLETED tasks.\n"
            "      - STOP is mandatory before assigning any new tasks or asking them to assist others. Follow the format for STOP as :"
                        "\n  robot1_name : STOP "
                        "\n  team1_name : STOP "
                        "\n  robot2_name : STOP "
            "      - When asking the help from the user do not any task other than stop to the robots\n"
            "      - The Plan must ALWAYS begin with STOP commands for all involved robots. These STOPs must be the first steps in the numbered plan — one STOP per robot.\n"
            "      - After STOPs are issued, continue the plan with the next logical steps.\n"
            "      - STOP steps must always come FIRST in sequence of ALL ROBOTS INVOLVED , without mixing or placing other tasks before them.\n"
            "      - STOP is not necessary to a robot when there was no task assinged to that robot earlier.\n"
            "      - When the task is difficult to continue because of Robot Morphology or Robot Capabilities then you can ask for assistace from the human or user"
            "           Example: \n"
            "               1. robot1_name: STOP\n"
            "               3. team 1: STOP\n"
            "               2. robot2_name: STOP\n"
            "               4. user: task description complete task which was failed\n"
            "      - While asking user to help only assign STOP tasks to other robots whihch were part of the plan, do not assign any new task to other robots or team of robots.\n"
            "      - You will recieve an event message in which it will be said that user or human has completed the task, after this message now replan the task after human help how to continue the task if the task is not fully completed \n"
            "      - Do NOT shift task execution to the 'Independent Tasks' section just to separate STOP commands. The Plan block must handle the entire sequence.\n\n"

            "     0 b.  If Independent Tasks: Follow Task Output Format Section 2 given below - \n"
            "      - If any event is RELEVANT to the task consider reassigning task by giving the initial task as stop to ALL robots followed by step by step logical plan (Check Robot's capabilities & Robot Morphology section mentioned above).\n"
            "      - If any event is IRRELEVANT to the task ignore it and dont reassign any tasks , dont give any Plan or Independent Tasks.\n"
            "      - Replanning should be such that all robots tasks are complete eventually including the tasks that were in progress before the event. In case of failure let robots help each other based on their capabilities.\n"            
            "      - It is Important to RESUME TASKS which were in progress: A robot who is already doing a task has been assigned with new task then ,it is COMPULSORY to complete it's prevous incomplete or unfinished. \n"
            "      - But DO NOT RESUME TASKS if the status of the task is TASKS COMPLETE at the time of event. At the end ensure all tasks of all robots are completed. \n"
            "      - DO NOT RESUME any task marked as TASK COMPLETE in the current task status section.\n"
            "      - Refer to current task status given below to determine whether to resume previous tasks or not after replanning.\n The current task status are -\n"
            f"      {status}\n\n"

            " - ONLY resume tasks for robots whose status is marked as IN PROGRESS at the time of the event. Do NOT resume for any robot whose status is TASKS COMPLETE. These robots or team can get new task which they are yet to complete or next in line in previous plan. Don't give task which were completed again. Resume must be strictly based on the current status — not task history.\n"
            "   - Use the keyword `resume` to instruct the robot to continue its previous task.\n"
            "   - Example: Robot: Resume task descrption of tasks which were in progress \n"
            "   - Do NOT resume tasks marked as TASK COMPLETE. \n"
            "   - Do NOT reassign already completed tasks unless explicitly asked by human. \n"
            "   - Do NOT assume a robot is idle if its task status says TASK COMPLETE.  \n"
            f"  - {replanning_rules} \n\n"



            "### **Task Output Format (STRICTLY FOLLOW THIS):**\n"
            "**First, think step-by-step: Imagine all the real-world actions required for the task. Try to understand what the user wants and try to understand the scene along with it to decide robot action. Identify if multiple robots and their capabilites or skills must interact. Only then classify the task into Plan or Independent or Team.**\n\n"
            "1. If the task requires multiple robots to COORDINATE to do Tasks that cannot be done parallely and have sequential dependence , ONLY THEN generate a step-by-step plan in this format Else you SHOULD AND MUST Proceed with Independent tasks:\n"
            "   - It is important that independent task must be identified as they can be done parallely.\n"
            "   - Do not give the task from history unless it was stopped \n"
            "   - Every time the Task Manager assigns, resumes, or replans any task for any robot, the output must start with either Plan: (for sequential/coordinated tasks) or Independent Tasks: (for standalone tasks). One of these two section headers is mandatory when Task Manager is assigning tasks to robots.\n"
            "   - You are an AI assistant so use all your logic and create a logical plan which contains all steps needs to be taken to complete the given task if it falls under a Plan category\n"
            "   - Take extreme care about FEASIBILITY and MORE SAFETY while choosing the order of the tasks in the plan.\n"
            "   - Each step in the Plan must ensure that all preconditions necessary for the subsequent action are explicitly and successfully satisfied. The transition from one step to the next must be both physically executable and logically consistent with the current world state. No action should be scheduled unless all enabling conditions have been fulfilled through prior, verifiable steps. Plans must avoid discontinuities or assumptions that skip essential task dependencies or violate causal flow."
            "   - If no valid in-progress task was interrupted, skip the Resume step completely.\n\n"
            "   If a robot or Team of robots are repeated in the list then it can Mixed Plan task\n"

            "   - **'Plan:Step_No. Robot_Name: Task Description (clear mention)'**\n"
            "   - Example:\n"
            "     Plan:\n"
            "     1. Robot_Name: Task Description\n"
            "     2. Robot_Name: Task Description\n"
            "     3. Robot_Name: Task Description\n"
            "     multirobot_list: [Team 1,Robot name 1,Team 2,Robot name 2, Robot name 3]\n"
            "     singlerobot_list: []\n\n"
            
            "   Similarly Remaining Steps. If this is a replanning event: "
            "       Only add `Resume` steps **IF** the task was in progress at the time of event.\n"
            "       Very IMPORTANT: The task order should be feasible and more important tasks should be done first.\n"
            "       If a robot's task status is `TASKS COMPLETE`, DO NOT add any Resume step for that robot — it's finished. \n"
            "       Resume the task ONLY IF the status was `IN PROGRESS`.\n"
            "       NEVER include Resume step if robot's task status is TASKS COMPLETE — the task is fully done and must never be repeated unless explicitly asked by human.\n"
            "       If tasks are interrupted and clearly marked as `IN PROGRESS` at time of event they should be resumed. \n"
            "       Ignore old tasks if they are marked as `TASKS COMPLETE` in the current robot state. \n"
            "       **IMPORTANT RULE :**ONLY TO BE APPLIED WHEN REPLANNING DUE TO AN EVENT :If a robot had a task that was marked as IN PROGRESS at the time of an event or interruption, that tasks MUST ALWAYS be resumed and completed by giving a Plan containing it BEFORE any new tasks are given and Dont give STOP.\n\n"

            
            "2. If a task or tasks involves only one robot or can be performed independently or you think this task can be performed PARALLELY alongside other robot, mention it in the following format:\n"
            "   Always ensure the sequence follows logical constraints and optimally utilizes each robot's capabilities.\n"
            "   Always ensure that solution doesn't violate morphology constraints of robots while interacting with objects. example, if workspace of robotic arm is given as 0.5 m then it can lift object placed outside that range or a mobile robot of height 0.1 m cannot reach top of the shelf.\n"
            "   Tasks that can be performed parallely should be part of this instead of following output format section 1 given above.\n"
            "   This format should be applied for both task allocation during planning and replanning, if a task or tasks involves only one robot or can be performed independently \n"
            "   Also, format your relevant lines EXACTLY as given below:\n"
            "   If a robot or Team of robots are repeated in the list then it can not be pure Independent task or Mixed Independent task\n"
            "   Independent Tasks:\n"
            "   Robot_Name tasks: task1_description, task2_description, ...\n"
            "   Robot_Name tasks: task1_description, task2_description, ...\n"
            "   Team 1:[Robot_Name, Robot_Name, Robot_Name]: task1_description, task2_description, ...\n"
            "   Robot_Name tasks: task1_description, task2_description, ...\n"
            "   Robot_Name tasks: task1_description, task2_description, ...\n"
            "   Team 2:[Robot_Name, Robot_Name, Robot_Name]: task1_description, task2_description, ...\n"

            "   - Example:\n"
            "      Independent Tasks:\n"
            "      Robot_Name_1 tasks: stand and sit 10 times, dance, ...\n"
            "      Robot_Name_2 tasks: go to charging station, task2_name, ...\n"
            "      Team 1:[Robot_Name_6, Robot_Name_7, Robot_Name_10]: 3 of the robots goto chair together, ...\n"
            "      Robot_Name_3 tasks: pick and place trash, move forward 5m, ...\n"
            "      Robot_Name_4 tasks : Perform Return to Home \n"
            "      Team 2:[Robot_Name_8, Robot_Name_12, Robot_Name_15]: 3 of the robots goto chair together, ...\n"
            "      multirobot_list: []\n"
            "      singlerobot_list: [Robot_Name_1, Robot_Name_2, Team 1, Robot_Name_3, Robot_Name_4, Team 2]\n\n"


            "3. If task involved requires tight COORDINATION of movements done parallely by robot then mention it in this format.\n"
            "   The robots require to move neither independently nor sequentially but their motion depend on each other in realtime.\n"
            "   The motion of robots is tightly coupled with each other.\n"
            "   This format should be applied for task allocation for coordinated tasks described in this section. \n"
            "   Choose which robots to be part of coordinated motion task according to user demands and their capabilities.\n"
            "   Also, format your relevant lines EXACTLY as given below:\n"
            "   Independent Tasks or Plan:\n"
            "   Team 1: [Robot_Name, Robot_Name, Robot_Name]: task1_description, task2_description, ...\n"
            "   Team 2: [Robot_Name, Robot_Name, Robot_Name]: task1_description, task2_description, ...\n\n"
            
            
            "4.1 Mixed Tasks: If a task requires multiple types among sequential and coordinated tasks, mention it in Plan with this format.\n"
            "   If a robot or Team of robots are repeated in the list then it can Mixed Plan task\n"
            "   If a robot has to be there in team also and single robot task also then go with Mixed Plan task\n"
            "      Plan:\n"
            "      1. Team 1: [Robot_Name, Robot_Name, Robot_Name]: Coordinated Task Description\n"
            "      2. Robot_Name_1: Task Description\n"
            "      3. Team 2: [Robot_Name, Robot_Name, Robot_Name]: Coordinated Task Description\n"
            "      4. Robot_Name_2: Task Description\n"
            "      5. Robot_Name_3: Task Description\n"
            "      multirobot_list: [Team 1,Robot_name_1,Team 2,Robot_name_2, Robot_name_3]\n"
            "      singlerobot_list: []\n\n"
            
            "4.2 Mixed Tasks: If a task requires multiple types among independent tasks and coordinated tasks, mention it in Independent task with this format.\n"
            "   If a robot or Team of robots are repeated in the list then it can not be pure Independent task or Mixed Independent task\n"
            "   Do not give a single robot an indpendent task if it is part of a team task  in the same task.\n"
            "      Independent Tasks:\n"
            "      Team 1: [Robot_Name_5, Robot_Name_6, Robot_Name_7]: Coordinated Task Description\n"
            "      Robot_Name_1: Task Description\n"
            "      Team 2: [Robot_Name, Robot_Name, Robot_Name]: Coordinated Task Description\n"
            "      Robot_Name_2: Task Description\n"
            "      Robot_Name_3: Task Description\n"
            "      multirobot_list: []\n"
            "      singlerobot_list: [Team 1,Robot_name_1,Team 2,Robot_name_2, Robot_name_3]\n\n"


            "5. Also provide three lists: **multirobot_list**,**singlerobot_list**,**team_list**.\n"
            "   - **multirobot_list**: A list of team names(in case of team task) and robot namess(involved in plan) (strings) involved in the multi-step Plan.\n"
            "   - **singlerobot_list**: A list of team names(in case of team independent task) and robot names(involved in independent task) (strings) involved in independent tasks. Dont include the robot involved in plan\n\n"


            "6. **Impossible Tasks:**\n"
            "   - If a robot is given a task that exceeds its payload, reach, or other constraints such as components, print why it cannot perform the task before stating it is unfeasible. Ensure this reasoning is clear before providing alternatives or rejecting the task."
            "   - However, if an alternative robot can perform the task instead, assign it and print a note:**\n"
            "   - If task area is out of range to a fixed robot then the robot can not do the task, in case of if any other robot is having it inside its working space with the required componensts then that robot can do the task \n"            
            "    **'Note: Tasks have been reassigned to available robots to ensure completion.'**\n"
            "   - If a robot is **incorrectly assigned** a task beyond its capacity (e.g., lifting more than its payload limit), immediately correct the plan and provide a feasible alternative. If no alternative exists, print the message below.\n\n"


            "7. Sequential execution (as described in Section 1) can cause delays. Therefore, for crit1ical tasks, use the independent task format provided in Section 2 whenever possible. \n"
            " IMPORTANT : DONT UNNECESSARY TEXT IN OUTPUT , DONT WANT ANY TASK DESCRIPTION . JUST MENTION PLAN AND/OR INDEPENDENT TASKS SECTION AND MULTI-ROBOT AND SINGLE ROBOT LISTS ACCORDING TO ABOVE RULES\n\n"

            "8. Chat History and Time:\n"
            f"   - Current time: {self.current_time}\n"
            "   - Time format is Hours: {hours:02d}, Minutes: {minutes:02d}, Seconds: {seconds:02d}"
            "   - Time stamped Chat History:\n"
            f"{self.chat_log}\n"
            "   - Use this to remember the old chat history and the time stamps of conversation.\n"
            

            "9. Configuration file of current application.\n"
            "   - Use this to understand the robot capabilities, morphology and task specific rules.\n"
            "   - Make sure to strictly follow the rules and constraints below.\n\n"
            f"  - Robot Capabilities: {capability}\n"
            f"  - Robot Morphology: {morphology}\n"
  
            "11. Role and Task specific rules:**\n"
            f"  - Task Specific Rules: {rules}\n"
           
            )

        except Exception as e :
            self.get_logger().warning(f"Error building prompt : {e}")
            return None
        
        return header + "\n" + static 

    def on_chat_input(self, msg: String):  
        entry = f"Config File name: {self.config_file} \nLLM No. : {self.model} \nLLM Name: {self.model_name}\n"

        entry = entry + f"{msg.data}\n\n\n" 

        # package_share = get_package_share_directory("chatty")

        # data_saver_file = os.path.join(package_share, "data", self.file_name)

        # with open(data_saver_file, "a") as f:
        #     f.write(entry)
        #     f.flush()  # Ensures data is written immediately
        

    def on_chat_output(self, msg: String):
        """
        We store every line from /chat/output.
        If it's a human line, parse new tasks (partial override).
        If it has 'Event:', we do a full re-plan.
        """
        self.current_message = msg.data
        self.conversation_log.append(self.current_message)
        # robot_names = self.robot_config["robot_names"]

        # if "] Human:" in line or any(f"{name} (msg)" in line for name in robot_names):
        if "Human:" in self.current_message or "(msg)" in self.current_message:
            self.get_logger().info("[TaskManager] Detected user message -> parse tasks for subset of robots.")
            self.get_logger().info(f"Chat Output: {self.current_message}")
            self.parse_tasks_with_gpt()


    def on_status_callback(self, msg:String) :
        self.get_logger().info("[TaskManager] Status Callback.")
        self.task_status = msg.data
        self.get_logger().info(f"Status {self.task_status}")
        
    def on_robot_state_callback(self, msg : String):
        self.robot_state = msg.data

    # --------------------------------------------------------------------------
    #  PARTIAL TASK PARSING (HUMAN COMMANDS)
    # --------------------------------------------------------------------------
    def parse_tasks_with_gpt(self):
        """
        Calls GPT with the entire conversation, prompting it to only mention
        the robots that actually get new tasks. Others remain unchanged.
        Encourages multi-step and pause/resume logic.
        """
        messages = self.build_messages()
        self.system_prompt = self.build_system_prompt()

        if self.system_prompt == None :
            self.get_logger().error("Invalid Prompt")
            exit(0)

        messages.insert(0, {"role": "system", "content": self.system_prompt})
        # ai_output = self.call_openai(messages, debug_label="Allocation")

        try:
            with open('prompts.txt', "a") as file:
                file.write(json.dumps(messages, indent=2))
                file.write("\n")  # optional: add newline for readability            # self.get_logger().info(f"[TaskManager] Loaded {len(self.chat_log)} previous messages.")
        except Exception as e:
            self.get_logger().error(f"[TaskManager] Failed to write prompts: {e}")

        ######################### OPENAI MODELS #########################

        if self.model == 1:
            ai_output = self.call_openai(messages, model_="gpt-4", debug_label="Allocation")
            self.model_name = "gpt-4"
            print(f"model name {self.model_name}")
            
        elif self.model == 2:
            ai_output = self.call_openai(messages, model_="gpt-4.1-nano", debug_label="Allocation")
            self.model_name = "gpt-4.1-nano"

        elif self.model == 3:
            ai_output = self.call_openai(messages, model_="gpt-3.5-turbo", debug_label="Allocation")
            self.model_name = "gpt-3.5-turbo"

        elif self.model == 4:
            ai_output = self.call_openai(messages, model_="gpt-4o", debug_label="Allocation")
            self.model_name = "gpt-4o"

        elif self.model == 5:
            ai_output = self.call_openai(messages, model_="gpt-4.1-mini", debug_label="Allocation")
            self.model_name = "gpt-4.1-mini"

        elif self.model == 6:
            ai_output = self.call_openai(messages, model_="gpt-4o-mini", debug_label="Allocation")
            self.model_name = "gpt-4o-mini"

        elif self.model == 7:
            ai_output = self.call_openai(messages, model_="gpt-4-turbo", debug_label="Allocation")
            self.model_name = "gpt-4-turbo"
            
        elif self.model == 8:
            ai_output = self.call_openai(messages, model_="gpt-4.1", debug_label="Allocation")
            self.model_name = "gpt-4.1"

        elif self.model == 9:
            ai_output = self.call_openai(messages, model_="o4-max", debug_label="Allocation")
            self.model_name = "o4-max"
            
        elif self.model == 11:
            ai_output = self.call_openai_1(messages, model_="o1", debug_label="Allocation")
            self.model_name = "o1"

        elif self.model == 12:
            ai_output = self.call_openai_1(messages, model_="o4-mini", debug_label="Allocation")
            self.model_name = "o4-mini"

        elif self.model == 13:
            ai_output = self.call_openai_1(messages, model_="o1-mini", debug_label="Allocation")
            self.model_name = "o1-mini"

        elif self.model == 14:
            ai_output = self.call_openai_1(messages, model_="o1-pro", debug_label="Allocation")
            self.model_name = "o1-pro"

        elif self.model == 15:
            ai_output = self.call_openai_1(messages, model_="o3-mini", debug_label="Allocation")
            self.model_name = "o3-mini"

        ############# OLLAMA MODELS #####################

        elif self.model == 16:
            ai_output = self.call_ollama(messages, model_="gemma3:latest")            
            self.model_name = "gemma3:latest"

        elif self.model == 17:
            ai_output = self.call_ollama(messages, model_="gemma2")     
            self.model_name = "gemma2"

        elif self.model == 18:
            ai_output = self.call_ollama(messages, model_="gemma3:1b")            
            self.model_name = "gemma3:1b"

        elif self.model == 19:
            ai_output = self.call_ollama(messages, model_="gemma3")           
            self.model_name = "gemma3"

        elif self.model == 20:
            ai_output = self.call_ollama(messages, model_="gemma:2b")        
            self.model_name = "gemma:2b"

        elif self.model == 21:
            ai_output = self.call_ollama(messages, model_="gemma3:4b")        
            self.model_name = "gemma3:4b"

        elif self.model == 22:
            ai_output = self.call_ollama(messages, model_="gemma2:latest")        
            self.model_name = "gemma2:latest"

        elif self.model == 23:
            ai_output = self.call_ollama(messages, model_="gemma:latest")        
            self.model_name = "gemma:latest"

        ############# DEEPSEEK MODELS #####################

        elif self.model == 24:
            ai_output = self.call_ollama(messages, model_="deepseek-r1")           
            self.model_name = "deepseek-r1"
            
        elif self.model == 25:
            ai_output = self.call_ollama(messages, model_="deepseek-r1:latest")            
            self.model_name = "deepseek-r1:latest"

        elif self.model == 26:
            ai_output = self.call_ollama(messages, model_="deepseek-v2")
            self.model_name = "deepseek-v2"

        elif self.model == 27:
            ai_output = self.call_ollama(messages, model_="deepseek-r1:1.5b")
            self.model_name = "deepseek-r1:1.5b"

        elif self.model == 28:
            ai_output = self.call_ollama(messages, model_="deepseek-llm")
            self.model_name = "deepseek-llm"

        elif self.model == 29:
            ai_output = self.call_ollama(messages, model_="deepseek-llm:7b")        
            self.model_name = "deepseek-llm:7b"

        elif self.model == 30:
            ai_output = self.call_ollama(messages, model_="deepseek-llm:latest")      
            self.model_name = "deepseek-llm:latest"

        elif self.model == 31:
            ai_output = self.call_ollama(messages, model_="deepseek-coder")
            self.model_name = "deepseek-coder"

        ############# QWEN MODELS #####################

        elif self.model == 32:
            ai_output = self.call_ollama(messages, model_="qwen2:1.5b")      
            self.model_name = "qwen2:1.5b"

        elif self.model == 33:
            ai_output = self.call_ollama(messages, model_="qwen2:0.5b")           
            self.model_name = "qwen2:0.5b"

        elif self.model == 34:
            ai_output = self.call_ollama(messages, model_="qwen:1.8b")         
            self.model_name = "qwen:1.8b"

        elif self.model == 35:
            ai_output = self.call_ollama(messages, model_="qwen:0.5b")
            self.model_name = "qwen:0.5b"

        elif self.model == 36:
            ai_output = self.call_ollama(messages, model_="qwen2:7b")
            self.model_name = "gqwen2:7b"

        elif self.model == 37:
            ai_output = self.call_ollama(messages, model_="qwen2.5:0.5b")
            self.model_name = "qwen2.5:0.5b"

        elif self.model == 38:
            ai_output = self.call_ollama(messages, model_="qwen:4b")          
            self.model_name = "qwen:4b"

        elif self.model == 39:
            ai_output = self.call_ollama(messages, model_="qwen2.5:7b")
            self.model_name = "qwen2.5:7b"

        elif self.model == 40:
            ai_output = self.call_ollama(messages, model_="qwen2.5:latest")
            self.model_name = "qwen2.5:latest"

        elif self.model == 41:
            ai_output = self.call_ollama(messages, model_="qwen2.5vl:latest")
            self.model_name = "qwen2.5vl:latest"

        elif self.model == 42:
            ai_output = self.call_ollama(messages, model_="qwen2.5vl:7b")     
            self.model_name = "qwen2.5vl:7b"

        elif self.model == 43:
            ai_output = self.call_ollama(messages, model_="qwen3:1.7b")
            self.model_name = "qwen3:1.7b"

        elif self.model == 44:
            ai_output = self.call_ollama(messages, model_="qwen3:8b")          
            self.model_name = "qwen3:8b"

        elif self.model == 45:
            ai_output = self.call_ollama(messages, model_="qwen2:latest")
            self.model_name = "qwen2:latest"

        elif self.model == 46:
            ai_output = self.call_ollama(messages, model_="qwen:latest")      
            self.model_name = "qwen:latest"

        elif self.model == 47:
            ai_output = self.call_ollama(messages, model_="qwen3:latest")
            self.model_name = "qwen3:latest"

        ############# LLAMA MODELS #####################

        elif self.model == 48:
            ai_output = self.call_ollama(messages, model_="llama3:8b")         
            self.model_name = "llama3:8b"

        elif self.model == 49:
            ai_output = self.call_ollama(messages, model_="llama3:latest")
            self.model_name = "llama3:latest"

        elif self.model == 50:
            ai_output = self.call_ollama(messages, model_="llama3.1:latest")
            self.model_name = "llama3.1:latest"

        elif self.model == 51:
            ai_output = self.call_ollama(messages, model_="dolphin3:8b")        
            self.model_name = "dolphin3:8b"

        elif self.model == 52:
            ai_output = self.call_ollama(messages, model_="dolphin3:latest")    
            self.model_name = "dolphin3:latest"

        elif self.model == 53:
            ai_output = self.call_ollama(messages, model_="llama2:latest")      
            self.model_name = "llama2:latest"

        elif self.model == 54:
            ai_output = self.call_ollama(messages, model_="llama2:7b")
            self.model_name = "llama2:7b"

        elif self.model == 55:
            ai_output = self.call_ollama(messages, model_="tinyllama:latest")
            self.model_name = "tinyllama:latest"

        elif self.model == 57:
            ai_output = self.call_ollama(messages, model_="llama3.2:latest")
            self.model_name = "llama3.2:latest"

        elif self.model == 58:
            ai_output = self.call_ollama(messages, model_="llama3.2:3b")        
            self.model_name = "llama3.2:3b"

        elif self.model == 59:
            ai_output = self.call_ollama(messages, model_="llama4")             
            self.model_name = "llama4"

        elif self.model == 60:
            ai_output = self.call_ollama(messages, model_="llama3.3")           
            self.model_name = "llama3.3"

        elif self.model == 61:
            ai_output = self.call_ollama(messages, model_="llama3.2")
            self.model_name = "llama3.2"

        elif self.model == 62:
            ai_output = self.call_ollama(messages, model_="llama3.1")           
            self.model_name = "llama3.1"

        elif self.model == 63:
            ai_output = self.call_ollama(messages, model_="llama3")
            self.model_name = "llama3"

        elif self.model == 64:
            ai_output = self.call_ollama(messages, model_="llama2")
            self.model_name = "llama2"

        elif self.model == 65:
            ai_output = self.call_ollama(messages, model_="llama-pro")         
            self.model_name = "llama-pro"

        elif self.model == 66:
            ai_output = self.call_ollama(messages, model_="tinyllama")
            self.model_name = "tinyllama"

        elif self.model == 67:
            ai_output = self.call_ollama(messages, model_="dolphin3")
            self.model_name = "dolphin3"

        elif self.model == 68:
            ai_output = self.call_ollama(messages, model_="llama-pro:instruct") 
            self.model_name = "llama-pro:instruct"

        elif self.model == 69:
            ai_output = self.call_ollama(messages, model_="llama-pro:latest")   
            self.model_name = "llama-pro:latest"

        elif self.model == 70:
            ai_output = self.call_ollama(messages, model_="llama-pro")          
            self.model_name = "llama-pro"

        ############# MISTRAL MODELS #####################

        elif self.model == 71:
            ai_output = self.call_ollama(messages, model_="mistral:7b")
            self.model_name = "mistral:7b"

        elif self.model == 72:
            ai_output = self.call_ollama(messages, model_="mistral-nemo")
            self.model_name = "mistral-nemo"

        elif self.model == 73:
            ai_output = self.call_ollama(messages, model_="mistral-nemo:12b")
            self.model_name = "mistral-nemo:12b"

        elif self.model == 74:
            ai_output = self.call_ollama(messages, model_="minicpm-v")
            self.model_name = "minicpm-v"

        elif self.model == 75:
            ai_output = self.call_ollama(messages, model_="minicpm-v:8b")
            self.model_name = "minicpm-v:8b"

        elif self.model == 76:
            ai_output = self.call_ollama(messages, model_="minicpm-v:latest")
            self.model_name = "minicpm-v:latest"

        elif self.model == 77:
            ai_output = self.call_ollama(messages, model_="mistral-nemo:latest")
            self.model_name = "mistral-nemo:latest"

        elif self.model == 78:
            ai_output = self.call_ollama(messages, model_="mistral:latest")
            self.model_name = "mistral:latest"

        ############# GEMINI MODELS #####################

        elif self.model == 80:
            ai_output = self.call_gemini(messages, model_="gemini-2.0-pro")
            self.model_name = "gemini-2.0-pro"

        elif self.model == 81:
            ai_output = self.call_gemini(messages, model_="gemini-2.5-pro")
            self.model_name = "gemini-2.5-pro"
        
        elif self.model == 82:
            ai_output = self.call_gemini(messages, model_="gemini-2.0-flash")
            self.model_name = "gemini-2.0-flash"

        elif self.model == 83:
            ai_output = self.call_gemini(messages, model_="gemini-2.0-flash-lite")
            self.model_name = "gemini-2.0-flash-lite"

        elif self.model == 84:
            ai_output = self.call_gemini(messages, model_="gemini-1.5-flash")
            self.model_name = "gemini-1.5-flash"

        elif self.model == 85:
            ai_output = self.call_gemini(messages, model_="gemini-1.5-flash-8b")
            self.model_name = "gemini-1.5-flash-8b"

        ############# GROK MODELS #####################

        elif self.model == 86:
            ai_output = self.call_xai(messages, model_="grok-2-1212")
            self.model_name = "grok-2-1212"

        elif self.model == 87:
            ai_output = self.call_xai(messages, model_="grok-2-vision-1212")
            self.model_name = "grok-2-vision-1212"

        elif self.model == 88:
            ai_output = self.call_xai(messages, model_="grok-3")
            self.model_name = "grok-3"

        elif self.model == 89:
            ai_output = self.call_xai(messages, model_="grok-3-fast")
            self.model_name = "grok-3-fast"

        elif self.model == 90:
            ai_output = self.call_xai(messages, model_="grok-3-mini")
            self.model_name = "grok-3-mini"

        elif self.model == 91:
            ai_output = self.call_xai(messages, model_="grok-3-mini-fast")
            self.model_name = "grok-3-mini-fast"
            print(f"model name {self.model_name}")

        elif self.model == 92:
            ai_output = self.call_xai(messages, model_="grok-4-0709")
            self.model_name = "grok-4-0709"
            print(f"model name {self.model_name}")


        #################################################################

        elif self.model == 93:
            ai_output = self.call_ollama(messages, model_="phi3:mini")
            self.model_name = "phi3:mini"
            print(f"model name {self.model_name}")

        elif self.model == 94:
            ai_output = self.call_ollama(messages, model_="tinyllama:1.1b")
            self.model_name = "tinyllama:1.1b"
        
        elif self.model == 95:
            ai_output = self.call_ollama(messages, model_="deepseek-r1:7b")
            self.model_name = "deepseek-r1:7b"

        elif self.model == 96:
            ai_output = self.call_ollama(messages, model_="llama3:70b")
            self.model_name = "llama3:70b"
        
        elif self.model == 97:
            ai_output = self.call_ollama(messages, model_="deepseek-r1:32b")
            self.model_name = "deepseek-r1:32b"
        
        elif self.model == 98:
            ai_output = self.call_ollama(messages, model_="yi:34b")
            self.model_name = "yi:34b"
        
        elif self.model == 99:
            ai_output = self.call_ollama(messages, model_="gemma3:27b")
            self.model_name = "gemma3:27b"
        
        elif self.model == 100:
            ai_output = self.call_ollama(messages, model_="qwen3:32b")
            self.model_name = "qwen3:32b"
        
        elif self.model == 101:
            ai_output = self.call_ollama(messages, model_="mixtral:8x7b")
            self.model_name = "mixtral:8x7b"

        ########### HERE WE CAN ADD MORE MODELS #####################


        else:
            ai_output = "Invalid model selection"
            self.model_name = "Not selected"

        
        print(f" Ai output \n\n {ai_output}")
        # print("hello...")

        # out_pub = Bool()
        # out_pub.data = True
        # print(f"Publish {out_pub.data}")
        # self.publisher_.publish(out_pub)

        self.publish_chat_output(ai_output, source="allocation")
        
        if "Plan" in ai_output or "Independent Tasks" in ai_output : # To stop only in the cases of valid responses and reject any other 
            self.master_status = True # Stop all other timers
            self.task_queue = Queue() # Create a new Queue to fit in the tasks
            self.multirobot_list = []
            self.singlerobot_list = []
            # self.team_list = []
            self.multirobot_list, self.singlerobot_list, self.single_tasks_dict = self.extract_task_data(ai_output)
        # print("Hello")

        print(f"Multirobot List : {self.multirobot_list}")
        print(f"Singlerobot List : {self.singlerobot_list}")
        print(" bfore 1st if")
        if "Plan" in ai_output and "Independent Tasks" in ai_output:
            # print("World")
            # self.multirobot_list, self.singlerobot_list, self.single_tasks_dict = self.extract_task_data(ai_output)
            print(f"MultiRobot List {self.multirobot_list}")
            print(f"SingleRobot List {self.singlerobot_list}")
            print(f"SingleRobot Task Dict {self.single_tasks_dict}")
            self.publish_plan_json(event="Strict")

        if "Plan" in ai_output:
            print("Hello I am in Plan")
            self.create_queue(ai_output) # add Plan in Queue
            self.publish_single_task(ai_output) # publish First task to task_json
            self.master_status = False
            self.plan_timer = self.create_timer(0.1,lambda : self.monitor_task_progress(ai_output)) # Monitor Task Progress
        else: 
            self.multirobot_completion_status = True

        if "Independent Tasks" in ai_output:
            self.master_status = False
            self.single_timer = self.create_timer(1.0,lambda : self.execute_single_tasks(ai_output)) # Monitor Task Progress
            
        print("GPT RAW OUTPUT: {}".format(ai_output))

    def create_queue(self, ai_output) :
        print("Creating Plan ...")        
        # Find the Plan section
        plan_start = ai_output.find("Plan")
        if plan_start == -1:
            self.get_logger().warning(" 'Plan' Not Found")
    
        # Extract lines after "Plan:"
        plan_lines = ai_output[plan_start:].split("\n")[1:]  # Skip "Plan:" line
        
        # Regex to match task lines (e.g., "1. Waffle: Finds the yellow ball")
        task_pattern = re.compile(r"^\d+\.\s(.+)$")  
        
        for line in plan_lines:
            match = task_pattern.match(line.strip())
            if match:
                self.task_queue.put(match.group(1))  # Extract the task part

    def monitor_task_progress(self, ai_output) :
        if self.master_status == True :
            self.plan_timer.destroy()
        elif "COMPLETE" in self.task_status: 
            print(f"Task Status : {self.task_status}")

            # Extract robot name (Assuming format: "Robot_name (status) : COMPLETE")
            match = re.match(r"([\w\s]+)\s*\(.*?\)\s*:.*?TASKS COMPLETE", self.task_status)
            subtask_robot_match = re.match(r"([\w\s]+)\s*\(.*?\)\s*:.*?COMPLETE", self.task_status)
            print(f"Match {match}")
            if match:
                robot_name = match.group(1)   # Extract and capitalize robot name
                print(f"robot_name is {robot_name}")
                # Check if robot_name is in self.single_tasks_dict
                if robot_name.lower() in [name.lower() for name in self.singlerobot_list]:
                    self.task_status = "NEXT IN PROGRESS"
                    print(f"Task for {robot_name} is an independent task, ignoring.")
                else:
                    self.task_status = "NEXT IN PROGRESS"
                    print(f"Previous task complete for {robot_name} ..... Publishing the next task from the Plan")
                    self.publish_single_task(ai_output)
            else:
                # If no robot name found, proceed normally
                self.get_logger().info(f"{subtask_robot_match} Sub-Task is completed")
                self.task_status = "NEXT IN PROGRESS"
                # self.publish_single_task(ai_output)

    def execute_single_tasks(self,ai_output):
        # print("inside independent")
        if self.master_status == True :
            # print("inside master status true")
            self.single_timer.destroy()
        elif self.multirobot_completion_status == True:
            # print("inside multirobot completion")
            if "Independent Tasks" in ai_output:
                print("Publishing individual tasks")
                self.update_all_robot_tasks(ai_output)
                self.publish_plan_json(event="Normal")
                self.single_timer.destroy()
                self.multirobot_completion_status = False

    def publish_single_task(self, ai_output: str):
        print("Publishing task by task")

        if self.task_queue.empty():
            self.get_logger().info("[TaskManager] ALL TASKS COMPLETED")
            self.plan_timer.destroy()
            self.publish_multirobot_success()
            self.publish_plan_json(event="Empty all")
            return
        print("*******queue task : ", list(self.task_queue.queue))
        task = self.task_queue.get()
        
        self.get_logger().info(f"[TaskManager] Processing Task: {task}")

        try:
            raw_robot_name, task_description = task.split(": ", 1)
        except ValueError:
            self.get_logger().warn(f"[TaskManager] Invalid task format: {task}")
            return

        raw_robot_name = raw_robot_name.strip().lower()
        raw_robot_name = raw_robot_name.replace("_", " ")  # Normalize robot name
        self.robot_tasks = {}
        matched = False
        print(f" Multirobot List : {self.multirobot_list}")
        for robot in self.multirobot_list:
            robot_display_name = robot.replace("_", " ").lower()
            # robot_display_name = robot.lower()  # Ensure we compare in lowercase

            print(f"Comparing '{raw_robot_name}' with '{robot_display_name}'")

            if raw_robot_name == robot_display_name:
                self.robot_tasks[robot] = task_description
                matched = True
                print(f"[MATCHED] Assigned '{task_description}' to {robot}")
                break
            elif "team" in robot_display_name:
                # Handle team tasks
                team_name = robot_display_name.split("team")[0].strip()
                if raw_robot_name == team_name:
                    self.robot_tasks[robot] = task_description
                    matched = True
                    print(f"[MATCHED] Assigned '{task_description}' to {robot} (team task)")
                    break
            
            else:
                print(f"[NO MATCH] '{raw_robot_name}' != '{robot_display_name}'")

        if not matched:
            self.get_logger().warn(f"[TaskManager] Unable to assign task: {task}")
            return

        json_obj = {
            "robot_tasks": self.robot_tasks,
            "sequence": task
        }

        json_str = json.dumps(json_obj, ensure_ascii=False)
        json_msg = String()
        json_msg.data = json_str
        self.pub_tasks_json.publish(json_msg)
        self.get_logger().info(f"[TaskManager] Published JSON: {json_str}")


    def publish_plan_json(self, event=None):
        print(f"Publishing Plan JSON with event: {event}")

        print(f"Plan state : {event}\n")

        robot_tasks = {name.lower(): "" for name in self.robot_config.get("robot_names", [])}

        seq = ""

        if event == "Empty all":
            print("Emptying All Tasks ---- NEW TASKS Received")
            return 
        elif event == "Strict":
            if self.single_tasks_dict:
                print("Publishing Strict Independent Tasks")
                
                # Ensure lowercase robot names and maintain base structure
                for robot, task in self.single_tasks_dict.items():
                    robot_lower = robot.lower()  # Ensure lowercase keys
                    if robot_lower in robot_tasks:  
                        robot_tasks[robot_lower] = task  # Assign task only to known robots
            else:
                print("No independent tasks found!")

        else:
            robot_tasks = self.robot_tasks  # Default case
            seq = self.sequence_of_tasks

        # Ensure JSON consistency
        json_obj = {
            "robot_tasks": robot_tasks,
            "sequence": seq
        }
        json_str = json.dumps(json_obj)  # Convert to JSON string
        json_msg = String()
        json_msg.data = json_str
        self.pub_tasks_json.publish(json_msg)
        self.get_logger().info(f"[TaskManager] Published JSON: {json_str}")

    def publish_multirobot_success(self):
        self.multirobot_completion_status = True
        chat_msg = String()
        chat_msg.data = (
            " !! Multi-Robot Task Completed Successfully !!.\n"
        )
        self.pub_chat_input.publish(chat_msg)


    def extract_task_data(self, ai_output):
        """
        Extracts:
        1) multirobot_list (robots involved in multi-robot tasks)
        2) singlerobot_list (ALL robots in the Independent Tasks section)
        3) single_tasks_dict (robots from singlerobot_list but NOT in multirobot_list)
        """

        print("Extracting task data from GPT output...\n")

        multirobot_list = []
        singlerobot_list = []
        # self.teams_list = []
        single_tasks_dict = {}

        # Extract lists using regex
        multi_match = re.search(r"multirobot_list\s*[:=]\s*(\[.*?\])", ai_output)
        print(f"Multi Match {multi_match}")
        single_match = re.search(r"singlerobot_list\s*[:=]\s*(\[.*?\])", ai_output)
        print(f"Single Match {single_match}")

        if multi_match:
            list_str = multi_match.group(1)
            # Add quotes around list items if they are unquoted
            quoted = re.sub(r"([^\[\],\s]+(?:\s+[^\[\],\s]+)*)", r'"\1"', list_str)
            multirobot_list = ast.literal_eval(quoted)
            print(f"Multirobot List : {multirobot_list}")

        if single_match:
            list_str = single_match.group(1)
            quoted = re.sub(r"([^\[\],\s]+(?:\s+[^\[\],\s]+)*)", r'"\1"', list_str)
            singlerobot_list = ast.literal_eval(quoted)
            print(f"Singlerobot List : {singlerobot_list}")

        # Extract robots mentioned in the Plan section
        plan_section_match = re.search(r"Plan:(.*?)(?:\n\n|\Z)", ai_output, re.DOTALL)
        robots_in_plan = set()

        if plan_section_match:
            plan_text = plan_section_match.group(1).strip()
            plan_lines = plan_text.split("\n")

            for line in plan_lines:
                stripped = line.strip()
                if not stripped:
                    continue

                match = re.match(r"(\w+):", stripped)
                print(f"Match {match}")
                if match:
                    robots_in_plan.add(match.group(1).capitalize())

        # Extract robots and their tasks from "Independent Tasks"
        independent_section = re.search(r"Independent Tasks\s*[:=]\s*(.*?)(?:\n\n|\Z)", ai_output, re.DOTALL)
        print(f"Independent Section {independent_section}")

        if independent_section:
            independent_text = independent_section.group(1).strip()
            lines = independent_text.split("\n")
            print(f"Independent Text {independent_text}")
            print(f"Lines {lines}")

            for line in lines:
                stripped = line.strip()
                print(f"Stripped Line {stripped}")
                if not stripped:
                    continue

                # match = re.match(r"(.+?):\s*(\[.*?\])\s*:\s*(.+)", stripped)
                match = re.match(r"(.+?):\s*(.+)", stripped)

                print(f"Match {match}")
                if match:
                    robot_name = match.group(1).capitalize()
                    task_description = match.group(2).strip()

                    # **ALL** robots in Independent Tasks should be in singlerobot_list
                    singlerobot_list.append(robot_name)

                    # **Only add to single_tasks_dict if NOT in multirobot_list**
                    if robot_name not in multirobot_list:
                        print(f"Robot Name: {robot_name} \n task description {task_description}")
                        single_tasks_dict[robot_name] = task_description

            print(f"Single task dictionsary: {single_tasks_dict}") 

        print(f"Single robot list: {singlerobot_list}")
        # **Ensure singlerobot_list contains all robots from Independent Tasks**
        singlerobot_list = list(set(
            robot for robot in singlerobot_list
            if robot.lower() not in ["singlerobot_list", "multirobot_list"]
        ))
        print(f"Final Singlerobot List: {singlerobot_list}")
        
        for robot in singlerobot_list:
            robot_lower = robot.replace(" ", "_").lower()
            if robot_lower not in self.robot_tasks:
                self.robot_tasks[robot_lower] = ""


        print(f"Robot Tasks: {self.robot_tasks}")
        return multirobot_list, singlerobot_list, single_tasks_dict

    # --------------------------------------------------------------------------
    # LLM CALL
    # --------------------------------------------------------------------------
    def call_openai(self, messages, model_="gpt-4o" ,debug_label=""):
        """
        Just calls openai.chat.completions with logs.
        """
        # print(f"Waffle State {self.waffle_state}")
        # print(f"Quadruped State {self.quadruped_state}")

        self.get_logger().info(f"[{debug_label}] Sending to GPT with full conversation.")
        try:
            # solved some OpenAI error: Error code: 400 - {'error': {'message': "Invalid type for 'messages[0].content[0]': expected an object, but got a string instead.", 'type': 'invalid_request_error', 'param': 'messages[0].content[0]', 'code': 'invalid_type'}}
            # Patch messages so all content is valid
            for m in messages:
                if isinstance(m["content"], list):
                    m["content"] = [{"type": "text", "text": str(x)} for x in m["content"]]
                elif not isinstance(m["content"], str):
                    m["content"] = str(m["content"])

            response = openai.chat.completions.create(
                model=model_,
                messages=messages,
                max_tokens=600,
                temperature=0.5,
            )

            content = response.choices[0].message.content.strip()
            self.get_logger().info(f"[{debug_label}] GPT raw output:\n{content} \n\n")
            return content
        except Exception as e:
            self.get_logger().error(f"[{debug_label}] OpenAI error: {e}")
            return "Error from GPT."

    def call_openai_1(self, messages, model_="gpt-4o" ,debug_label=""):
        """
        Just calls openai.chat.completions with logs.
        """
        # print(f"Waffle State {self.waffle_state}")
        # print(f"Quadruped State {self.quadruped_state}")

        self.get_logger().info(f"[{debug_label}] Sending to GPT with full conversation.")
        try:
            # solved some OpenAI error: Error code: 400 - {'error': {'message': "Invalid type for 'messages[0].content[0]': expected an object, but got a string instead.", 'type': 'invalid_request_error', 'param': 'messages[0].content[0]', 'code': 'invalid_type'}}
            # Patch messages so all content is valid
            for m in messages:
                if isinstance(m["content"], list):
                    m["content"] = [{"type": "text", "text": str(x)} for x in m["content"]]
                elif not isinstance(m["content"], str):
                    m["content"] = str(m["content"])

            response = openai.chat.completions.create(
                model=model_,
                messages=messages,
                # max_completion_tokens=600,
                # temperature=0.5,
            )

            content = response.choices[0].message.content.strip()
            self.get_logger().info(f"[{debug_label}] GPT raw output:\n{content} \n\n")
            return content
        except Exception as e:
            self.get_logger().error(f"[{debug_label}] OpenAI error: {e}")
            return f"ERROR: {e}"
        
    def call_ollama(self, prompt, model_="llama3.2"):
        if not prompt:
            self.get_logger().error("Empty prompt passed to call_ollama. Aborting GPT call.")
            return "Error: Empty input."
        
        try:
            response: ChatResponse = chat(
                model=model_,
                messages=prompt
            )
            return response.message.content
        except Exception as e:
            self.get_logger().error(f"Ollama API failed: {e}")
            return f"ERROR: {e}"


    # def call_ollama(self, prompt, model_="llama3.2"):
    #     url = "http://localhost:11434/api/chat"
    #     payload = {
    #         "model": model_,
    #         "messages": prompt,
    #         "stream": False
    #     }

    #     response = requests.post(url, json=payload)
    #     return response.json()["message"]["content"]

    def call_gemini(self, prompt, model_="gemini-2.0-flash"):

        try:   
            client = genai.Client(api_key=gemini_api_key)

            response = client.models.generate_content(
                model = model_,
                contents = self.to_gemini_messages(prompt),
                config = types.GenerateContentConfig(
                    max_output_tokens = 500,
                    temperature = 0.5
                )
            )

            return response.text
        except Exception as e:
            self.get_logger().error(f"Gemini API failed: {e}")
            return f"ERROR: {e}"
        
    def call_xai(self, prompt, model_='grok-3'):
        client = OpenAI(
            api_key=xai_api_key,
            base_url="https://api.x.ai/v1"
        )

        # Make the API call
        try:
            response = client.chat.completions.create(
                model="grok-3",  # Use the grok-beta model (check xAI docs for available models)
                messages=prompt,
                temperature=0.7  # Controls randomness; adjust as needed
            )

            return response.choices[0].message.content
        except Exception as e:
            return f"Error: {e}"
            

    def to_gemini_messages(self, openai_messages):

        system_parts  = []
        gem_messages  = []

        for m in openai_messages:
            role, text = m["role"], str(m["content"])

            if role == "system":
                system_parts.append(text)

            elif role == "user":
                gem_messages.append({"role": "user", "parts": [{"text": text}]})

            elif role in ("assistant", "model"):
                gem_messages.append({"role": "model", "parts": [{"text": text}]})

            else:  # tool / function / anything unknown: treat as user
                gem_messages.append({"role": "user", "parts": [{"text": text}]})

        # prepend the merged system prompt as first user msg
        if system_parts:
            gem_messages.insert(
                0,
                {"role": "user", "parts": [{"text": "\n\n".join(system_parts)}]},
            )

        return gem_messages


    # --------------------------------------------------------------------------
    #  PARTIAL UPDATE
    # --------------------------------------------------------------------------
    def update_all_robot_tasks(self, gpt_text: str):
        """
        Dynamically extracts task assignments from GPT output and updates the robot_tasks dictionary.
        Retains previous tasks unless explicitly changed.
        """
        # self.get_logger().info(f"[TaskManager] Parsing GPT output:\n{gpt_text}")

        print(" Inside Update All Robot Tasks")
        self.sequence_of_tasks = ""

        if "ask human for help" in gpt_text.lower():
            self.get_logger().warn("[TaskManager] GPT says no valid plan. Wiping tasks.")
            for r in self.robot_tasks:
                self.robot_tasks[r] = ""
            return

        updated_tasks = {}

        lines = gpt_text.split("\n")
        for line in lines:
            stripped = line.strip()
            if not stripped:
                self.get_logger().info("[TaskManager] Empty line, skipping.")
                continue

            self.get_logger().info(f"[TaskManager] Processing line: {stripped}")

            lower_line = stripped.lower().lstrip("- ").strip()

            self.get_logger().info(f"[TaskManager] Lowercase line: {lower_line}")

            # Check if it's a sequence line
            if lower_line.startswith("sequence:"):
                _, seq_text = stripped.split(":", 1)
                self.sequence_of_tasks = seq_text.strip()
                self.get_logger().info(f"[TaskManager] Sequence of tasks updated: {self.sequence_of_tasks}")
                continue
            
            print(f"Robot task: {self.robot_tasks}")
            # Check for any robot-specific task assignment
            for robot in self.robot_tasks:
                display_name = robot.replace(" ", "_").lower() + " tasks:"
                self.get_logger().info(f"[TaskManager] Checking robot: {robot}, display name: {display_name}")
                if lower_line.startswith(display_name):
                    try:
                        _, task_text = stripped.split(":", 1)
                        updated_tasks[robot] = task_text.strip()
                        self.get_logger().info(f"[TaskManager] Updated task for {robot}: {updated_tasks[robot]}")
                    except ValueError:
                        pass  # malformed line, skip
        
        # Update updated_tasks based on matching normalized names
        for name, task in self.single_tasks_dict.items():
            normalized_name = name.lower().replace(" ", "_")

            for existing_key in self.robot_tasks:
                if existing_key.lower().replace(" ", "_") == normalized_name:
                    updated_tasks[existing_key] = task
                    break  # stop once match is found

        print(f"Updated Tasks: {updated_tasks}")

        # Merge updates without deleting old tasks
        for robot, task in updated_tasks.items():
            self.robot_tasks[robot] = task

        self.get_logger().info(f"[TaskManager] Updated Tasks -> {self.robot_tasks}, Sequence: {self.sequence_of_tasks}")

    # --------------------------------------------------------------------------
    #  PUBLISHING CHAT OUTPUT
    # --------------------------------------------------------------------------
    def publish_chat_output(self, ai_output: str,source):
        """
        Cleans the GPT output (removes '#' and '*') and publishes it to /chat/input.
        """
        cleaned_output = re.sub(r"[\#\*]", "", ai_output)
        chat_msg = String()
        chat_msg.data = cleaned_output
        self.pub_chat_input.publish(chat_msg)
        # self.get_logger().info(f"[TaskManager] Published cleaned GPT output:\n{cleaned_output}")

    # --------------------------------------------------------------------------
    #  BUILD MESSAGES (FULL HISTORY, NO TRUNCATION)
    # --------------------------------------------------------------------------
    def build_messages(self):
        """
        Convert the entire conversation log to GPT messages.
        """
        messages = []
        self.get_logger().info(f"   Conversation log: \n {self.conversation_log}")
        for line in self.conversation_log:
            parts = line.split("] ", 1)
            if len(parts) < 2:
                continue
            role_content = parts[1]
            sub = role_content.split(":", 1)
            if len(sub) < 2:
                continue

            role_str = sub[0].strip().lower()
            content_str = sub[1].strip()
            
            # raw_content = sub[1].strip()

            # # Fix malformed list-style content
            # if raw_content.startswith("[") and raw_content.endswith("]"):
            #     try:
            #         content_str = " ".join(ast.literal_eval(raw_content))  # safely convert list to string
            #     except:
            #         content_str = raw_content  # fallback
            # else:
            #     content_str = raw_content

            self.get_logger().info(f"[debug_label]: Messages being sent to GPT: {messages}")

            if role_str == "human":
                role = "user"
            # elif role_str in ["quadruped", "ur5_chef", "ur5_helper", "taskmanager"]:
            #     role = "assistant"
            else:
                # Manager or unknown => system
                role = "system"

            messages.append({"role": role, "content": content_str})

            self.get_logger().info(json.dumps(messages, indent=2))

            # self.get_logger().info(f"current task : {content_str}\n")
            # self.history_task = f"Time of the task assisgned :{self.current_time}; Task : {content_str}"
            # self.history_task += f"[Task Assigned] {self.current_time} | [Task]: {content_str}\n"
            # self.get_logger().info(f"All the past tasks with time \n {self.history_task}")
            # self.get_logger().info(f"\n\n\n History of tasks ")

        self.get_logger().info(json.dumps(messages, indent=2))
        return messages



def main():
    rclpy.init()
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[TaskManager] Shutting down....")
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()