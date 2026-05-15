#!/usr/bin/env python3
import json
import rclpy
import subprocess
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class TaskManagerSubscriber(Node):
    def __init__(self, ns = ""):
        super().__init__('task_manager_subscriber_ur5', namespace=ns)
        self.ns = ns.lstrip("/")  # Remove leading '/' if present
        ns = ns.lstrip("/") 
        self.get_logger().info(f'Task Manager Node started in namespace: {ns}')

        self.declare_parameter('use_sim', False)
        self.use_sim = self.get_parameter('use_sim').value

        # Create topic subscriptions
        self.subscription1 = self.create_subscription(String, '/task_manager/tasks_json', self.callback, 10)
        self.chat_status_sub = self.create_subscription(String, '/chat/task_status', self.callback_chat_status, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/chat/task_status', 10)
        self.robot_state_pub = self.create_publisher(String, '/robot_states', 10)
        self.ur5_helper_state_pub = self.create_publisher(String, '/ur5_helper_task_status', 10)
        self.ur5_chef_state_pub = self.create_publisher(String, '/ur5_chef_task_status', 10)

        # Robot task and state dictionaries
        self.robot_states = {
            ns: {
                "interrupt": False,
                "in_progress": False,
                "complete": False,
                "interrupt_msg": "",
                "in_prog_msg": "",
                "comp_msg": "",
                "current_task": "",
                "old_task": "",
                "subprocess": None
            }
        }
        self.ns_role = {
            "arm1": "UR5 Chef",
            "arm2": "UR5 Helper"
        }
        self.role = self.ns_role.get(self.ns, f"UR5 ({self.ns})")

        # self.initialise_empty()
        self.timer1 = self.create_timer(0.5, self.timer_callback1)


    def callback_chat_status(self, msg):
        if f"{self.role} (status) : TASKS COMPLETE" == msg.data:
            self.ur5_task_completed()

    def timer_callback1(self):
        ns = self.ns
        msg = String()
        state = self.robot_states[ns]

        if state["interrupt"]:
            msg.data = state["interrupt_msg"]
            self.robot_state_pub.publish(msg)

        if state["in_progress"]:
            msg.data = state["in_prog_msg"]
            self.robot_state_pub.publish(msg)

        if state["complete"]:
            msg.data = state["comp_msg"]
            self.robot_state_pub.publish(msg)


    # def initialise_empty(self):
    #     print("Initialising Base Robot States")
    #     updated_data = {
    #         "go2": {
    #             "relative_position": "None",
    #             "relative_orientation": "None",
    #             "vertical_state": "Standing",
    #             "carry_state": "Empty"
    #         },
    #         "burger": {
    #             "relative_position": "None",
    #             "relative_orientation": "None"
    #         },
    #         "waffle": {
    #             "relative_position": "None",
    #             "relative_orientation": "None",
    #             "manipulator_state": "Home",
    #             "gripper_state": "Empty"
    #         }
    #     }
        
    #     # Convert to string and publish
    #     msg = String()
    #     msg.data = json.dumps(updated_data)
    #     # msg.data = "Hello World"
    #     self.robot_state_pub.publish(msg)
    #     self.get_logger().info(f"Published modified data: {msg.data}")


    def ur5_interrupt(self):
        ns = self.ns
        state = self.robot_states[ns]
        state["in_progress"] = False
        state["complete"] = False
        state["interrupt"] = True
        state["interrupt_msg"] = f"{self.role} (status) : {state['current_task']} : INTERRUPTED"

    def ur5_in_progress(self):
        ns = self.ns
        state = self.robot_states[ns]
        state["complete"] = False
        state["interrupt"] = False
        state["in_progress"] = True
        state["in_prog_msg"] = f"{self.role} (status) : {state['current_task']} : IN PROGRESS"

    def ur5_task_completed(self):
        ns = self.ns
        state = self.robot_states[ns]
        state["in_progress"] = False
        state["interrupt"] = False
        state["complete"] = True
        state["comp_msg"] = f"{self.role} (status) : {state['current_task']} : TASKS COMPLETE"



    def remove_apostrophes(self, text):
        return text.replace("'", "")

    def execute_script(self, user_prompt):
        ns = self.ns
        state = self.robot_states[ns]
        user_prompt = self.remove_apostrophes(user_prompt)
        print(ns)
        print(type(ns))
        try:
            self.get_logger().info('In execute_script')
            command = (
                f"gnome-terminal -- bash -c 'python3 /home/vipul/ws/mia_ws/src/robot_codes/robot_codes/llm-ur5-argument.py "
                f"\"{user_prompt}\" --ns {ns}; exec bash'"
            )

            state["subprocess"] = subprocess.Popen(command, shell=True)
            self.get_logger().info(f'Executed llm-ur5-argument.py with prompt: {user_prompt}, ns: {ns}')

        except Exception as e:
            self.get_logger().error(f"Failed to execute llm-ur5-argument.py: {str(e)}")


    def callback(self, msg):
        ns = self.ns
        state = self.robot_states[ns]

        try:
            data = json.loads(msg.data)

            # Choose task key based on role/namespace
            role_task_key_map = {
                "arm1": "ur5_chef",
                "arm2": "ur5_helper"
            }

            task_key = role_task_key_map.get(ns, None)
            if task_key is None:
                self.get_logger().warn(f"No valid task key mapping found for namespace '{ns}'")
                return

            ur5_task = data.get("robot_tasks", {}).get(task_key, "").strip() or f"No {self.role} task found."
            state["current_task"] = ur5_task

            if ur5_task == f"No {self.role} task found.":
                self.get_logger().info(f"No task found for {self.role}.")
                return

            if "stop" in ur5_task.lower():
                self.get_logger().info(f"Stopping all {self.role} tasks")
                if state["subprocess"]:
                    state["subprocess"].terminate()
                    state["subprocess"].wait()
                    subprocess.run("pkill -f 'gnome-terminal'", shell=True)
                stop_msg = String()
                stop_msg.data = f"{self.role} (status) : STOP TASKS COMPLETE"
                self.status_pub.publish(stop_msg)
                self.ur5_interrupt()
                return

            self.get_logger().info(f"{self.role} task: {ur5_task}")

            if state["old_task"] == ur5_task or state["in_progress"]:
                return

            self.ur5_in_progress()
            self.execute_script(ur5_task)
            state["old_task"] = ur5_task

        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON message.")


def main(args=None):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--ns', type=str, default="")
    cli_args = parser.parse_args()

    with open("/home/vipul/ws/mia_ws/src/robot_codes/robot_codes/data1.txt", "w") as f:
        f.write("")
        f.flush()

    with open("/home/vipul/ws/mia_ws/src/robot_codes/robot_codes/data2.txt", "w") as f:
        f.write("")
        f.flush()
    
    rclpy.init(args=args)
    node = TaskManagerSubscriber(ns=cli_args.ns)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  
    main()