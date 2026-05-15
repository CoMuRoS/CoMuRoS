#!/usr/bin/env python3
import json
import rclpy
import subprocess
from rclpy.node import Node
from std_msgs.msg import String,Bool
import subprocess
from geometry_msgs.msg import Twist
import os,signal
import time

class TaskManagerSubscriber(Node):
    def __init__(self): 
        super().__init__('task_manager_subscriber_go2')

        self.get_logger().info('Task Manager Node has been started')

        self.declare_parameter('use_sim', False)
        self.use_sim = self.get_parameter('use_sim').value

        self.subscription1 = self.create_subscription(String, '/task_manager/tasks_json', self.callback, 10 )
        # self.subscription2 = self.create_subscription(String, '/chat/output', self.callback_status, 10)
        self.subscription3 = self.create_subscription(String, '/chat/history', self.callback_history, 10)
        self.chat_status_sub = self.create_subscription(String, '/chat/task_status', self.callback_chat_status, 10)
        self.go2_callback = self.create_subscription(Bool, '/go2_goto_status', self.go2_goto_callback, 10)

        self.status_pub = self.create_publisher(String,"/chat/task_status",10)
        self.robot_state_pub = self.create_publisher(String, '/robot_states', 10)
        self.quadruped_state_pub = self.create_publisher(String, '/quadruped_task_status', 10)
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.initialise_empty()

        self.timer1 = self.create_timer(0.5, self.timrt_callback1)

        # self.interrupt_sub = self.create_subscription(String, '/task_manager/tasks_json', self.interrupt_callback, 10)
        self.interrupt = False
        self.task_inprogress = False
        self.task_complete = False

        self.interrupt_msg = ""
        self.in_prog_msg = ""
        self.task_comp_msg = ""
        self.quadruped_task = ""
        self.subproces_instance = None
        self.quad_old_task = None

    def go2_goto_callback(self,msg):
        if msg.data :
            self.write_completion_status()

    def write_completion_status(self): # Go2 goto completion file writing 
        with open("/home/vipul/ws/himura_ws/src/quadruped_sim/quadruped_sim/data.txt", "a") as f:
            f.write("Quadruped (status) : TASKS COMPLETE\n")  # Added newline for clarity
            f.flush()

    def callback_chat_status(self, msg):
        print("Receved /chat/task_status")
        if "Quadruped (status) : TASKS COMPLETE" == msg.data:
            print("Quadruped (status) : TASKS COMPLETE")
            self.quadruped_task_completed()

    def timrt_callback1(self):
        msg = String()

        if self.interrupt:
            msg.data = self.interrupt_msg
            self.quadruped_state_pub.publish(msg)

        if self.task_inprogress:
            msg.data = self.in_prog_msg
            self.quadruped_state_pub.publish(msg)
            
        if self.task_complete:
            msg.data = self.task_comp_msg
            self.quadruped_state_pub.publish(msg)

    def initialise_empty(self):
        print("Initialising Base Robot States")
        updated_data = {
            "go2": {
                "relative_position": "None",
                "relative_orientation": "None",
                "vertical_state": "Standing",
                "carry_state": "Empty"
            },
            "burger": {
                "relative_position": "None",
                "relative_orientation": "None"
            },
            "waffle": {
                "relative_position": "None",
                "relative_orientation": "None",
                "manipulator_state": "Home",
                "gripper_state": "Empty"
            }
        }
        
        # Convert to string and publish
        msg = String()
        msg.data = json.dumps(updated_data)
        # msg.data = "Hello World"
        self.robot_state_pub.publish(msg)
        self.get_logger().info(f"Published modified data: {msg.data}")

    def quadruped_interrupt(self):
        self.task_inprogress = False
        self.task_complete = False
        self.interrupt = True
        self.interrupt_msg = f"Quadruped (status) : {self.quadruped_task} : INTERRUPTED"

    def quadruped_in_progress(self):
        self.task_complete = False
        self.interrupt = False
        self.task_inprogress = True
        self.in_prog_msg = f"Quadruped (status)------->> : {self.quadruped_task} : IN PROGRESS"

    def quadruped_task_completed(self):
        self.task_inprogress = False
        self.interrupt = False
        self.task_complete = True
        self.task_comp_msg = f"Quadruped (status) : {self.quadruped_task} : TASKS COMPLETE"

    # def interrupt_callback(self, msg):
    #     try:
    #         data = json.loads(msg.data)  # Parse JSON
    #         self.msg_data = data
    #         # Extract the waffle task correctly
    #         quadru_task = data.get("robot_tasks", {}).get("quadruped", "").strip() or "No quadruped task found."

    #         if "stop" in quadru_task.lower():
    #             # publish new task topic as waffle task interrupted
    #             print("Received STOP command. Terminating subprocess.")
    #             self.subprocess_instance.terminate()
    #             self.subprocess_instance.wait()  # Ensure the process fully stops
    #             self.subprocess_instance = None  # Reset the process instance
    #             subprocess.run("pkill -f 'gnome-terminal'", shell=True)

    #             mssg = String()
    #             mssg.data = "Quadruped (status) : STOP TASKS COMPLETE"
    #             self.node_status.publish(mssg)

    #             self.waffle_tasks_interrupted()
                
    #         # else:
    #         #     self.interrupt = False
    #     except json.JSONDecodeError:
    #         self.get_logger().error("Failed to decode JSON message.")

    def remove_apostrophes(self, text):
        return text.replace("'", "")

    def execute_script(self, user_prompt):
        user_prompt = self.remove_apostrophes(user_prompt)
    	
        try:
            # Construct the command to run the Python script with two arguments
            self.get_logger().info(f'in execute script')
            if self.use_sim:
                command = (
                    f"gnome-terminal --disable-factory -- bash -c 'python3 /home/vipul/ws/himura_ws/src/quadruped_sim/quadruped_sim/llm-go2-argument-sim.py "
                    f"\"{user_prompt}\"; exec bash'"
                )
            
            else:
                command = (
                    f"gnome-terminal --disable-factory -- bash -c 'python3 /home/vipul/ws/himura_ws/src/quadruped_sim/quadruped_sim/llm-go2-argument-sim.py "
                    f"\"{user_prompt}\"; exec bash'"
                )


            self.subproces_instance = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

            print(f"PID is {self.subproces_instance.pid}")

            # cmd = "gnome-terminal --disable-factory -- bash -c 'while true; do echo Running; sleep 1; done; exec bash'"
            # p = subprocess.Popen(
            #     cmd,
            #     shell=True,
            #     preexec_fn=os.setsid            # ← start a new session/process group
            # )

            # # 2️⃣ Let it run for 5 seconds
            # time.sleep(5)

            # # 3️⃣ Kill the entire process group you just created
            # os.killpg(os.getpgid(p.pid), signal.SIGTERM)

            # Log the execution details
            self.get_logger().info(f'Executed llm-quadruped-argument.py  user prompt: {user_prompt}')
        except Exception as e:
            # Log any errors that occur
            self.get_logger().error(f"Failed to execute llm-quadruped-argument.py: {str(e)}")


    def callback(self, msg):
        try:
            data = json.loads(msg.data)  # Parse JSON


            quadruped_task = data.get("robot_tasks", {}).get("quadruped", "").strip() or "No quadruped task found."
            self.quadruped_task = quadruped_task
            if self.quadruped_task == "No quadruped task found.":
                self.get_logger().info("no task found for quadruped.")
                return
            elif "stop" in self.quadruped_task.lower() : 
                self.get_logger().info(f'Stopping all Tasks')
                if self.subproces_instance:
                    os.killpg(os.getpgid(self.subproces_instance.pid), signal.SIGTERM)

                # go2.publish_vel(0.0 ,0.0, 0.0) # Stop GO2
                t = Twist()
                t.linear.x = 0.0
                t.linear.y = 0.0
                t.angular.z = 0.0
                self.twist_publisher_.publish(t)

                msg = String()
                msg.data = "Quadruped (status) : STOP TASKS COMPLETE"
                self.status_pub.publish(msg)

                self.quadruped_interrupt()

                msg.data = f"Quadruped (status) : {quadruped_task} : INTERRUPTED"
                return

            # Log the extracted task
            self.get_logger().info(f'Quadruped task: {quadruped_task}')

            # if self.quad_old_task == quadruped_task or self.task_inprogress:
            #     return
            self.quadruped_in_progress()

            # Execute the Python script with the extracted task
            self.execute_script(quadruped_task)
            self.quad_old_task = quadruped_task

        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON message.")
    

    def callback_status(self, msg):
        try:
            data2 = msg.data
            self.get_logger().info(f'Latest Chat: {data2}')

            if "Unknown" in msg.data:
                print("I an human")
                if "Plan" in msg.data or "Independent Tasks" in msg.data:
                    print("over right")
                    # Open the file in write mode ('w') to delete existing content and add new text
                    with open("task_status.txt", "w") as file:
                        file.write("")
                    self.get_logger().warn("Task Status file has been erased ...")

        except Exception as e:
            self.get_logger().error(f"Failed to process chat output: {str(e)}")

    def callback_history(self, msg):
        try:
            data3 = msg.data
            self.get_logger().info(f'Chat history: {data3}')
        except Exception as e:
            self.get_logger().error(f"Failed to process chat history: {str(e)}")


def main(args=None):

    with open("/home/vipul/ws/himura_ws/src/quadruped_sim/quadruped_sim/data.txt", "w") as f:
        f.write("")
        f.flush()
    
    with open("elapsed_time_log.txt", "w") as f:
        f.write("0.0")
        f.flush()
        
    rclpy.init(args=args)
    node = TaskManagerSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  
    main()