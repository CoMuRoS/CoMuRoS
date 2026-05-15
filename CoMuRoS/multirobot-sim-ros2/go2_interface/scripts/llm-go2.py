import json
import rclpy
import subprocess
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class TaskManagerSubscriber(Node):
    def __init__(self):  # Fixed constructor
        super().__init__('task_manager_subscriber_go2')

        # Subscription to task manager JSON topic
        self.subscription1 = self.create_subscription(
            String,
            '/task_manager/tasks_json',
            self.callback,
            10  # Queue size
        )
        self.get_logger().info('Task Manager Subscriber node has started.')

        # Subscription to chat output
        self.subscription2 = self.create_subscription(
            String,
            '/chat/output',
            self.callback_status,
            10
        )
        
        self.interrupt = False
        self.task_inprogress = False
        self.task_complete = False

        self.interrupt_msg = ""
        self.in_prog_msg = ""
        self.task_comp_msg = ""
        self.quadruped_task = ""

        self.get_logger().info('Chat Output Subscriber node has started.')

        # Subscription to chat history (use a different topic to avoid duplication)
        self.subscription3 = self.create_subscription(
            String,
            '/chat/history',  # Changed to avoid subscribing to /chat/output twice
            self.callback_history,
            10
        )

        self.chat_status_sub = self.create_subscription(String, '/chat/task_status', self.callback_chat_status, 10)
        self.quadruped_state_pub = self.create_publisher(String, '/quadruped_task_status', 10)

        self.status_pub = self.create_publisher(String,"/chat/task_status",10)

        self.timer1 = self.create_timer(0.5, self.timrt_callback1)

        self.subproces_instance = None
        self.get_logger().info('Chat History Subscriber node has started.')


    def callback_chat_status(self, msg):
       

        if "Quadruped (status) : TASKS COMPLETE" == msg.data:
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


    def quadruped_interrupt(self):
        self.task_inprogress = False
        self.task_complete = False
        self.interrupt = True
        self.interrupt_msg = f"Quadruped (status) : {self.quadruped_task} : INTERRUPTED"

    def quadruped_in_progress(self):
        self.task_complete = False
        self.interrupt = False
        self.task_inprogress = True
        self.in_prog_msg = f"Quadruped (status) : {self.quadruped_task} : IN PROGRESS"

    def quadruped_task_completed(self):
        self.task_inprogress = False
        self.interrupt = False
        self.task_complete = True
        self.task_comp_msg = f"Quadruped (status) : {self.quadruped_task} : TASKS COMPLETE"


    def callback(self, msg):
        try:
            data = json.loads(msg.data)  # Parse JSON

            # Extract the waffle task
            # quadruped_task = data.get("quadruped", "").strip() or "No quadruped task found."
            quadruped_task = data.get("robot_tasks", {}).get("quadruped", "").strip() or "No quadruped task found."
            self.quadruped_task = quadruped_task
            if self.quadruped_task == "No quadruped task found.":
                self.get_logger().info("no task found for quadruped.")
                return
            elif "stop" in self.quadruped_task.lower() : 
                self.get_logger().info(f'Stopping all Tasks')
                if self.subproces_instance:
                    self.subproces_instance.terminate()
                    self.subproces_instance.wait()
                    subprocess.run("pkill -f 'llm-go2-argument6.py'", shell=True)

                    msg = String()
                    msg.data = "Quadruped (status) : STOP TASKS COMPLETE"
                    self.status_pub.publish(msg)

                    self.quadruped_interrupt()

                    msg.data = f"Quadruped (status) : {quadruped_task} : INTERRUPTED"
                    return

            # Log the extracted task
            self.get_logger().info(f'Quadruped task: {quadruped_task}')
            self.quadruped_in_progress()

            # Execute the Python script with the extracted task
            self.execute_script(quadruped_task)

        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON message.")

    # def execute_script(self, user_prompt):
    #     try:
    #         command = f"gnome-terminal -- bash -c 'python3 llm-quadruped-argument.py \"{user_prompt}\"; read -p \"Press Enter to exit...\"; exec bash'"
    #         subprocess.Popen(command, shell=True)
    #         self.get_logger().info(f'Executed llm-quadruped-argument.py with user prompt: {user_prompt}')
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to execute llm-quadruped-argument.py: {str(e)}")
    


    def execute_script(self, user_prompt):
    	
        try:
            # Construct the command to run the Python script with two arguments
            self.get_logger().info(f'in execute script')
            command = (
                f"gnome-terminal -- bash -c 'python3 /home/unitree/unitree_sdk2_python/example/go2/high_level/llm-go2-argument6.py "
                f"\"{user_prompt}\"; exec bash'"
            )
                # Run the command in a new terminal
            self.subproces_instance = subprocess.Popen(command, shell=True)
            
            # Log the execution details
            self.get_logger().info(f'Executed llm-quadruped-argument.py  user prompt: {user_prompt}')
        except Exception as e:
            # Log any errors that occur
            self.get_logger().error(f"Failed to execute llm-quadruped-argument.py: {str(e)}")


    def callback_status(self, msg):
        try:
            data2 = msg.data
            self.get_logger().info(f'Latest Chat: {data2}')
        except Exception as e:
            self.get_logger().error(f"Failed to process chat output: {str(e)}")

    def callback_history(self, msg):
        try:
            data3 = msg.data
            self.get_logger().info(f'Chat history: {data3}')
        except Exception as e:
            self.get_logger().error(f"Failed to process chat history: {str(e)}")


def main(args=None):

    with open("data.txt", "w") as f:
        f.write("")
        
    rclpy.init(args=args)
    node = TaskManagerSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  # Fixed main guard
    main()
