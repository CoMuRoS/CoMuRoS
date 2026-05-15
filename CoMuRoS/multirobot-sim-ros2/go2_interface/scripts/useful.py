import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)
import math
from dataclasses import dataclass

@dataclass
class TestOption:
    name: str
    id: int
    description: str
    example_code: str

# Updated TestOption list with new functions
option_list = [
    TestOption(name="damp", id=0, description="Reduce all movement of the robot to a damped state.", example_code="sport_client.Damp()"),
    TestOption(name="stand", id=1, description="Make the robot transition to a standing position.", example_code="stand(sport_client)"),
    TestOption(name="sit", id=2, description="Make the robot transition to a sitting or lying position.", example_code="sport_client.StandDown()"),
    TestOption(name="move forward", id=3, description="Move the robot forward at a specified speed for a duration.", example_code="move_linear(sport_client, 0.3, 0, 2)"),
    TestOption(name="move lateral", id=4, description="Move the robot laterally to the left for a duration.", example_code="move_linear(sport_client, 0, 0.3, 2)"),
    TestOption(name="move rotate", id=5, description="Rotate the robot on the spot at a specified angular velocity for a duration.", example_code="rotate_in_place(sport_client, 0.5, 2)"),
    TestOption(name="stop_move", id=6, description="Stop all robot motion.", example_code="sport_client.StopMove()"),
    TestOption(name="switch_gait 0", id=7, description="Switch the robot's gait to type 0.", example_code="sport_client.SwitchGait(0)"),
    TestOption(name="switch_gait 1", id=8, description="Switch the robot's gait to type 1.", example_code="sport_client.SwitchGait(1)"),
    TestOption(name="balanced stand", id=9, description="Make the robot assume a balanced standing posture.", example_code="sport_client.BalanceStand()"),
    TestOption(name="recovery", id=10, description="Recover the robot to a safe standing posture.", example_code="sport_client.RecoveryStand()"),
    TestOption(name="move back", id=11, description="Move the robot backward for a duration.", example_code="move_linear(sport_client, -0.3, 0, 2)"),
    TestOption(name="move right", id=12, description="Move the robot to the right for a duration.", example_code="move_linear(sport_client, 0, -0.3, 2)"),
    TestOption(name="turn left", id=14, description="Rotate the robot counterclockwise at a specified angular velocity for a duration.", example_code="rotate_in_place(sport_client, -0.5, 2)"),
    TestOption(name="turn right", id=15, description="Rotate the robot clockwise at a specified angular velocity for a duration.", example_code="rotate_in_place(sport_client, 0.5, 2)"),
    TestOption(name="goto_position", id=16, description="Move the robot to a specific relative position in the given time.", example_code="goto_position(sport_client, 1, 1, 5)"),
    TestOption(name="rotate_to_yaw_angle", id=17, description="Rotate the robot to a specific yaw angle in radians within the given time.", example_code="rotate_to_yaw_angle(sport_client, 1.57, 5)"),
    TestOption(name="soft_turn", id=18, description="Make a soft turn while also moving linearly at a specific speed for a specific duration.", example_code="soft_turn(sport_client, 0.1, 0.4, duration)"),
    TestOption(name="circle", id=19, description="Make the robot perform one revolution on a circle at a specified radius for certain duration.", example_code="circle(sport_client, radius=1, duration=20)"),
    TestOption(name="square_trajectory", id=20, description="Make the robot move in a square trajectory without yawing.", example_code="square_trajectory_no_yaw(sport_client, side_length=2, speed=0.3)"),
]

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
               description="Command the robot to move forward with a fixed velocity for 2 seconds.",
               example_code="controller.move_forward()"),

    TestOption(name="move_backward", id=4,
               description="Command the robot to move backward with a fixed velocity for 2 seconds.",
               example_code="controller.move_backward()"),

    TestOption(name="stop", id=5,
               description="Immediately stop all robot movement and cancel current motion.",
               example_code="controller.stop()"),
]

class UserInterface:
    def __init__(self):
        self.test_option_ = None

    def convert_to_int(self, input_str):
        try:
            return int(input_str)
        except ValueError:
            return None

    def terminal_handle(self):
        input_str = input("Enter id or name: \n")

        if input_str == "list":
            self.test_option_.name = None
            self.test_option_.id = None
            for option in option_list:
                print(f"{option.name}, id: {option.id}")
            return

        for option in option_list:
            if input_str == option.name or self.convert_to_int(input_str) == option.id:
                self.test_option_.name = option.name
                self.test_option_.id = option.id
                print(f"Test: {self.test_option_.name}, test_id: {self.test_option_.id}")
                return

        print("No matching test option found.")
        
def move_linear(sport_client, vx, vy, duration):
    """
    Continuously move the robot linearly at a specific speed for a specific duration.
    Args:
        sport_client: Instance of SportClient.
        vx: Linear velocity in the x-direction (forward/backward).
        vy: Linear velocity in the y-direction (left/right).
        duration: How long to move in seconds.
    """
    start_time = time.time()
    while time.time() - start_time < duration:
        sport_client.Move(vx, vy, 0)  # Continuously send motion command
        time.sleep(0.1)  # Small sleep to avoid spamming the API
    sport_client.StopMove()  # Stop after moving

def rotate_in_place(sport_client, w, duration):
    """
    Continuously turn/yaw the robot linearly at a specific speed for a specific duration.
    Args:
        sport_client: Instance of SportClient.z
        w: yaw rate (left/right).
        duration: How long to move in seconds.
    """
    start_time = time.time()
    while time.time() - start_time < duration:
        sport_client.Move(0, 0, w)  # Continuously send motion command
        time.sleep(0.1)  # Small sleep to avoid spamming the API
    sport_client.StopMove()  # Stop after moving

def soft_turn(sport_client, vx, w, duration):
    """
    make a soft turn while also moving linearly at a specific speed for a specific duration.
    Args:
        sport_client: Instance of SportClient.
        vx: Linear velocity in the x-direction (forward/backward).
        w: yaw rate (left/right).
        duration: How long to move in seconds.
    """
    start_time = time.time()
    while time.time() - start_time < duration:
        sport_client.Move(vx, 0, w)  # Continuously send motion command
        time.sleep(0.1)  # Small sleep to avoid spamming the API
    sport_client.StopMove()  # Stop after moving


import numpy as np
def goto_position(sport_client, x, y, duration):
    """
    Continuously move the robot linearly for a specific duration to reach specific position in given time.
    Args:
        sport_client: Instance of SportClient.
        x: relative longitudinal position (forward/backward).
        y: relative lateral (left/right).
        duration: How long to move in seconds.
    """
    import math
    start_time = time.time()
    while time.time() - start_time < duration:
        vx = x/duration
        vy = y/duration
        if abs(vx)>=0.2:
            vx = 0.2*np.sign(vx)
        if abs(vy) >= 0.4:
            vy = 0.2*np.sign(vy)
        sport_client.Move(x/duration, y/duration, 0)  # Continuously send motion command
        time.sleep(0.1)  # Small sleep to avoid spamming the API
    sport_client.StopMove()  # Stop after moving


def rotate_to_yaw_angle(sport_client, theta, duration):
    """
    Continuously rotate/yaw the robot for a specific duration to reach specific angle in given time.
    Args:
        sport_client: Instance of SportClient.
        theta: relative angle (counterclockwise/clockwise) in radians.
        duration: How long to rotate in seconds.
    """
    start_time = time.time()
    while time.time() - start_time < duration:
        w = theta/duration
        if abs(w)>=0.6:
            w = 0.6*math.sign(w)
        sport_client.Move(0,0, w)  # Continuously send motion command
        time.sleep(0.1)  # Small sleep to avoid spamming the API
    sport_client.StopMove()  # Stop after moving


def circle(sport_client, radius=0.5, duration=20):
     """
    Before executing this make sure robot has reached to a point on  circumference and has orientation tangential to desired circle. Perform one revolution of the center at user defined radius.  
    Args:
        sport_client: Instance of SportClient.
        radius: move on a circle of radius for some duration.
        
     """ 
     w = (2*3.14)/duration
     vx= w*radius
     start_time = time.time()
     if abs(w)>=0.6:
        w = 0.6*math.sign(w)
     while time.time() - start_time < duration:
        sport_client.Move(vx,0, w)  # Continuously send motion command
        time.sleep(0.1)  # Small sleep to avoid spamming the API
     sport_client.StopMove()  # Stop after moving


def stand(sport_client):
    time.sleep(5.0)
    sport_client.StandUp()
    sport_client.BalanceStand()

def square_trajectory_no_yaw(sport_client,side_length, speed):
    """
    Make the robot follow a square trajectory without yawing.
    Args:
        sport_client: Instance of SportClient.
        side_length: Length of each side of the square (meters).
        speed: Speed at which the robot moves (m/s).
    """
    
    # Calculate time needed to traverse one side
    duration = side_length / speed

    # Move forward
    print("Moving forward")
    move_linear(sport_client, vx=speed, vy=0, duration=duration)

    # Move left
    print("Moving left")
    move_linear(sport_client, vx=0, vy=speed, duration=duration)

    # Move backward
    print("Moving backward")
    move_linear(sport_client, vx=-speed, vy=0, duration=duration)

    # Move right
    print("Moving right")
    move_linear(sport_client, vx=0, vy=-speed, duration=duration)


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    ChannelFactoryInitialize(0, sys.argv[1])

    # Initialize TestOption with all required fields
    test_option = TestOption(name=None, id=None, description=None, example_code=None)
    user_interface = UserInterface()
    user_interface.test_option_ = test_option

    sport_client = SportClient()  
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    while True:
        user_interface.terminal_handle()

        print(f"Updated Test Option: Name = {test_option.name}, ID = {test_option.id}\n")

        if test_option.id == 0:
            sport_client.Damp()
        elif test_option.id == 1:
            stand(sport_client)
        elif test_option.id == 2:
            sport_client.StandDown()
        elif test_option.id == 3:
            move_linear(sport_client, -0.3, 0, 2)
        elif test_option.id == 4:
            move_linear(sport_client, 0, 0.3, 2)
        elif test_option.id == 5:
            rotate_in_place(sport_client, 0.5, 2)
        elif test_option.id == 6:
            sport_client.StopMove()
        elif test_option.id == 7:
            sport_client.SwitchGait(0)
        elif test_option.id == 8:
            sport_client.SwitchGait(1)
        elif test_option.id == 9:
            sport_client.BalanceStand()
        elif test_option.id == 10:
            sport_client.RecoveryStand()
        elif test_option.id == 11:
            move_linear(sport_client, -0.3, 0, 2)
        elif test_option.id == 12:
            move_linear(sport_client, 0, -0.3, 2)
        elif test_option.id == 14:
            rotate_in_place(sport_client, -0.5, 2)
        elif test_option.id == 15:
            rotate_in_place(sport_client, 0.3, 4)
        elif test_option.id == 16:
            goto_position(sport_client, 1, 1, 5)
        elif test_option.id == 17:
            rotate_to_yaw_angle(sport_client, 1.57, 5)
        elif test_option.id == 18:
            soft_turn(sport_client, 1.57, 5)
        elif test_option.id == 19:
            circle(sport_client)
        elif test_option.id == 20:
            square_trajectory_no_yaw(sport_client, side_length=2, speed=0.3)

        time.sleep(1)
