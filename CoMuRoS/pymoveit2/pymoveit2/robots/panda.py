# from typing import List

# MOVE_GROUP_ARM: str = "arm"
# MOVE_GROUP_GRIPPER: str = "gripper"

# OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
# CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


# def joint_names(prefix: str = "panda_") -> List[str]:
#     return [
#         prefix + "joint1",
#         prefix + "joint2",
#         prefix + "joint3",
#         prefix + "joint4",
#         prefix + "joint5",
#         prefix + "joint6",
#         prefix + "joint7",
#     ]


# def base_link_name(prefix: str = "panda_") -> str:
#     return prefix + "link0"


# def end_effector_name(prefix: str = "panda_") -> str:
#     return prefix + "hand_tcp"


# def gripper_joint_names(prefix: str = "panda_") -> List[str]:
#     return [
#         prefix + "finger_joint1",
#         prefix + "finger_joint2",
#     ]

from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.0]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.80285]


def joint_names(prefix: str = "") -> List[str]:
    return [        
        prefix + "shoulder_pan_joint",
        prefix + "shoulder_lift_joint",
        prefix + "elbow_joint",
        prefix + "wrist_1_joint",
        prefix + "wrist_2_joint",
        prefix + "wrist_3_joint",
    ]


def base_link_name(prefix: str = "") -> str:
    return prefix + "base_link"


def end_effector_name(prefix: str = "") -> str:
    return prefix + "wrist_3_link"


def gripper_joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "robotiq_85_left_knuckle_joint",        
    ]
