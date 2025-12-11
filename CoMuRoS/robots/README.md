# Robot Packages - Multi-Robot System

This directory contains the robot-specific packages for the CoMuRoS (Collaborative Multi-Robot System) project. These packages handle robot simulation, control, and LLM-based task execution for robots.

## Package Structure

```
CoMuRoS/CoMuRoS/robots/
├── multi_robot/                        # Multi-robot Ignition/Gazebo simulation environment
│   ├── launch/                         # multi_robot.launch.py
│   ├── env/                            # Ignition resource paths
│   ├── models/                         # Food court assets (tables, chairs, humans, food, props...)
│   ├── worlds/                         # foodcourt_world.sdf + turtlebot3 demo worlds
│   ├── rviz/                           # Multi-robot RViz config
│   └── package.xml
│
├── x3_uav/                             # X3 UAV drone packages
│   ├── x3_uav_description/             # URDF, meshes, RViz configs
│   │   ├── urdf/ 
│   │   ├── meshes/ 
│   │   ├── launch/ 
│   │   └── rviz/
│   │
│   ├── x3_uav_ignition/                # Ignition simulation + ROS-GZ bridge
│   │   ├── config/ 
│   │   ├── launch/ 
│   │   └── worlds/
│   │
│   └── x3_uav_llm/                     # LLM + controllers for UAV
│       ├── x3_uav_llm/ (LLM node, controllers)
│       ├── data/ (chat + task logs)
│       ├── test/ (pytest files)
│       └── setup.py / package.xml
│
└── yahboom/                            # Yahboom Rosmaster X3 ground robots
    ├── yahboom_rosmaster_description/  # URDF + meshes + RViz configs
    │   ├── urdf/
    │   ├── meshes/
    │   ├── launch/
    │   └── config/
    │
    ├── yahboom_rosmaster_gazebo/       # Gazebo worlds + models for Yahboom
    │   ├── models/
    │   ├── launch/
    │   ├── config/ 
    │   └── worlds/
    │
    └── yahboom_llm/                    # LLM + holonomic controllers
        ├── yahboom_llm/ (LLM interface + controller)
        ├── data/ (chat + task logs)
        ├── test/ (pytest files)
        └── setup.py / package.xml

```

---

## Running the Complete System

To run the full multi-robot system, execute these four launch commands **in separate terminals**:

### Terminal 1: Launch Simulation Environment
```bash
ros2 launch multi_robot multi_robot.launch.py
```
**What it does:**
- Starts Ignition Gazebo with the food court environment
- Spawns 3 robots with delays:
  - **r1** (Cleaning Robot) at position (11.0, 0.0, 0.05) after 5 seconds
  - **r2** (Delivery Robot) at position (0.0, 0.0, 0.05) after 15 seconds  
  - **r3** (Drone) at position (-1.0, 1.0, 0.05) after 25 seconds
- Sets up ROS-Gazebo bridge for `/clock` and `/tf` topics
- Opens RViz for multi-robot visualization

### Terminal 2: Launch Robot LLM Interfaces
```bash
ros2 launch robot_llm robot_llms.launch.py
```
**What it does:**
- Starts LLM interface nodes for all robots:
  - `cleaning_bot_llm_node` - Interprets commands for cleaning robot
  - `delivery_bot_llm_node` - Interprets commands for delivery robot
  - `drone_llm_node` - Interprets commands for the drone
- Each node subscribes to task messages from the chat system
- Converts high-level natural language commands into robot-specific actions

### Terminal 3: Launch Robot Control Services
```bash
ros2 launch robot_llm robot_services.launch.py
```
**What it does:**
- Starts position controller services for each robot:
  - `cleaning_bot_position_controller` (namespace: r1) - Holonomic controller for robot 1
  - `deliver_bot_position_controller` (namespace: r2) - Holonomic controller for robot 2
  - `drone_position_controller` (namespace: r3) - Drone flight controller
- Provides low-level motion control services
- Executes movement commands and trajectories

### Terminal 4: Launch Chat Interface
```bash
ros2 launch chatty chat_system.launch.py
```
**What it does:**
- Starts the natural language chat interface GUI
- Launches chat manager for conversation history
- Starts task manager to convert natural language to robot tasks using LLMs
- Enables optional audio input/output (microphone and text-to-speech)
- Publishes time information for task scheduling

**Optional parameters:**
```bash
# Use specific LLM model (default: 4)
ros2 launch chatty chat_system.launch.py model:=1

# Use custom robot configuration
ros2 launch chatty chat_system.launch.py config_file:=robot_config_roscon_2025

# Enable voice input/output
ros2 launch chatty chat_system.launch.py enable_audio_input:=true enable_audio_output:=true
```

---

## Example Commands

Once all four terminals are running, use natural language in the chat interface:


- "Clean the place"
- "Serve the veg food to table 3"
- "Which table is empty ?"
- "Where is the child sitting ?"

---

## Building the Packages

### Build All Packages
```bash
cd ~/ros2_ws/src
colcon build --symlink-install
source install/setup.bash
```

---

## Adding New Robots

### Step 1: Add Robot to Simulation

Edit `multi_robot/launch/multi_robot.launch.py` and add your robot launch:

```python
robot4_launch = TimerAction(
    period=30.0,  # Spawn delay in seconds
    actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(your_robot_launch_path),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'robot_name': 'your_robot_name',
                'prefix': 'r4',
                'spawn_x': '5.0',
                'spawn_y': '5.0',
                'spawn_z': '0.05',
                # ... other parameters
            }.items()
        )
    ]
)
```

Then add it to the LaunchDescription:
```python
return LaunchDescription([
    # ... existing launches
    robot4_launch  # Add your new robot
])
```

### Step 2: Create Robot Package Structure

Create a new package following this structure:
```
your_robot_llm/
├── data/
│   ├── chat_history.txt
│   └── robot_task_history.txt
├── launch/                         # Optional
├── package.xml
├── resource/
├── setup.cfg
├── setup.py
├── test/
└── your_robot_llm/
    ├── __init__.py
    ├── your_robot_llm.py           # LLM interface
    └── your_robot_controller.py    # Position controller service
```

### Step 3: Add LLM Node to robot_llms.launch.py

Edit `robot_llm/launch/robot_llms.launch.py`:

```python
your_robot_node = Node(
    package='your_robot_llm',
    executable='your_robot_llm',
    name='your_robot_llm_node',
    parameters=[{
        'robot_name': 'your_robot',
    }],
    output='screen',
)
```

Add to LaunchDescription:
```python
return LaunchDescription([
    cleaning_robot_node,
    delivery_bot_node,
    drone_node,
    your_robot_node,  # Add your new robot LLM
])
```

### Step 4: Add Controller Service to robot_services.launch.py

Edit `robot_llm/launch/robot_services.launch.py`:

```python
your_robot_controller = Node(
    package='your_robot_llm',
    executable='your_robot_position_controller',
    name='your_robot_position_controller',
    parameters=[{
        'namespace': 'r4',  # Match the prefix from multi_robot.launch.py
    }],
    output='screen',
)
```

Add to LaunchDescription:
```python
return LaunchDescription([
    holonomic_robot1,
    holonomic_robot2,
    drone_1,
    your_robot_controller,  # Add your new controller
])
```

### Step 5: Update Robot Configuration

Create or edit a configuration file in `chatty/config/`:

```json
{
  "robot_names": ["cleaning_bot", "delivery_bot", "drone", "your_robot"],
  "robot_capabilities": {
    "your_robot": "Description of your robot's capabilities"
  },
  "robot_morphology": {
    "your_robot": "Physical specifications"
  },
  "robot_states": {
    "your_robot": {
      "relative_position": "Starting position",
      "mode": "Idle"
    }
  }
}
```

### Step 6: Build and Test

```bash
cd ~/comuros
colcon build --packages-select your_robot_llm
source install/setup.bash

# Test the new robot
ros2 launch multi_robot multi_robot.launch.py
ros2 launch robot_llm robot_llms.launch.py
ros2 launch robot_llm robot_services.launch.py
ros2 launch chatty chat_system.launch.py
```

---
