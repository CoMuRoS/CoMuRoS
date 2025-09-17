Code for paper: LLM-Based Generalizable Hierarchical Task Planning and Execution for Heterogeneous Robot Teams with Event-Driven Replanning.

# CoMuRoS: Collaborative Multi-Robot System (`chatty` package)

A ROS 2 Python package that enables natural-language interaction with a team of homogeneous or heterogeneous robot/s through a chat-based interface using Large Language Models (LLMs). The system provides a GUI for interaction, a manager for tracking conversations, and a task manager that converts natural language into robot-executable plans.

## 🚀 Quick Start for New Users

**Want to get started immediately?** Follow these 4 simple steps:

1. **Clone & Build** (5 minutes)
2. **Set API Keys** (2 minutes) 
3. **Launch System** (1 command)
4. **Start Chatting** with your robots!

---

## Features

- **Natural Language Interface**: Chat with robots using everyday language
- **Multi-LLM Support**: Compatible with OpenAI, Ollama, Gemini, and XAI models  
- **GUI Interface**: User-friendly chat interface built with customtkinter
- **Persistent Chat History**: All conversations are saved and restored automatically
- **Heterogeneous Robot Support**: Works with different types of robots through configurable JSON files
- **Task Planning**: Converts natural language into structured robot tasks

## File Structure

```
chatty/
├── chatty/
│   ├── __init__.py
│   ├── chat_gui.py            # GUI interface using customtkinter
│   ├── chat_manager.py        # Manages chat history & routing
│   └── task_manager.py        # Converts chat to tasks using LLMs
├── config/                    # Robot configuration JSON files
│   └── robot_config_*.json  
├── data/  
│   └── chat_history.txt       # Stores all chat logs
├── launch/
│   └── chat_system.launch.py  # Launch file for all nodes
├── resource/
│   └── chatty
├── test/
│   └── test_*.py
├── package.xml
├── setup.py
├── setup.cfg
├── requirements.txt
└── README.md
```

---

## 📋 Installation Guide

### Step 1: Prerequisites
Make sure you have:
- **ROS 2** (Humble or Iron recommended)
- **Python 3.8+**
- **Git**

Check your ROS 2 installation:
```bash
ros2 --version
```

### Step 2: Clone the Repository
```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# Clone the repository
git clone <repository-url> chatty

# Navigate back to workspace root
cd ~/ros2_ws
```

### Step 3: Install Dependencies
```bash
# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install Python dependencies
cd src/chatty
pip install -r requirements.txt
```

### Step 4: Build the Package
```bash
# From your workspace root (~/ros2_ws)
colcon build --symlink-install

# Source the setup file
source install/setup.bash

# Add to your ~/.bashrc for permanent setup
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 🔐 Setting Up API Keys

**IMPORTANT**: You need at least one API key to use the system.

### Option 1: Temporary (Current Session Only)
```bash
export OPENAI_API_KEY="your_openai_key_here"
export GEMINI_API_KEY="your_gemini_key_here"
export XAI_API_KEY="your_grok_key_here"
```

### Option 2: Permanent (Recommended)
Add to your `~/.bashrc` file:
```bash
echo 'export OPENAI_API_KEY="your_openai_key_here"' >> ~/.bashrc
echo 'export GEMINI_API_KEY="your_gemini_key_here"' >> ~/.bashrc
echo 'export XAI_API_KEY="your_grok_key_here"' >> ~/.bashrc
source ~/.bashrc
```

### Where to Get API Keys:
- **OpenAI**: [platform.openai.com](https://platform.openai.com)
- **Gemini**: [ai.google.dev](https://ai.google.dev)
- **XAI (Grok)**: [x.ai](https://x.ai)

---

## 🎯 Running the System

### Method 1: Launch Everything at Once (Recommended)
```bash
# Basic launch with default settings
ros2 launch chatty chat_system.launch.py

# Launch with specific model and robot configuration
ros2 launch chatty chat_system.launch.py model:=1 config_file:=robot_config_example
```

### Method 2: Run Nodes Individually (Advanced)
Open 3 separate terminals:

**Terminal 1 - Chat Manager:**
```bash
ros2 run chatty chat_manager
```

**Terminal 2 - Task Manager:**
```bash
ros2 run chatty task_manager --ros-args -p model:=1 -p config_file:=robot_config_example
```

**Terminal 3 - GUI:**
```bash
ros2 run chatty chat_gui
```

---

## 🤖 Creating Your Own Robot Configuration

### Step 1: Copy Example Template
Copy the file robot_config_example.json and paste it in the same directory and thrn rename it

### Step 2: Edit Your Configuration
Open `robot_config_my_robots.json` and customize:

```json
{
  "robot_names": ["robot1", "robot2", "robot3"],

  "robot_capabilities": {
    "robot1": "Description of robot1 capabilities.",
    "robot2": "Description of robot2 capabilities.",
    "robot3": "Description of robot3 capabilities."
  },

  "robot_morphology": {
    "robot1": "Payload: 10 kg, Reach: 1.0 m, Speed: 0.5 m/s, Degrees of Freedom: 6",
    "robot2": "Payload: 15 kg, Reach: 1.2 m, Speed: 0.4 m/s, Degrees of Freedom: 6",
    "robot3": "Payload: 20 kg, Speed: 0–2 m/s, Size: 800 x 600 x 600 mm (L x W x H)"
  },

  "robot_states": {
    "robot1": {
      "relative_position": "Fixed",
      "relative_orientation": "Facing forward",
      "carry_state": "Empty",
      "arm_state": "Stowed",
      "mode": "Idle"
    },

    "robot2": {
      "relative_position": "Fixed",
      "relative_orientation": "Facing right",
      "carry_state": "Empty",
      "arm_state": "Stowed",
      "mode": "Idle"
    },

    "robot3": {
      "relative_position": "None",
      "relative_orientation": "None",
      "mobility_state": "Stationary",
      "carry_state": "Empty",
      "battery_state": "Full",
      "mode": "Idle"
    }

  },

  "task_specific_rules": [
    ""
  ],

  "task_replanning_rules": [
    ""
  ]
}

```

---

### Configuration Field Descriptions

- **robot_names**:  
    List of unique robot IDs in your team.  
    Example:  
    ```json
    "robot_names": ["robot1", "robot2", "robot3"]
    ```

- **robot_capabilities**:  
    Dictionary mapping each robot ID to a description of its abilities (e.g., manipulation, transport, inspection).  
    Example:  
    ```json
    "robot_capabilities": {
        "robot1": "Can pick and place objects up to 10kg.",
        "robot2": "Transports materials between stations.",
        "robot3": "Performs aerial inspection tasks."
    }
    ```

- **robot_morphology**:  
    Dictionary mapping each robot ID to its physical specifications (payload, reach, speed, size, etc.).  
    Example:  
    ```json
    "robot_morphology": {
        "robot1": "Payload: 10 kg, Reach: 1.0 m, Speed: 0.5 m/s, DOF: 6",
        "robot2": "Payload: 15 kg, Reach: 1.2 m, Speed: 0.4 m/s, DOF: 6",
        "robot3": "Payload: 2 kg, Speed: 2 m/s, Size: 800x600x600 mm"
    }
    ```

- **robot_states**:  
    Dictionary mapping each robot ID to its initial state (position, orientation, mode, etc.).  
    Example:  
    ```json
    "robot_states": {
        "robot1": {
            "relative_position": "Fixed",
            "relative_orientation": "Facing forward",
            "carry_state": "Empty",
            "arm_state": "Stowed",
            "mode": "Idle"
        }
    }
    ```

- **task_specific_rules**:  
    List of constraints or rules for task execution (e.g., "robot1 cannot lift over 10kg", "robot3 only operates outdoors").  
    Example:  
    ```json
    "task_specific_rules": [
        "robot1 cannot operate in wet environments",
        "robot2 must recharge after 2 hours"
    ]
    ```

- **task_replanning_rules**:  
    List of fallback or replanning strategies if a task fails (e.g., "If robot1 is busy, assign to robot2").  
    Example:  
    ```json
    "task_replanning_rules": [
        "If a robot is unavailable, reassign the task to the next available robot",
        "If battery low, send robot to charging station before resuming task"
    ]
    ```


### Step 3: Launch with Your Configuration
```bash
ros2 launch chatty chat_system.launch.py config_file:=robot_config_my_robots
```

---

## 🧠 Adding Your Own LLM Model

### Step 1: Edit task_manager.py
Open `chatty/chatty/task_manager.py` and find the model selection section.

### Step 2: Add Your Model
Add a new case in the model selection logic:
```python
elif self.model == 105:  # Choose an unused number
    ai_output = self.call_your_custom_llm(messages, model_="your-model-name")
    self.model_name = "your-model-name"
```

### Step 3: Implement Your Function
Add your custom function in the same file:
```python
def call_your_custom_llm(self, messages, model_="your-model-name"):
    """
    Your custom LLM implementation
    """
    try:
        # Your API call logic here
        # Return the response in the expected format
        return response
    except Exception as e:
        self.get_logger().error(f"Error with custom LLM: {e}")
        return None
```

### Step 4: Update Model Table
Add your model to the table below and rebuild:
```bash
colcon build --packages-select chatty
source install/setup.bash
```

### Step 5: Test Your Model
```bash
ros2 launch chatty chat_system.launch.py model:=10
```

---

## 📊 Available Models

| Model ID | Provider | Model Name          | Description                    |
|----------|----------|---------------------|--------------------------------|
| 1        | OpenAI   | gpt-4               | Most capable OpenAI model     |
| 2        | OpenAI   | gpt-4.1-nano        | Fast and efficient             |
| 3        | OpenAI   | gpt-3.5-turbo       | Good balance of speed/quality  |
| 4        | OpenAI   | gpt-4o              | Optimized GPT-4 variant        |
| 5        | OpenAI   | gpt-4.1-mini        | Lightweight version            |
| 6        | Gemini   | gemini-pro          | Google's flagship model        |
| 7        | XAI      | grok-1              | Elon Musk's AI model           |
| 100      | Ollama   | llama2              | Local open-source model        |
| 101      | Ollama   | mistral             | Local Mistral model            |

*Add your custom models*

---

## 🎮 How to Use the System

### 1. Start the System
```bash
ros2 launch chatty chat_system.launch.py model:=1 config_file:=robot_config_my_robots
```

### 2. Chat Interface Opens
A GUI window will appear showing your chat history.

### 3. Start Commanding Your Robots
Type natural language commands like:
- "Robot1, pick up the blue box and place it on the conveyor"
- "Send the drone to inspect the warehouse roof"
- "Have robot2 transport materials from station A to station B"

### 4. View Generated Tasks
The system converts your commands into structured JSON tasks that your robots can execute.

---

## 🔧 System Architecture

### Nodes Overview

#### `chat_gui.py`
- Provides GUI interface using `customtkinter` for human users
- Publishes user messages to `/chat/input` topic
- Displays robot responses from `/chat/output` topic
- Automatically fetches chat history via `get_chat_history` service on startup

#### `chat_manager.py`
- Subscribes to `/chat/input` and publishes to `/chat/output`
- Maintains and publishes complete chat history
- Handles `get_chat_history` service requests
- Provides persistent storage by writing to `data/chat_history.txt`

#### `task_manager.py`
- Subscribes to `/chat/output` topic
- Converts natural language messages to structured task JSON using LLMs
- Supports multiple LLM providers (OpenAI, Ollama, Gemini, XAI)
- **Parameters:**
  - `model`: Integer ID specifying which LLM to use
  - `config_file`: Name of the robot configuration JSON file (without `.json` extension)

---

## 📁 Data Management

### Chat History
All chat messages are automatically timestamped and stored in:
```
data/chat_history.txt
```

This file helps restore conversation context when re-launching the system. The GUI automatically loads this history on startup, ensuring seamless conversation continuity.

---

## 🐛 Troubleshooting

### Common Issues

**1. "No module named 'customtkinter'"**
```bash
pip install customtkinter
```

**2. "API key not found"**
```bash
# Check if your API key is set
echo $OPENAI_API_KEY
# If empty, follow the API key setup section above
```

**3. "Config file not found"**
```bash
# List available config files
ls ~/ros2_ws/src/chatty/config/
# Use exact filename without .json extension
```

**4. GUI doesn't open**
```bash
# Check if you have display access (for remote systems)
echo $DISPLAY
# Install tkinter if missing
sudo apt-get install python3-tk
```

**5. "Package 'chatty' not found"**
```bash
# Rebuild and source
cd ~/ros2_ws
colcon build --packages-select chatty
source install/setup.bash
```

---

## 🎯 Next Steps

After getting the system running:

1. **Experiment with Commands**: Try different natural language inputs
2. **Create Multiple Robot Configs**: Design configs for different scenarios
3. **Integrate Your Robots**: Connect real robots to execute the generated tasks
4. **Customize the GUI**: Modify the interface for your specific needs
5. **Add More LLMs**: Integrate additional AI models for better performance

---

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Make your changes and test them
4. Submit a pull request with a clear description

---

## 📄 License

[Add your license information here]

## 📞 Support

- **Issues**: Report bugs or request features on GitHub
- **Discussions**: Join community discussions for help and ideas
- **Documentation**: Check the wiki for detailed guides

