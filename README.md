# Autonomous Radar-Mapping Robot

An embedded robotics project that scans its surroundings, detects obstacles and hazards, and sends live sensor data to a Python radar visualization.

The system combines **embedded C, sensor integration, robot motion control, TCP communication, and real-time visualization**.

## Demo

- [Watch the robot demo](https://drive.google.com/file/d/17zTY26zI-6VLDzKnHN7GtRfwhL1wV5gx/view?usp=drive_link)
- [View a captured radar scan](https://drive.google.com/file/d/1fBO3oBK7KrYdduAYPxW-klrQkZY_3J-x/view?usp=sharing)

## Core Features

- **180° environment scanning**
- **IR and ultrasonic object detection**
- **Object distance and width estimation**
- **Cliff and bump hazard detection**
- **Autonomous and manual robot movement**
- **TCP communication between the robot and Python client**
- **Live radar visualization using Matplotlib**

## Tech Stack

**Embedded:** C, TM4C123 microcontroller  
**Robot Platform:** iRobot Create  
**Sensors:** IR, PING ultrasonic, cliff sensors, bump sensors  
**Communication:** UART, TCP sockets  
**Visualization:** Python, Matplotlib, NumPy  
**Development:** Code Composer Studio

## How It Works

### 1. Environment Scanning

The robot rotates its sensor assembly across a **180° field of view** and collects sensor readings at multiple angles.

IR and ultrasonic sensors are used to detect nearby objects and estimate their distance.

### 2. Object Detection

Adjacent sensor readings are grouped into detected objects.

The firmware calculates:

- Start and end angle
- Object midpoint
- Distance
- Estimated object width

This information is used to build a simplified map of the robot's surroundings.

### 3. Hazard Detection

Cliff and bump sensors detect obstacles, boundaries, and drop-offs.

The robot can stop movement when unsafe conditions are detected.

### 4. Robot Navigation

The system supports:

- Forward movement
- Reverse movement
- Incremental turning
- 90° turns
- Environment rescanning

Movement commands are processed by the embedded controller.

### 5. Real-Time Visualization

Sensor data is transmitted from the robot to a Python client over TCP.

The Python application parses the scan data and updates a live polar radar display showing:

- Object angle
- Distance
- Estimated size
- Hazard locations

## Controls

| Command | Action |
| --- | --- |
| `W` | Move forward |
| `S` | Move backward |
| `A` | Turn left |
| `D` | Turn right |
| `1` | Short scan |
| `2` | Full 180° scan |
| `9` | Turn 90° left |
| `0` | Turn 90° right |

The Python client also supports keyboard input while receiving sensor data in a separate thread.

## Running the Visualization

### Requirements

- Python 3
- NumPy
- Matplotlib
- keyboard

Install dependencies:

```bash
pip install numpy matplotlib keyboard
```

Update the robot connection information in the visualization script if needed:

```python
HOST = "192.168.1.1"
PORT = 288
```

Run the visualization:

```bash
python front_scan.py
```

The robot firmware must be running and connected before starting the Python client.

## Repository Structure

```text
Autonomous-Radar-Mapping-Robot/
├── YesInterruptMove.c      # Main movement and command logic
├── scan.c                  # Sensor scanning and object detection
├── adc.c                   # IR sensor ADC support
├── open_interface.c        # iRobot Create interface
├── new_uart_interrupt.c    # UART communication
├── front_scan.py           # Python radar visualization
└── targetConfigs/          # TM4C123 configuration
```

## What This Project Demonstrates

This project demonstrates embedded systems development across:

- Sensor integration
- Real-time data collection
- Robot motion control
- Object detection
- TCP communication
- Multithreaded Python applications
- Real-time data visualization
