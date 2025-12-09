
#  ROS2 Camera Click Teleoperation Interface

This project implements a camera-based teleoperation interface using ROS2.  
A live webcam feed is displayed, and the user clicks on the image to control robot movement:

- **Click above the center line → move forward**
- **Click below the center line → move backward**

Compatible with:
- ✔️ Webcam (`v4l2_camera`)
- ✔️ TurtleBot / Turtlesim
- ✔️ A single launch file that starts everything

---

##  Repository Structure

```

ros2_camera_robot_interface/
└── ros2_ws/
├── src/
│   └── camera_click_teleop/
│       ├── camera_click_teleop/
│       │   ├── click_teleop_node.py
│       │   └── **init**.py
│       ├── launch/
│       │   └── click_teleop_with_v4l2.launch.py
│       ├── resource/
│       │   └── camera_click_teleop
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
└── install/ (auto-created by colcon)

````

---

##  Features

- Live webcam feed using OpenCV  
- Mouse click → robot movement mapping  
- Publishes to **/cmd_vel** (`geometry_msgs/Twist`)  
- Green horizontal reference line on the video  
- “Pulse and stop” safety behavior  
- One launch file starts:
  - Webcam  
  - Teleop node  
  - Turtlesim or real robot  

---

##  Dependencies (Ubuntu 22.04 + ROS2 Humble)

```bash
sudo apt update
sudo apt install ros-humble-desktop \
                 ros-humble-v4l2-camera \
                 ros-humble-image-tools \
                 ros-humble-turtlesim \
                 ros-humble-cv-bridge \
                 python3-opencv
````

---

##  Build Instructions

```bash
cd ~/ros2_camera_robot_interface/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select camera_click_teleop
source install/setup.bash
```

---

##  Demo Launch

Run everything with one command:

```bash
ros2 launch camera_click_teleop click_teleop_with_v4l2.launch.py
```

This launches:

* `v4l2_camera_node` (webcam)
* `turtlesim_node`
* `click_teleop_node`

---

##  Interface Behavior

* The webcam feed is displayed in an OpenCV window
* A **green horizontal line** marks the center
* Behavior:

  * **Click above → forward**
  * **Click below → backward**

Published topic:

```
/cmd_vel   (geometry_msgs/Twist)
```

---

##  Debugging / Testing

Check that velocity commands are being published:

```bash
ros2 topic echo /cmd_vel
```

Values should change when you click on the window.

---

## ✔️ Current Project Status

| Task                        | Status |
| --------------------------- | ------ |
| Camera → OpenCV window      | ✔️     |
| Mouse click detection       | ✔️     |
| Publishes /cmd_vel          | ✔️     |
| Full launch automation      | ✔️     |
| TurtleBot/Turtlesim control | ✔️     |
| Clean repository structure  | ✔️     |

---

##  Next Steps (Optional Extensions)

* ArUco marker control
* UR5 robot integration
* Docker deployment

---

##  Contributors

* **Mohammed Shehsin**
* **Laura de León Torres**



