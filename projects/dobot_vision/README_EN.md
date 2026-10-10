**Dobot ROS2 Vision**

This SDK is only compatible with the EG0053-01 workstation and is not compatible with other workstations!

Author: dobot_futingxing

Last updated: 25-06-10

---

### **Environment Configuration**

#### **Operating System**

It is recommended to use **Ubuntu 22.04** version of the Linux system.

---

### **Install ROS2**

#### **1. Set Locale**

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

#### **2. Add Source**

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### **3. Install ROS2**

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

#### **4. Set Environment Variables**

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### **5. Turtlesim Demo**

Start two terminals and run the following commands respectively:

```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

- The first command will start a blue-background turtlesim simulator.
- The second command will start a keyboard control node, allowing you to control the turtle's movement using the arrow keys.
  ![turtlesim](/image/image2544.png)

---

### **Gazebo Installation**

#### **Install**

```bash
sudo apt install ros-humble-gazebo-*
```

#### **Add Environment Variables**

```bash
echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
```

#### **Run**

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

---

### **MoveIt Installation**

#### **Install**

```bash
sudo apt-get install ros-humble-moveit
```

---

### **Install Dobot ROS2**

1. Go to [DOBOT_6Axis_ROS2_V4](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4/tree/main) to download and install the package.
2. Set the model to CR3:
   ```bash
   echo "export DOBOT_TYPE=cr3" >> ~/.bashrc
   source ~/.bashrc
   ```

---

### **Install CUDA**

1. Ensure that the latest graphics driver is installed.
   ![CUDA](/image/image55985.png)
2. Download and install CUDA 11.8:

   ```bash
   wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run
   sudo sh cuda_11.8.0_520.61.05_linux.run
   ```

   - Uncheck the option to install the graphics driver during installation.
3. Add environment variables:

   ```bash
   sudo gedit ~/.bashrc
   ```

   Add the following content to the file:
   ```bash
   export PATH=/usr/local/cuda-11.8/bin:$PATH
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-11.8/lib64
   ```
4. Restart the operating system.
5. Verify the installation:

   ```bash
   nvcc -V
   ```
6. Install cuDNN, refer to the [cuDNN official website](https://developer.nvidia.com/cudnn-downloads).
   ![cuDNN](/image/image45234.png)

---

### **Install Intel Depth Camera Driver**

Refer to the [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) installation guide.
![librealsense](/image/image1254.png)

---

### **Install Dobot ROS2 Vision**

1. Create and enter the workspace:
   ```bash
   mkdir -p ~/dobot_ros2_vision_ws/src
   cd ~/dobot_ros2_vision_ws
   ```
2. Place the files in the `src` directory.
3. Build:
   ```bash
   colcon build
   ```
4. Set environment variables:
   ```bash
   echo "source ~/dobot_ros2_vision_ws/install/local_setup.sh" >> ~/.bashrc
   source ~/.bashrc
   ```

---

### **Install Required Libraries**

```bash
pip install pyrealsense2 ultralytics opencv-python pyyaml
```

---

### **Configure Host IP**

Modify the IP of the host's network card used to connect to the robotic arm to the `192.168.5.xxx` subnet (subnet mask `255.255.255.0`), and enable the corresponding wired network.
![IP](/image/image574632_EN.png)

---

### **Usage Instructions**

#### **Camera Information Publishing**

Run:

```bash
ros2 run realsense_manipulator_bringup realsense_camera_bringup
```

- Published information includes RGB images, depth images, complete point cloud, filtered point cloud, camera intrinsics, and distortion coefficients.
- /realsense_cloud_intact  # Complete PointCloud containing (x,y,z)
- /realsense_cloud  # Filtered PointCloud with invalid depth values removed
- /realsense_color_image  # RGB image
- /realsense_depth_image  # Depth image
- /realsense/internal_parameters # Intrinsics and distortion
  ![realsense_manipulator_bringup](/image/image5287.png)

#### **Hand-Eye Calibration**

1. Print a 4x4 (50x50mm) calibration board.
2. Execute:
   ```bash
   ros2 launch dobot_camera_calibration aruco_dobot.launch.py
   ```
3. Start calibration:
   ```bash
   ros2 run dobot_camera_calibration hand_eye_calibration
   ```

   - Enter `1` to collect data, `2` to calculate calibration results, `3` to save results, `4` to verify calibration results, and `5` to clear data.
   - ![dobot_camera_calibration](/image/image42434.png)

#### **YOLO Model Training**

1. Download the [Ultralytics](https://github.com/ultralytics/ultralytics) source code.
2. Place the annotated dataset in the `datasets` folder.
3. Modify the `train.py` file and run it to start training.
   ![Ultralytics1](/image/image5245.png)
   ![Ultralytics2](/image/image5421.png)
4. After training, place the `best.pt` file in the `vision_resource_7b9c/yolo` folder.
   ![Ultralytics3](/image/image5242.png)

#### **Recognition and Grasping**

1. Connect the robotic arm:

   ```bash
   ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
   ```
2. Run recognition:

   ```bash
   ros2 run object_recognition yolo_v11
   ```
3. Obtain position and grasp:

   ```bash
   ros2 run object_recognition dobot_move
   ```

   - Enter `2` to view the current recognized grasping points, `1` to execute grasping, and `3` to return to the origin.
     ![object_recognition](/image/image554.png)
     ![D435-XYZ](/image/image5255.png)

---

**
