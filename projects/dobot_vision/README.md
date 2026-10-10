**Dobot ROS2 视觉**

本SDK只适用于EG0053-01工作台其它工作站不可用！！

作者：dobot_futingxing

最后更新时间：25-06-10

---

### **运行环境配置**

#### **操作系统**

推荐使用 **Ubuntu 22.04** 版本的 Linux 系统。

---

### **安装 ROS2**

#### **1. 设置编码**

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

#### **2. 添加源**

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### **3. 安装 ROS2**

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

#### **4. 设置环境变量**

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### **5. 小海龟仿真示例**

启动两个终端，分别运行以下指令：

```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

- 第一句指令将启动一个蓝色背景的海龟仿真器。
- 第二句指令将启动键盘控制节点，使用键盘上的“上下左右”按键控制小海龟运动。

![turtlesim](/image/image2544.png)

---

### **Gazebo 安装**

#### **安装**

```bash
sudo apt install ros-humble-gazebo-*
```

#### **环境变量添加**

```bash
echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
```

#### **运行**

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

---

### **MoveIt 安装**

#### **安装**

```bash
sudo apt-get install ros-humble-moveit
```

---

### **安装 Dobot ROS2**

1. 前往 [DOBOT_6Axis_ROS2_V4](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4/tree/main) 下载并安装软件包。
2. 设置机型为 CR3：
   ```bash
   echo "export DOBOT_TYPE=cr3" >> ~/.bashrc
   source ~/.bashrc
   ```

---

### **安装 CUDA**

1. 确保已安装最新的显卡驱动。
   ![CUDA](/image/image55985.png)
2. 下载并安装 CUDA 11.8：

   ```bash
   wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run
   sudo sh cuda_11.8.0_520.61.05_linux.run
   ```

   - 安装时取消勾选安装显卡驱动的选项。
3. 添加环境变量：

   ```bash
   sudo gedit ~/.bashrc
   ```

   在文件中添加以下内容：
   ```bash
   export PATH=/usr/local/cuda-11.8/bin:$PATH
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-11.8/lib64
   ```
4. 重启操作系统。
5. 验证安装：

   ```bash
   nvcc -V
   ```
6. 安装 cuDNN，参考 [cuDNN 官网](https://developer.nvidia.com/cudnn-downloads)。
   ![cuDNN](/image/image45234.png)

---

### **安装英特尔深度相机驱动**

参考 [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) 安装指南。
![librealsense](/image/image1254.png)

---

### **安装 Dobot ROS2 Vision**

1. 创建并进入工作空间：
   ```bash
   mkdir -p ~/dobot_ros2_vision_ws/src
   cd ~/dobot_ros2_vision_ws
   ```
2. 将文件放入 `src` 目录下。
3. 编译：
   ```bash
   colcon build
   ```
4. 设置环境变量：
   ```bash
   echo "source ~/dobot_ros2_vision_ws/install/local_setup.sh" >> ~/.bashrc
   source ~/.bashrc
   ```

---

### **安装必要库**

```bash
pip install pyrealsense2 ultralytics opencv-python pyyaml
```

---

### **配置主机 IP**

将主机用于连接机械臂的网卡 IP 修改为 `192.168.5.xxx` 网段（子网掩码 `255.255.255.0`），并打开对应有线网络的开关。
![IP](/image/image574632.png)
--

### **使用说明**

#### **相机信息发布**

运行：

```bash
ros2 run realsense_manipulator_bringup realsense_camera_bringup
```

- 发布的信息包括 RGB 图像、深度图像、完整点云、滤波点云、相机内参及畸变系数。
- /realsense_cloud_intact  #完整PointCloud包含完整（x,y,z）
- /realsense_cloud  #过滤无效深度值PointCloud过滤深度为0值的结果
- /realsense_color_image  #RGB图像
- /realsense_depth_image  #深度图像
- /realsense/internal_parameters #内参及畸变
  ![realsense_manipulator_bringup](/image/image5287.png)

#### **手眼标定**

1. 打印 4x4（50x50mm）标定板。
2. 执行：
   ```bash
   ros2 launch dobot_camera_calibration aruco_dobot.launch.py
   ```
3. 开始标定：
   ```bash
   ros2 run dobot_camera_calibration hand_eye_calibration
   ```

   - 输入 `1` 采集数据，输入 `2` 计算标定结果，输入 `3` 保存结果，输入 `4` 校验标定结果，输入 `5` 清空数据。
   - ![dobot_camera_calibration](/image/image42434.png)

#### **YOLO 模型训练**

1. 下载 [Ultralytics](https://github.com/ultralytics/ultralytics) 源码。
2. 将标注后的数据集放入 `datasets` 文件夹。
3. 修改 `train.py` 文件并运行以开始训练。
   ![Ultralytics1](/image/image5245.png)
   ![Ultralytics2](/image/image5421.png)
4. 训练完成后，将 `best.pt` 文件放入 `vision_resource_7b9c/yolo` 文件夹。
   ![Ultralytics3](/image/image5242.png)

#### **识别抓取**

1. 连接机械臂：

   ```bash
   ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
   ```
2. 运行识别：

   ```bash
   ros2 run object_recognition yolo_v11
   ```
3. 获取位置并抓取：

   ```bash
   ros2 run object_recognition dobot_move
   ```

   - 输入 `2` 查看当前识别到的抓取点，输入 `1` 执行抓取，输入 `3` 回到原点。
     ![object_recognition](/image/image554.png)
     ![D435-XYZ](/image/image5255.png)

---
