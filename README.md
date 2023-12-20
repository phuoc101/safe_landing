This is the main repository for my master's thesis: "Vision-based safe autonomous UAV landing with panoramic sensors".

# ⚙ Dependencies

The repository was developed and tested with:

- Ubuntu 20.04
- ROS Noetic
- Pytorch 1.11+cu113 (But newer versions of Pytorch should work as well)
- [PICAM360](https://www.picam360.com/) panoramic camera module
- [PX4](https://github.com/PX4/PX4-Autopilot) (follow the instructions on the main documentations)

Prepare your workspace:

```bash
mkdir -p safelanding_ws/src
cd safelanding_ws/src
git clone https://github.com/phuoc101/detection_msgs.git
git clone https://github.com/phuoc101/yolov7_ros.git
git clone https://github.com/phuoc101/safe_landing.git
cd ..
catkin build
```

# 🚀 Run the software

```bash
# Source ROS environment
source <path_to_ros-ws>/devel/setup.bash #or .zsh, depending on your shell
```

The following commands run the program in Gazebo simulation. There are 2 options: hover and wait or adaptively select landing zone:

```bash
# Start PX4 simulation
roslaunch px4 roslaunch px4

## !!! CHOOSE ONE
# Adaptive emergency landing
roslaunch safe_landing offboard_live_sim.launch yolo_weights:=<your_weight> video_device:=/dev/video0 visualize_yolo:=true conf_thres:=0.5

# Adaptive emergency landing
roslaunch safe_landing offboard_live_sim_adaptive.launch yolo_weights:=<your_weight> video_device:=/dev/video0 visualize_yolo:=true conf_thres:=0.5

# The yolo image is published under topic /yolov7/image_raw
rosrun rqt_image_view rqt_image_view
```

# 🕹 Demo

[Link to demonstration video](https://www.youtube.com/watch?v=XdolUS1bUVs)
