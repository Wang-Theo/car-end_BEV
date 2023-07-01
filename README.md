# BEV_lidar_cali

![Ubuntu 20.04](https://img.shields.io/badge/OS-Ubuntu_20.04-informational?style=flat&logo=ubuntu&logoColor=white&color=2bbc8a)
![ROS Noetic](https://img.shields.io/badge/Tools-ROS_Noetic-informational?style=flat&logo=ROS&logoColor=white&color=2bbc8a)
![C++](https://img.shields.io/badge/Code-C++-informational?style=flat&logo=c%2B%2B&logoColor=white&color=2bbc8a)
![Python](https://img.shields.io/badge/Code-Python-informational?style=flat&logo=Python&logoColor=white&color=2bbc8a)

This is a project to calibration car-end multi-camera system with road-end lidar.

> Two implementation parts involved:
> - Transform images from car-end multi-cameras to a image in bird’s eye view (BEV)
> - Calibrate BEV image with road-end lidar

## Prerequisites

### 1. **Ubuntu** and **ROS**
* Ubuntu >= 20.04
* ROS    >= Neotic. [ROS Installation](http://wiki.ros.org/noetic/Installation)

### 2. **nuScenes Dataset**
* Full dataset (v1.0) - mini

<img src="/images/sensor_setup.png" width="500" alt="Sensor_set"/>

Download dataset
```
mkdir -p /data/sets/nuscenes
wget https://www.nuscenes.org/data/v1.0-mini.tgz
sudo tar -xf v1.0-mini.tgz -C /data/sets/nuscenes
```
Download the devkit to home directory
```
cd && git clone https://github.com/nutonomy/nuscenes-devkit.git
```

Setup a Conda environment
```
conda create --name nuscenes python=3.7
```

Setup environment variable
- run `sudo gedit ~/.bashrc`
- add path `export PYTHONPATH="${PYTHONPATH}:$HOME/nuscenes-devkit/python-sdk"` and  `export NUSCENES="/data/sets/nuscenes"`

## Build
- Python `.ipynb` test file

Change ipynb environment to conda environment `nuscenes` and run `.ipynb` test file

- C++

The tool is a normal ROS package. Place it under a workspace and build it with catkin.
```
catkin_make -DCATKIN_WHITELIST_PACKAGES=bev_lidar_cali
source devel/setup.bash
```

## Run

We convert nuScenes dataset to a roabag referring repo [nuScenes2Bag](https://github.com/clynamen/nuscenes2bag) developed by [clynamen](https://github.com/clynamen/) and [
ChernoA](https://github.com/ChernoA)

Visualize nuScenes v1.0-mini dataset in rviz:
```
roslaunch bev_lidar_cali nuscenes_rviz.launch 
```
<img src="/images/nuscene_rviz.png" width="500" alt="nuscene_rviz"/>
