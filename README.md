# car-end_BEV

![Ubuntu 20.04](https://img.shields.io/badge/OS-Ubuntu_20.04-informational?style=flat&logo=ubuntu&logoColor=white&color=2bbc8a)
![ROS Noetic](https://img.shields.io/badge/Tools-ROS_Noetic-informational?style=flat&logo=ROS&logoColor=white&color=2bbc8a)
![C++](https://img.shields.io/badge/Code-C++-informational?style=flat&logo=c%2B%2B&logoColor=white&color=2bbc8a)
![Python](https://img.shields.io/badge/Code-Python-informational?style=flat&logo=Python&logoColor=white&color=2bbc8a)

This is a project to build car-end multi-camera BEV (bird’s eye view).

## Prerequisites

### 1. **Ubuntu** and **ROS**
* Ubuntu >= 20.04
* ROS    >= Neotic. [ROS Installation](http://wiki.ros.org/noetic/Installation)

### 2. **jsoncpp**
Execute command to install jsoncpp
```
sudo apt-get install libjsoncpp-dev
```

### 3. **nuScenes Dataset**
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

### 1. Visualize nuScenes v1.0-mini dataset in rviz:
```
roslaunch bev_lidar_cali nuscenes_rviz.launch 
```
<img src="/images/nuscene_rivz.png" width="500" alt="nuscene_rviz"/>

### 2. try bev demo:
```
roslaunch bev_lidar_cali nuscenes_toBEV.launch
```
You can select mode by changing `flag` value in `nuscenes_toBEV.launch` file
  - flag = 1: Bird of View demo  
    <img src="/images/bev demo.png" width="500" alt="bev_demo"/>
  - flag = 2: ORB detection demo  
    <img src="/images/orb_detection_demo.png" width="500" alt="orb_detection_demo"/>
  - flag = 3: Join images demo  
    <img src="/images/join_image_demo.png" width="500" alt="join_iamge_demo"/>

### 3. test get camera parameters:
```
rosrun bev_lidar_cali test_read_json
```
You can select camera by editing input of `GetPoints` function in `test_read_json.cpp` in `test` folder
  - Camera front: `CAM_FRONT`
  - Camera front right: `CAM_FRONT_RIGHT`
  - Camera front left: `CAM_FRONT_LEFT`
  - Camera back: `CAM_BACK`
  - Camera back right: `CAM_BACK_RIGHT`
  - Camera back left: `CAM_BACK_LEFT`

<img src="/images/test_read_json.png" width="800" alt="test_read_json"/>

### 3. test do bev:
```
roslaunch bev_lidar_cali nuscenes_toBEV.launch
```
A video available at [Google Drive](https://drive.google.com/file/d/1ctHiO9Mpl_HCP0iu9xUlXhaTUAE6kotB/view?usp=drive_link)

<img src="/images/result_image.png" width="500" alt="result_image"/>
