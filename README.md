# DS-SLAM

This code is modify from the classical SLAM system [ORB-SLAM2](https://github.com/RuPingCen/ORB_SLAM2), and the code is test on the ubuntu1604 with ROS Kinectic. The processor of platform is I7-4710MQ 16G RAM. The Chinese tutorial can be found [in my blog](https://blog.csdn.net/crp997576280/article/details/104297673).

![image](https://github.com/RuPingCen/DS-SLAM/raw/master/fig/DS-SLAM-KITTI-05.gif)
![image](https://github.com/RuPingCen/DS-SLAM/raw/master/fig/RMSE_and_trackingtime.png)

# Useage

## 1 Prerequisites

Please follow the ORB-SLAM2 dependency library command to install the dependency library, such as the OPENCV 3.2, Eigen3, ROS Kinetic

## 2 Clone the code
```
cd yourworkspace
git clone https://github.com/RuPingCen/DS-SLAM.git
cd Vocabulary
tar -zxvf ORBvoc.txt.tar.gz 
cd ..
./build.sh
```
## 3 Run Without ROS
## 3.1 RGBD Mode
The TUMX.yaml need to set as TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change PATH_TO_SEQUENCE_FOLDERto the uncompressed sequence folder.
```
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER
```
## 3.2 STEREO Mode
Change KITTIX.yamlby KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change PATH_TO_DATASET_FOLDER to the uncompressed dataset folder. Change SEQUENCE_NUMBER to 00, 01, 02,.., 11.
```
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## 4 Run With ROS

## 4.1 RGBD Mode

This code can be run on the TUM RGBD dataset and ASTRA RGBD camera.

### 4.1.1  RGBD Mode In TUM Dataset

```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/crp/crp/SLAM/ORB_SLAM2/Examples/ROS
rosrun ORB_SLAM2 astra Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM2/TUM1_ROSbag.yaml
```

Download the [TUM RGBD dataset]( https://vision.in.tum.de/data/datasets/rgbd-dataset), If you are in China, you can download it from [Baidu Cloud](https://pan.baidu.com/s/1W8tBo_QHpAHNyer10dW0Zg) (CODE：di9m).

```
rosbag play rgbd_dataset_freiburg1_room.bag /camera/rgb/image_color:=/camera/rgb/image_raw /camera/depth/image:=/camera/depth/image
```

launch the  pointcloud_mapping node , if you want to get a dense point cloude map.
```
roslaunch pointcloud_mapping tum1.launch
```
### 4.1.2  RGBD Mode In Astra camera

launch the Astra camera node
```
roslaunch astra_launch astra.launch
```
```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/crp/crp/SLAM/ORB_SLAM2/Examples/ROS
rosrun ORB_SLAM2 astra Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM2/Astra.yaml
```
launch the  pointcloud_mapping node , if you want to get a dense point cloude map.
```
roslaunch pointcloud_mapping astra.launch
```

## 4.2 STEREO Mode
## 4.2.1 STEREO Mode in KITTI 
```
roslaunch publish_image_datasets publish_kitti.launch 
```
```
rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/KITTI00-02.yaml false
```
## 4.2.2 STEREO Mode in indemind camera
launch the camera node
```
roslaunch indemind_stereo stereo.launch
```
```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/crp/crp/SLAM/ORB_SLAM2/Examples/ROS
rosrun ORB_SLAM2 indemind Vocabulary/ORBvoc.txt Examples/Stereo/INDEMIND.yaml true
 ```
## 4.2.3 mapping node with stereo
```
roslaunch elas_ros kitti_no_rviz.launch
roslaunch pointcloud_mapping kitti.launch
```
