# ORB_SLAM3_ROS2
This repository clones both https://github.com/UZ-SLAMLab/ORB_SLAM3.git and https://github.com/zang09/ORB_SLAM3_ROS2.git to run ORB_SLAM3 on ROS2. Besides, based on the latter repository, this one adds up rgbd-inertial mode.

## Prerequisites
- Environment and Dependencies
  - Ubuntu 22.04
  - ROS2 humble
  - OpenCV 4.2.0
  - [Pangolin](https://github.com/stevenlovegrove/Pangolin) 
  - [Eigen3](http://eigen.tuxfamily.org) Required at least 3.1.0.

- Build ORB_SLAM3
  - Go to folder ORB_SLAM3.
```
$ git clone https://github.com/Yu-MeiLin/orbslam_dev.git
$ cd ORB_SLAM3
$ chmod +x build.sh
$ ./build.sh
```  

- Install related ROS2 package
```
$ sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters
```

## Build ORB_SLAM3 ROS2 Version
1. Change this [line](https://github.com/Yu-MeiLin/orbslam_dev/blob/4e87b03643b046f67a74c8fc29889ac92566058d/slam/src/orbslam3_ros2/CMakeLists.txt#L7) to your own `python site-packages` path

2. Change this [line](https://github.com/Yu-MeiLin/orbslam_dev/blob/4e87b03643b046f67a74c8fc29889ac92566058d/slam/src/orbslam3_ros2/CMakeModules/FindORB_SLAM3.cmake#L8) to your own `ORB_SLAM3` path

Now, you are ready to build!
```
$ cd slam
$ colcon build --symlink-install --packages-select orbslam3
```

## Troubleshooting
1. If you cannot find `sophus/se3.hpp`:  
Go to your `ORB_SLAM3_ROOT_DIR` and install sophus library.
```
$ cd ~/{ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus/build
$ sudo make install
```

2. Please compile with `OpenCV 4.2.0` version.
Refer this [#issue](https://github.com/zang09/ORB_SLAM3_ROS2/issues/2#issuecomment-1251850857)

## How to use
1. Source the workspace  
```
$ source slam/install/local_setup.bash
```

2. Run orbslam with different modes.  
This repository supports `MONO, STEREO, RGBD, STEREO-INERTIAL, RGBD-INERTIAL` mode now.  
You can find vocabulary file and config file in here. (e.g. `orbslam3_ros2/vocabulary/ORBvoc.txt`, `orbslam3_ros2/config/monocular/TUM1.yaml` for monocular SLAM).
  - `MONO` mode  
```
$ ros2 run orbslam3 mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
  - `STEREO` mode  
```
$ ros2 run orbslam3 stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
```
  - `RGBD` mode  
```
$ ros2 run orbslam3 rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
  - `STEREO-INERTIAL` mode  
```
$ ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
```
  - `RGBD-INERTIAL` mode  
```
$ ros2 run orbslam3 rgbd-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
```  
## Acknowledgments
This repository is modified from [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3.git) and [ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2.git) repository.  
Add `rgbd-inertial` mode and improve build difficulites.
