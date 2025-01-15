# VINS-with-Error-Model
**A modified version of VINS-Mono integrating a descriptor-based optical flow error model**, developed based on [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono).


This repository integrates a descriptor-based optical flow error model into VINS-Mono. The error model quantitatively assesses the quality of optical flow by utilizing the feature descriptor, then rejects outlier optical flow features and constructs an error model for visual measurements, thereby enhancing visual localization based on the optical flow quality.

For a detailed explanation and experimental results, please refer to our [paper].

[paper]: https://

## 1. Prerequisites

The dependency is equal to that of [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono).

* **Ubuntu**: Ubuntu 64-bit 16.04 or 18.04.  
  
* **ROS**: ROS Kinetic or Melodic. Follow [ROS Installation](http://wiki.ros.org/ROS/Installation).

* **Ceres Solver**: Ceres Solver 1.14.0. Follow [Ceres Installation](http://ceres-solver.org/installation.html).

## 2. building_construction

``` bash
$ cd ~/catkin_ws/src 
$ git clone https:
$ cd ../
$ catkin_make  
```

## 3. Run on the dataset

* **Guidelines for running on the EuRoC and TUM VI datasets.**

``` bash
$ mv ~/catkin_ws/src/VINS-with-Error-Model/run.sh ~/catkin_ws/
$ cd ~/catkin_ws/
$ sh run.sh
```
The default sequence run by the script `run.sh` is MH_01_easy. To run other sequences, please modify the file paths and launch files specified in the `run.sh` script.

* **Guidelines for running on the datasets collected by our developed in-house embedded positioning device.**

  The dataset can be accessed from the following Google Drive link: [https://drive.google.com/drive/folders/1pj_Mf7HZoEAfbB15ckfvofcqjkrKX6zL?usp=drive_link](https://drive.google.com/drive/folders/1pj_Mf7HZoEAfbB15ckfvofcqjkrKX6zL?usp=drive_link).

  Given that the image resolution of the dataset is 960×540, please ensure that the resolution of `fisheye_mask.jpg` in the `config` folder is also 960×540. Fisheye mask files with different resolutions can be found in the `config/fishmask/` folder.

``` bash
$ cd ~/catkin_ws/
$ sh run.sh 
```

## 4. Acknowledgements

Thanks to the open-source [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) project, we can rapidly develop our algorithm within the VINS system.









