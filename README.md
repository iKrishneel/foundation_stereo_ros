# Foundation Stereo ROS 2

This repository provides an integration of[Foundation Stereo](https://nvlabs.github.io/FoundationStereo/) into ROS2.
Foundation Stereo is a state-of-the-art stereo matching model that enables robust depth estimation from stereo image pairs.
To run the foundation stereo, you will first need to download the trained weights provided by the authors of the papers. 
The weights are available on their [gdrive](https://drive.google.com/file/d/1Yh_2o9QCUrVqZrnAXZ7RUr0zTp3JrMKe/view?usp=drive_link)

## Prerequisites

Before using this package, ensure that your system meets the following requirements:

- **Operating System:** Ubuntu 22.04 (Jammy Jellyfish)
- **ROS 2 Distribution:** Humble Hawksbill
- **CUDA:** Ensure that CUDA is installed and configured on your system to leverage GPU acceleration.

### Installation
1. Clone the Repository
Begin by cloning this repository into the src directory of your ROS 2 workspace:​

```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/iKrishneel/foundation_stereo_ros.git --recursive
```
2. Install Dependencies
Navigate to the root of your workspace and install the necessary dependencies using rosdep:​

```bash
$ cd ~/ros2_ws
$ rosdep install --from-paths src --ignore-src -r -y
```

3. Build the Package
Build the package using colcon:​
```bash
$ colcon build --symlink-install --packages-up-to foundation_stereo_ros
```

4. Download Pretrained Weights
The Foundation Stereo model requires pretrained weights for operation. Download the weights from the official Google Drive link. After downloading, place the weights file in a known directory, for example:​
```bash
/path/to/weights/model_best_bp2.pth
```

## Usage
Running the Node
To launch the Foundation Stereo node, source your workspace and use the provided launch file:​

```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch foundation_stereo_ros foundation_stereo.launch.py weights:=/path/to/your/model_best_bp2.pth left:=LEFT_IMAGE right:=RIGHT_IMAGE info:=CAMERA_INFO
```

*Arguments*
- `weights`: Path to the pretrained weights file.​
- `left`: Topic name for the left stereo image.​
- `right`: Topic name for the right stereo image.​
- `info`: Topic name for the left camera's info.​

Ensure that the topics specified for left, right, and info match the topics published by your stereo camera setup.

## Parameters
The node accepts the following parameters:​

- `config (string)` &rarr; Path to the network configuration file. The default configuration file can be found [here](https://github.com/iKrishneel/foundation_stereo_ros/blob/master/launch/configs/foundation_stereo.yaml).​
- `scale (float)` &rarr; Scale factor for resizing the input images. Valid range is [0, 1], with a default value of `1.0`.​
- `baseline (float)` &rarr; Baseline distance between the stereo cameras.​
- `weights (string)` &rarr; Path to the pretrained weights file.​
- `pub_pcd (boolean)` &rarr; Flag indicating whether to publish the point cloud. Default is `False`.​

To set these parameters, you can include them in your launch file or set them using the ROS 2 parameter interface.
