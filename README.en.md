<a name="readme-top"></a>


[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# Real-time 2D Multi-Person Pose Estimation on CPU: Lightweight OpenPose (ROS support) 

<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#summary">Summary</a></li>
    <li>
      <a href="#setup">Setup</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li>
      <a href="#training">Training</a>
      <ul>
        <li><a href="#preparation">Preparation</a></li>
        <li><a href="#training-by-yourself">Training by yourself</a></li>
        <li><a href="#validation">Validation</a></li>
        <li><a href="#pre-trained-model">Pre-trained model</a></li>
        <li><a href="#python-Demo">Python Demo</a></li>
      </ul>
    </li>
    <li>
      <a href="#launch-and-usage">Launch and Usage</a>
      <ul>
        <li><a href="#camera">Camera</a></li>
        <li><a href="#2d-skeleton-detection">2D skeleton detection </a></li>
        <li><a href="#3d-skeleton-detection">3D skeleton detection</a></li>
        <li><a href="#2d-subscriptions">2D Subscriptions</a></li>
        <li><a href="#2d-publications">2D Publications</a></li>
        <li><a href="#3d-subscriptions">3D Subscriptions</a></li>
        <li><a href="#3d-publications">3D Publications</a></li>
        <li><a href="#parameters">Parameters</a></li>
      </ul>
    </li>
    <li><a href="#milestone">Milestone</a></li>
    <li><a href="#finally">Finally</a></li>
  </ol>
</details>


## Summary
The repository where the training code for the paper [Real-time 2D Multi-Person Pose Estimation on CPU: Lightweight OpenPose](https://arxiv.org/pdf/1811.12004.pdf) is implemented is now available as a fork repository for ROS1. The repository is a forked repository of [OpenPose](). This is a greatly optimized version of the [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) method that allows real-time inference on CPUs without loss of accuracy. To identify the pose of a person in an image, a skeleton (consisting of key points and connections between them) is detected. Up to 18 skeletons can be estimated in real time: "ears, eyes, nose, neck, shoulders, elbows, wrists, hips, knees, and ankles." In addition, using the point cloud information from the RGB-D sensor, the system can obtain not only 2D skeletal coordinates but also 3D skeletal coordinates.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<p align="center">
  <img src="img/preview_2.png" height="320"/>
</p>

## Setup


### Prerequisites

* Ubuntu 20.04
* Python 3.8
* PyTorch 1.13.1
* OpenCV 4.6.0
* ROS Noetic Ninjemys

### Installation

```bash
$ cd ~/catkin_ws/src/
$ git clone https://gitlab.com/TeamSOBITS/lightweight_human_pose_estimation_pytorch.git
$ cd lightweight_human_pose_estimation_pytorch
$ bash install.sh
$ cd ~/catkin_ws/
$ catkin_make 
```

## Training

<details>
<summary>details</summary>

### Preparation

1. Download COCO 2017 dataset: [http://cocodataset.org/#download](http://cocodataset.org/#download) (train, val, annotations) and unpack it to `<COCO_HOME>` folder.
2. Install requirements

```bash
$ python3 -m pip install -r requirements.txt
```

### Training by yourself

Training consists of 3 steps (given AP values for full validation dataset):
* Training from MobileNet weights. Expected AP after this step is ~38%.
* Training from weights, obtained from previous step. Expected AP after this step is ~39%.
* Training from weights, obtained from previous step and increased number of refinement stages to 3 in network. Expected AP after this step is ~40% (for the network with 1 refinement stage, two next are discarded).

1. Download pre-trained MobileNet v1 weights `mobilenet_sgd_68.848.pth.tar` from: [https://github.com/marvis/pytorch-mobilenet](https://github.com/marvis/pytorch-mobilenet) (sgd option). If this doesn't work, download from [GoogleDrive](https://drive.google.com/file/d/18Ya27IAhILvBHqV_tDp0QjDFvsNNy-hv/view?usp=sharing).

2. Convert train annotations in internal format. Run:

```bash
$ python3 scripts/prepare_train_labels.py --labels <COCO_HOME>/annotations/person_keypoints_train2017.json
```

It will produce `prepared_train_annotation.pkl` with converted in internal format annotations.

   [OPTIONAL] For fast validation it is recommended to make *subset* of validation dataset. Run:

```bash
$ python3 scripts/make_val_subset.py --labels <COCO_HOME>/annotations/person_keypoints_val2017.json
```

It will produce `val_subset.json` with annotations just for 250 random images (out of 5000).

3. To train from MobileNet weights, run:

```bash
$ python3 train.py --train-images-folder <COCO_HOME>/train2017/ --prepared-train-labels prepared_train_annotation.pkl --val-labels val_subset.json --val-images-folder <COCO_HOME>/val2017/ --checkpoint-path <path_to>/mobilenet_sgd_68.848.pth.tar --from-mobilenet
```

4. Next, to train from checkpoint from previous step, run:

```bash
$ python3 train.py --train-images-folder <COCO_HOME>/train2017/ --prepared-train-labels prepared_train_annotation.pkl --val-labels val_subset.json --val-images-folder <COCO_HOME>/val2017/ --checkpoint-path <path_to>/checkpoint_iter_420000.pth --weights-only
```

5. Finally, to train from checkpoint from previous step and 3 refinement stages in network, run:

```bash
$ python3 train.py --train-images-folder <COCO_HOME>/train2017/ --prepared-train-labels prepared_train_annotation.pkl --val-labels val_subset.json --val-images-folder <COCO_HOME>/val2017/ --checkpoint-path <path_to>/checkpoint_iter_280000.pth --weights-only --num-refinement-stages 3
```

We took checkpoint after 370000 iterations as the final one.

We did not perform the best checkpoint selection at any step, so similar result may be achieved after less number of iterations.


### Validation

1. Run:

```bash
$ python3 val.py --labels <COCO_HOME>/annotations/person_keypoints_val2017.json --images-folder <COCO_HOME>/val2017 --checkpoint-path <CHECKPOINT>
```

### Pre-trained model

The model expects normalized image (mean=[128, 128, 128], scale=[1/256, 1/256, 1/256]) in planar BGR format.
Pre-trained on COCO model is available at: https://download.01.org/opencv/openvino_training_extensions/models/human_pose_estimation/checkpoint_iter_370000.pth, it has 40% of AP on COCO validation set (38.6% of AP on the val *subset*).

### Python Demo

We provide python demo just for the quick results preview. Please, consider c++ demo for the best performance. To run the python demo from a webcam:

```bash
$ cd script
$ python3 demo.py --checkpoint-path checkpoints/checkpoint_iter_370000.pth --video 0
```
</details>


## Launch and Usage


### Camera
When using a USB camera (PC built-in camera)
```bash
roslaunch lightweight_human_pose_estimation camera.launch
```
<details>
<summary>※Solution for USB camera error</summary>

If the following error occurs:
```bash
[ERROR] [1663911409.917317256]: Permission denied opening /dev/bus/usb/001/002
```

Run the following code:
```bash
$ sudo chmod o+w /dev/bus/usb/001/002
```
</details>


[When using Azure Kinect](https://github.com/TeamSOBITS/azure_kinect_ros_driver) 
[When using RealSense](https://github.com/TeamSOBITS/realsense_ros) 

### 2D skeleton detection 
```bash
roslaunch lightweight_human_pose_estimation human_pose_estimation.launch
```
### 3D skeleton detection 
```bash
$ roslaunch lightweight_human_pose_estimation human_pose_estimation_3d.launch
```

### 2D Subscriptions
|Topic Name|Type|Meaning|
|---|---|---|
|/camera/rgb/image_raw|sensor_msgs/Image|Sensor image|

### 2D Publications
|トピック名|型|意味|
|---|---|---|
|/human_pose_estimation/pose|lightweight_human_pose_estimation/KeyPoints|2D skeletal information|
|/human_pose_estimation/pose|sensor_msgs/Image|2D skeletal image|

### 3D Subscriptions
|トピック名|型|意味|
|---|---|---|
|/camera/depth/points|sensor_msgs/PointCloud2|Sensor point cloud|
|/human_pose_estimation/pose|lightweight_human_pose_estimation/KeyPoints|2D skeletal information|


### 3D Publications
|トピック名|型|意味|
|---|---|---|
|/lightweight_human_pose_estimation/human_pose_estimation/pose_3d|lightweight_human_pose_estimation/KeyPoints_3d|3D skeletal information|

### Parameters
|パラメータ名|型|意味|
|---|---|---|
|/human_pose_estimation/lightweight_human_pose_estimation/checkpoint_path|string|Path of the model weight file|
|/human_pose_estimation/lightweight_human_pose_estimation/cpu|bool|Skeleton detection by CPU only (when using CUDA : False)|
|/human_pose_estimation/lightweight_human_pose_estimation/pose_image_topic_name|string|Topic name of sensor image|
|/human_pose_estimation/lightweight_human_pose_estimation/pose_img_show_flag|bool|Flag whether to display images|
|/human_pose_estimation/lightweight_human_pose_estimation/pose_pub_result_image|bool|Flag whether to publish skeletal detection images|
|/human_pose_estimation/lightweight_human_pose_estimation/smooth|bool|Flag whether to smooth the framework with the previous frame|
|/human_pose_estimation/lightweight_human_pose_estimation/track|bool|Flag whether to propagate the result of the previous frame|


## Milestone

- [x] OSS
    - [x]  Improved documentation
    - [x]  Unified coding style

See the open issues for a full list of proposed features (and known issues).
<!-- ## 参考文献
* [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
* [ROS Noetic](http://wiki.ros.org/noetic)
* [ROS Control](http://wiki.ros.org/ros_control) -->

### Finally
If this helps your research, please cite the paper:

```
@inproceedings{osokin2018lightweight_openpose,
    author={Osokin, Daniil},
    title={Real-time 2D Multi-Person Pose Estimation on CPU: Lightweight OpenPose},
    booktitle = {arXiv preprint arXiv:1811.12004},
    year = {2018}
}
```
<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/lightweight_human_pose_estimation_pytorch.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/lightweight_human_pose_estimation_pytorch/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/lightweight_human_pose_estimation_pytorch.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/lightweight_human_pose_estimation_pytorch/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/lightweight_human_pose_estimation_pytorch.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/lightweight_human_pose_estimation_pytorch/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/lightweight_human_pose_estimation_pytorch.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/lightweight_human_pose_estimation_pytorch/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/lightweight_human_pose_estimation_pytorch.svg?style=for-the-badge
[license-url]: LICENSE