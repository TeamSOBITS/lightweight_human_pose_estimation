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


<!-- INTRODUCTION -->
## Introduction

The repository where the training code for the paper [Real-time 2D Multi-Person Pose Estimation on CPU: Lightweight OpenPose](https://arxiv.org/pdf/1811.12004.pdf) is implemented is now available as a fork repository for ROS1.
To identify the pose of a person in an image, a skeleton (consisting of key points and connections between them) is detected.
Up to 18 skeletons can be estimated in real time: "ears, eyes, nose, neck, shoulders, elbows, wrists, hips, knees, and ankles."
In addition, using the point cloud information from the RGB-D sensor, the system can obtain not only 2D skeletal coordinates but also 3D skeletal coordinates.

<details>
<summary>List of body parts</summary>

| ID | Variable | Body Part |
| --- | --- | --- |
| 0  | nose   | nose |
| 1  | neck   | neck |
| 2  | r_sho  | right shoulder |
| 3  | r_elb  | right elbow |
| 4  | r_wri  | right wrist |
| 5  | l_sho  | left shoulder |
| 6  | l_elb  | left elbow |
| 7  | l_wri  | left wrist |
| 8  | r_hip  | right hip |
| 9  | r_knee | right knee |
| 10 | r_ank  | right ankle |
| 11 | l_hip  | left hip |
| 12 | l_knee | left knee |
| 13 | l_ank  | left ankle |
| 14 | r_eye  | right eye |
| 15 | l_eye  | left eye |
| 16 | r_ear  | right ear |
| 17 | l_ear  | left ear |

</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Setup

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Prerequisites

First, please set up the following environment before proceeding to the next installation stage.

| System  | Version |
| --- | --- |
| Ubuntu  | 20.04 (Focal Fossa) |
| ROS     | Noetic Ninjemys |
| Python  | 3.8 |
| OpenCV  | 4.9.0 |
| PyTorch | >=0.4.1 (Tested on: 2.2.1) |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### Installation

1. Go to the `src` folder of ROS.
   ```sh
   $ roscd
   # Or just use "cd ~/catkin_ws/" and change directory.
   $ cd src/
   ```
2. Clone this repository.
   ```sh
   $ git clone https://github.com/TeamSOBITS/lightweight_human_pose_estimation_pytorch
   ```
3. Navigate into the repository.
   ```sh
   $ cd lightweight_human_pose_estimation_pytorch/
   ```
4. Install the dependent packages.
   ```sh
   $ bash install.sh
   ```
5. Compile the package.
   ```sh
   $ roscd
   # Or just use "cd ~/catkin_ws/" and change directory.
   $ catkin_make
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- LAUNCH AND USAGE EXAMPLES -->
## Launch and Usage

### Camera bring-up

When using a USB camera (PC built-in camera), execute the following command.
```bash
roslaunch lightweight_human_pose_estimation camera.launch
```

<details>
<summary>Solution for USB camera error</summary>

If the following error occurs:
```bash
[ERROR] [1663911409.917317256]: Permission denied opening /dev/bus/usb/001/002
```

Run the following code:
```bash
$ sudo chmod o+w /dev/bus/usb/001/002
```
> [!NOTE]
> Beware that the `/dev/bus/usb/001/002` section might change.
Adapt the command based on the log in the terminal.

</details>


> [!NOTE]
> If you plan to use [Azure Kinect](https://github.com/TeamSOBITS/azure_kinect_ros_driver) or [RealSense](https://github.com/TeamSOBITS/realsense_ros), plase do not forget to configure the environment. 

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### Inference Parameters

1. Depending on the functions you need to turn on for the pose detection, please update the [human_pose.launch](launch/human_pose.launch).
    ``` xml
    <!-- Camera RBG Image Raw topic -->
    <arg name ="input_image_topic"      default="/camera/rgb/image_raw"/>

    <!-- Select the camera base frame -->
    <arg name ="base_frame_name"        default="camera_link"/>
    <!-- Select the pose_2d topic name -->
    <arg name ="pose_2d_topic_name"     default="/human_pose_2d/pose_array"/>
    <!-- Select the cloud topic name -->
    <arg name ="cloud_topic_name"       default="/camera/depth/points"/>
    <!-- Select the camera_info topic name -->
    <arg name ="camera_info_topic_name" default="/camera/rgb/camera_info"/>

    <!-- Show 2D Pose result image (true) -->
    <arg name ="pose_2d_img_show"       default="false"/>
    <!-- Publish 2D Pose result image (true) -->
    <arg name ="pose_2d_img_pub"        default="true"/>
    <!-- Show 2D Pose result as log in terminal (true) -->
    <arg name ="pose_2d_log_show"       default="false"/>

    <!-- Enable 3D Pose detection (true) -->
    <arg name ="pose_3d_detect"         default="true"/>
    <!-- Publish 3D Pose result as topic (true) -->
    <arg name ="pose_3d_topic_pub"      default="false"/>
    <!-- Broadcast 3D Pose result as TF (true) -->
    <arg name ="pose_3d_tf_pub"         default="true"/>
    <!-- Show 3D Pose result as log in terminal (true) -->
    <arg name ="pose_3d_log_show"       default="false"/>
    ```

2. Execute the launch file.
    ```bash
    roslaunch lightweight_human_pose_estimation human_pose.launch
    ```

### Subscribers & Publishers

- Subscribers:

| Topic | Type | Meaning |
| --- | --- | --- |
| /camera/rgb/image_raw     | sensor_msgs/Image                                 | Camera Image |
| /camera/depth/points      | sensor_msgs/PointCloud2                           | Camera PointCloud |
| /human_pose_2d/pose_array | lightweight_human_pose_estimation/KeyPoint2DArray | 2D Pose result information |

- Publishers:

| Topic | Type | Meaning |
| --- | --- | --- |
| /human_pose_2d/pose_array | lightweight_human_pose_estimation/KeyPoint2DArray | 2D Pose result information |
| /human_pose_2d/pose_img   | sensor_msgs/Image                                 | 2D Pose result image |
| /human_pose_3d/pose_array | lightweight_human_pose_estimation/KeyPoints_3d    | 3D Pose result information |


### Services

| Service | Type | Meaning |
| --- | --- | --- |
| /human_pose_3d/run_ctr | sobits_msgs/RunCtrl | 3D Pose Detection toogle (ON:`true`, OFF:`false`) |

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
- Training from MobileNet weights. Expected AP after this step is ~38%.
- Training from weights, obtained from previous step.
Expected AP after this step is ~39%.
- Training from weights, obtained from previous step and increased number of refinement stages to 3 in network.
Expected AP after this step is ~40% (for the network with 1 refinement stage, two next are discarded).

1. Download pre-trained MobileNet v1 weights `mobilenet_sgd_68.848.pth.tar` from: [https://github.com/marvis/pytorch-mobilenet](https://github.com/marvis/pytorch-mobilenet) (sgd option).
If this doesn't work, download from [GoogleDrive](https://drive.google.com/file/d/18Ya27IAhILvBHqV_tDp0QjDFvsNNy-hv/view?usp=sharing).

2. Convert train annotations in internal format.
It will produce `prepared_train_annotation.pkl` with converted in internal format annotations.
Run:
```bash
$ python3 scripts/prepare_train_labels.py --labels <COCO_HOME>/annotations/person_keypoints_train2017.json
```

[OPTIONAL] For fast validation it is recommended to make *subset* of validation dataset.
It will produce `val_subset.json` with annotations just for 250 random images (out of 5000).
Run:
```bash
$ python3 scripts/make_val_subset.py --labels <COCO_HOME>/annotations/person_keypoints_val2017.json
```

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
> ![NOTE]
> 
We took checkpoint after 370000 iterations as the final one.
We did not perform the best checkpoint selection at any step, so similar result may be achieved after less number of iterations.


### Validation

1. Run:
```bash
$ python3 val.py --labels <COCO_HOME>/annotations/person_keypoints_val2017.json --images-folder <COCO_HOME>/val2017 --checkpoint-path <CHECKPOINT>
```

### Pre-trained model

The model expects normalized image (mean=[128, 128, 128], scale=[1/256, 1/256, 1/256]) in planar BGR format.
Pre-trained on COCO model is available at: [checkpoint_iter_370000.pth](https://download.01.org/opencv/openvino_training_extensions/models/human_pose_estimation/checkpoint_iter_370000.pth), it has 40% of AP on COCO validation set (38.6% of AP on the val *subset*).

### Python Demo

We provide python demo just for the quick results preview. Please, consider c++ demo for the best performance. To run the python demo from a webcam:
```bash
$ cd lightweight-human-pose-estimation/script
$ python3 demo.py --checkpoint-path checkpoints/checkpoint_iter_370000.pth --video 0
```
</details>


## Milestone

- [x] OSS
    - [x]  Improved documentation
    - [x]  Unified coding style

See the [open issues][issues-url] for a full list of proposed features (and known issues).

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

- [Real-time 2D Multi-Person Pose Estimation on CPU: Lightweight OpenPose](https://github.com/Daniil-Osokin/lightweight-human-pose-estimation.pytorch)
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