<a name="readme-top"></a>


[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# Real-time 2D Multi-Person Pose Estimation on CPU: Lightweight OpenPose (ROS support) 

<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#セットアップ">セットアップ</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li>
      <a href="#本モデルの学習">本モデルの学習</a>
      <ul>
        <li><a href="#事前設定">事前設定</a></li>
        <li><a href="#学習">学習</a></li>
        <li><a href="#検証">検証</a></li>
        <li><a href="#学習済みモデル">学習済みモデル</a></li>
        <li><a href="#pythonデモ">Pythonデモ</a></li>
      </ul>
    </li>
    <li>
      <a href="#実行方法">実行方法</a>
      <ul>
        <li><a href="#カメラの起動">カメラの起動</a></li>
        <li><a href="#2次元骨格検出起動">2次元骨格検出起動</a></li>
        <li><a href="#3次元骨格検出起動">3次元骨格検出起動</a></li>
        <li><a href="#2d-subscriptions">2D Subscriptions</a></li>
        <li><a href="#2d-publications">2D Publications</a></li>
        <li><a href="#3d-subscriptions">3D Subscriptions</a></li>
        <li><a href="#3d-publications">3D Publications</a></li>
        <li><a href="#parameters">Parameters</a></li>
      </ul>
    </li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <li><a href="#最後に">最後に</a></li>
  </ol>
</details>


## 概要
論文 [Real-time 2D Multi-Person Pose Estimation on CPU: Lightweight OpenPose](https://arxiv.org/pdf/1811.12004.pdf) の学習用コードが実装されたレポジトリをROS1に対応したforkレポジトリとなります。[OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) の手法を大幅に最適化し、CPU上で精度を落とさずにリアルタイム推論ができるようにしたものです。画像中にいる人物のポーズを特定するために、骨格（キーポイントとそれらの間の接続で構成される）を検出する。「耳、目、鼻、首、肩、肘、手首、腰、膝、足首」の最大18個の骨格のリアルタイムに推定できます。またRGB-Dセンサの点群情報を用いて、2次元の骨格座標だけでなく、3次元の骨格座標を取得することができます。

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<p align="center">
  <img src="img/preview_2.png" height="320"/>
</p>

## セットアップ


### 環境条件

* Ubuntu 20.04
* Python 3.8
* PyTorch 1.13.1
* OpenCV 4.6.0
* ROS Noetic Ninjemys

### インストール方法

```bash
$ cd ~/catkin_ws/src/
$ git clone https://gitlab.com/TeamSOBITS/lightweight_human_pose_estimation_pytorch.git
$ cd lightweight_human_pose_estimation_pytorch
$ bash install.sh
$ cd ~/catkin_ws/
$ catkin_make 
```

## 本モデルの学習

<details>
<summary>詳細</summary>

### 事前設定

1. COCO2017データセットのダウンロード: [http://cocodataset.org/#download](http://cocodataset.org/#download) で(train, val, annotations) と `<COCO_HOME>` フォルダに解凍します。
2. 必要なパッケージをインストール

```bash
$ python3 -m pip install -r requirements.txt
```

### 学習

トレーニングは3つのステップ（完全な検証データセットのAP値が与えられます）:
* MobileNetの重みから学習。このステップ後の予想APは～38%。
* 前のステップで得られた重みからのトレーニング。このステップ後に期待されるAPは～39%です。
* 前のステップで得られた重みからのトレーニング。このステップ後の期待されるAPは～40%です（洗練段階が1のネットワークでは、次の2段階は破棄されます）。

1. 学習済みのMobileNet v1 weightsを`mobilenet_sgd_68.848.pth.tar`からダウンロードします: [https://github.com/marvis/pytorch-mobilenet](https://github.com/marvis/pytorch-mobilenet) (sgd オプション). うまくいかない場合, [GoogleDrive](https://drive.google.com/file/d/18Ya27IAhILvBHqV_tDp0QjDFvsNNy-hv/view?usp=sharing)からダウンロードしてください。

<!-- 2. Convert train annotations in internal format. Run: -->
2. train annotationsを内部形式に変換します。 以下を実行:
```bash
$ python3 scripts/prepare_train_labels.py --labels <COCO_HOME>/annotations/person_keypoints_train2017.json
```

prepared_train_annotation.pkl`が生成され、内部形式のannotationsに変換されます。
   [オプション] 高速な検証のためには、検証データセットの*サブセット*を作成することを推奨します。以下を実行
```bash
$ python3 scripts/make_val_subset.py --labels <COCO_HOME>/annotations/person_keypoints_val2017.json
```
val_subset.json`が生成され、（5000枚のうち）ランダムな250枚の画像にannotationsが付加されます。

3. MobileNetのweightsからトレーニングするために、以下を実行:

```bash
$ python3 train.py --train-images-folder <COCO_HOME>/train2017/ --prepared-train-labels prepared_train_annotation.pkl --val-labels val_subset.json --val-images-folder <COCO_HOME>/val2017/ --checkpoint-path <path_to>/mobilenet_sgd_68.848.pth.tar --from-mobilenet
```

4. 次に、前のステップのチェックポイントからトレーニングするために、以下を実行:

```bash
$ python3 train.py --train-images-folder <COCO_HOME>/train2017/ --prepared-train-labels prepared_train_annotation.pkl --val-labels val_subset.json --val-images-folder <COCO_HOME>/val2017/ --checkpoint-path <path_to>/checkpoint_iter_420000.pth --weights-only
```

5. 最後に、前ステップのチェックポイントと3段階のネットワークから学習するために、以下を実行：

```bash
$ python3 train.py --train-images-folder <COCO_HOME>/train2017/ --prepared-train-labels prepared_train_annotation.pkl --val-labels val_subset.json --val-images-folder <COCO_HOME>/val2017/ --checkpoint-path <path_to>/checkpoint_iter_280000.pth --weights-only --num-refinement-stages 3
```

370000回繰り返した後のチェックポイントを最終的なチェックポイントとしました。

どのステップでも最適なチェックポイントを選択したわけではないので、より少ない反復回数で同様の結果が得られる可能性があります。

### 検証

1. 以下を実行:

```bash
$ python3 val.py --labels <COCO_HOME>/annotations/person_keypoints_val2017.json --images-folder <COCO_HOME>/val2017 --checkpoint-path <CHECKPOINT>
```

### 学習済みモデル

このモデルは、平面BGR形式の正規化画像（mean=[128, 128, 128]、scale=[1/256, 1/256, 1/256] ）を想定しています。

COCOで事前に訓練されたモデルは、https://download.01.org/opencv/openvino_training_extensions/models/human_pose_estimation/checkpoint_iter_370000.pth、COCO検証セットで40％のAPを持っています（val *subset*では38.6％）。

### Pythonデモ

Pythonデモは、簡単な結果のプレビューのために提供しています。最高のパフォーマンスを得るには、c++デモをご検討ください。ウェブカメラからpythonデモを行うためには、以下を実行：

```bash
$ cd script
$ python3 demo.py --checkpoint-path checkpoints/checkpoint_iter_370000.pth --video 0
```
</details>


## 実行方法


### カメラの起動
USBカメラ(PC内蔵カメラ)を利用する場合
```bash
roslaunch lightweight_human_pose_estimation camera.launch
```
<details>
<summary>※USBカメラエラーの対策法</summary>

以下のようなエラーが発生した場合：
```bash
[ERROR] [1663911409.917317256]: Permission denied opening /dev/bus/usb/001/002
```

以下のコードを実行してください：
```bash
$ sudo chmod o+w /dev/bus/usb/001/002
```
</details>


[Azure Kinectを利用する場合](https://github.com/TeamSOBITS/azure_kinect_ros_driver) 
[RealSenseを利用する場合](https://github.com/TeamSOBITS/realsense_ros) 

### 2次元骨格検出起動
```bash
roslaunch lightweight_human_pose_estimation human_pose_estimation.launch
```
### 3次元骨格検出起動
```bash
$ roslaunch lightweight_human_pose_estimation human_pose_estimation_3d.launch
```

### 2D Subscriptions
|トピック名|型|意味|
|---|---|---|
|/camera/rgb/image_raw|sensor_msgs/Image|センサの画像|

### 2D Publications
|トピック名|型|意味|
|---|---|---|
|/human_pose_estimation/pose|lightweight_human_pose_estimation/KeyPoints|2次元の骨格情報|
|/human_pose_estimation/pose|sensor_msgs/Image|2次元の骨格画像|

### 3D Subscriptions
|トピック名|型|意味|
|---|---|---|
|/camera/depth/points|sensor_msgs/PointCloud2|センサの点群|
|/human_pose_estimation/pose|lightweight_human_pose_estimation/KeyPoints|2次元の骨格情報|


### 3D Publications
|トピック名|型|意味|
|---|---|---|
|/lightweight_human_pose_estimation/human_pose_estimation/pose_3d|lightweight_human_pose_estimation/KeyPoints_3d|3次元の骨格情報|

### Parameters
|パラメータ名|型|意味|
|---|---|---|
|/human_pose_estimation/lightweight_human_pose_estimation/checkpoint_path|string|モデルのweightファイルのパス|
|/human_pose_estimation/lightweight_human_pose_estimation/cpu|bool|CPUのみで骨格検出するか(CUDAを利用する場合：False)|
|/human_pose_estimation/lightweight_human_pose_estimation/pose_image_topic_name|string|センサ画像のトピック名|
|/human_pose_estimation/lightweight_human_pose_estimation/pose_img_show_flag|bool|画像を表示するかのフラグ|
|/human_pose_estimation/lightweight_human_pose_estimation/pose_pub_result_image|bool|骨格検出画像をパブリッシュするかのフラグ|
|/human_pose_estimation/lightweight_human_pose_estimation/smooth|bool|前フレームとの骨格をスムーズ化するかのフラグ|
|/human_pose_estimation/lightweight_human_pose_estimation/track|bool|前フレームの結果を伝播するかのフラグ|

<!-- マイルストーン -->
## マイルストーン

- [x] OSS
    - [x] ドキュメンテーションの充実
    - [x] コーディングスタイルの統一

現時点のバグや新規機能の依頼を確認するために[Issueページ][license-url] をご覧ください．

<!-- ## 参考文献
* [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
* [ROS Noetic](http://wiki.ros.org/noetic)
* [ROS Control](http://wiki.ros.org/ros_control) -->

### 最後に
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