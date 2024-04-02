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

論文 [Real-time 2D Multi-Person Pose Estimation on CPU: Lightweight OpenPose](https://arxiv.org/pdf/1811.12004.pdf) の学習用コードが実装されたレポジトリをROS1に対応したレポジトリとなります．
画像中にいる人物のポーズを特定するために，骨格（キーポイントとそれらの間の接続で構成される）を検出する．
その中，「耳，目，鼻，首，肩，肘，手首，腰，膝，足首」の最大18個の骨格のリアルタイムに推定できます．
またRGB-Dセンサの点群情報を用いて，2次元の骨格座標だけでなく，3次元の骨格座標を取得することもできる．


<details>
<summary>検出可能な骨格一覧</summary>

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


<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## セットアップ

ここで，本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 環境条件
まず，以下の環境
を整えてから，次のインストール段階に進んでください．

| System  | Version |
| --- | --- |
| Ubuntu  | 20.04 (Focal Fossa) |
| ROS     | Noetic Ninjemys |
| Python  | 3.8 |
| OpenCV  | 4.9.0 |
| PyTorch | >=0.4.1 (Tested on: 2.2.1) |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### インストール方法

1. ROSの`src`フォルダに移動します．
    ```sh
    $ roscd
    # もしくは，"cd ~/catkin_ws/"へ移動．
    $ cd src/
    ```
2. 本レポジトリをcloneします．
    ```sh
    $ git clone https://gitlab.com/TeamSOBITS/lightweight_human_pose_estimation_pytorch
    ```
3. レポジトリの中へ移動します．
    ```sh
    $ cd lightweight_human_pose_estimation_pytorch/
    ```
4. 依存パッケージをインストールします．
    ```sh
    $ bash install.sh
    ```
5. パッケージをコンパイルします．
    ```sh
    $ roscd
    # もしくは，"cd ~/catkin_ws/"へ移動．
    $ catkin_make
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 実行・操作方法 -->
## 実行・操作方法

### カメラの起動

USBカメラ(PC内蔵カメラ)を利用する場合，次のコマンドを実行する．
```bash
roslaunch lightweight_human_pose_estimation camera.launch
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<details>
<summary>USBカメラエラーの対策法</summary>

以下のようなエラーが発生した場合：
```bash
[ERROR] [1663911409.917317256]: Permission denied opening /dev/bus/usb/001/002
```

以下のコードを実行する．
```bash
$ sudo chmod o+w /dev/bus/usb/001/002
```

> [!INFO]
> `/dev/bus/usb/001/002`が変わる可能性がある．表示に応じて，コマンドを修正してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

</details>

> [!INFO]
> [Azure Kinect](https://github.com/TeamSOBITS/azure_kinect_ros_driver) や[RealSenseを利用](https://github.com/TeamSOBITS/realsense_ros) を使用する場合，それぞれのセットアップを済まし，カメラを起動してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 推論パラメータ

2次元の推論時のパラメータを[human_pose.launch](launch/human_pose.launch)に修正することができる．

| パラメータ名 | 型 | 意味 |
| --- | --- | --- |
| checkpoint_path | string | モデルのweightファイルのパス |
| height_size     | int    | 入力画像の拡張 |
| cpu             | bool   | CPUのみで骨格検出するか(CUDAを利用する場合：`false`) |
| smooth          | int    | 前フレームとの骨格をスムーズ化するかのフラグ |
| track           | int    | 前フレームの結果を伝播するかのフラグ |

### 骨格検出の起動

1. 骨格検出関係の機能に応じて[human_pose.launch](launch/human_pose.launch)に修正する．
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

2. launchファイルを実行する．
    ```bash
    roslaunch lightweight_human_pose_estimation human_pose.launch
    ```

### Subscribers & Publishers

- Subscribers:

| トピック名 | 型 | 意味 |
| --- | --- | --- |
| /camera/rgb/image_raw     | sensor_msgs/Image                                 | センサの画像 |
| /camera/depth/points      | sensor_msgs/PointCloud2                           | センサの点群 |
| /human_pose_2d/pose_array | lightweight_human_pose_estimation/KeyPoint2DArray | 2次元の骨格情報 |

- Publishers:

| トピック名 | 型 | 意味 |
| --- | --- | --- |
| /human_pose_2d/pose_array | lightweight_human_pose_estimation/KeyPoint2DArray | 2次元の骨格情報 |
| /human_pose_2d/pose_img   | sensor_msgs/Image                                 | 2次元の骨格画像 |
| /human_pose_3d/pose_array | lightweight_human_pose_estimation/KeyPoints_3d    | 3次元の骨格情報 |


### Services

| サービス名 | 型 | 意味 |
| --- | --- | --- |
| /human_pose_3d/run_ctr | sobits_msgs/RunCtrl | 3次元検出の切り替え(ON:`true`, OFF:`false`) |


## 本モデルの学習

<details>
<summary>詳細</summary>

### 事前設定

1. COCO2017データセットのダウンロード: [http://cocodataset.org/#download](http://cocodataset.org/#download) で(train, val, annotations) と `<COCO_HOME>` フォルダに解凍します．
2. 必要なパッケージをインストール

```bash
$ python3 -m pip install -r requirements.txt
```

### 学習

トレーニングは3つのステップ（完全な検証データセットのAP値が与えられます）:
- MobileNetの重みから学習．
このステップ後の予想APは～38%．
- 前のステップで得られた重みからのトレーニング．
このステップ後に期待されるAPは～39%です．
- 前のステップで得られた重みからのトレーニング．
このステップ後の期待されるAPは～40%です（洗練段階が1のネットワークでは，次の2段階は破棄されます）．

1. 学習済みのMobileNet v1 weightsを`mobilenet_sgd_68.848.pth.tar`からダウンロードします: [https://github.com/marvis/pytorch-mobilenet](https://github.com/marvis/pytorch-mobilenet) (sgd オプション). 指定されたモデルが存在しない場合, [GoogleDrive](https://drive.google.com/file/d/18Ya27IAhILvBHqV_tDp0QjDFvsNNy-hv/view?usp=sharing)からダウンロードしてください．

<!-- 2. Convert train annotations in internal format. Run: -->
2. train annotationsを内部形式に変換する．
その後，prepared_train_annotation.pkl`が生成され，内部形式のannotationsに変換される．
```bash
$ python3 scripts/prepare_train_labels.py --labels <COCO_HOME>/annotations/person_keypoints_train2017.json
```

[任意] 高速な検証のためには，検証データセットの*サブセット*を作成することを推奨する．
val_subset.json`が生成され，（5000枚のうち）ランダムな250枚の画像にannotationsが付加される．
```bash
$ python3 scripts/make_val_subset.py --labels <COCO_HOME>/annotations/person_keypoints_val2017.json
```

3. MobileNetのweightsからトレーニングする
```bash
$ python3 train.py --train-images-folder <COCO_HOME>/train2017/ --prepared-train-labels prepared_train_annotation.pkl --val-labels val_subset.json --val-images-folder <COCO_HOME>/val2017/ --checkpoint-path <path_to>/mobilenet_sgd_68.848.pth.tar --from-mobilenet
```

4. 次に，前のステップのチェックポイントからトレーニングする．
```bash
$ python3 train.py --train-images-folder <COCO_HOME>/train2017/ --prepared-train-labels prepared_train_annotation.pkl --val-labels val_subset.json --val-images-folder <COCO_HOME>/val2017/ --checkpoint-path <path_to>/checkpoint_iter_420000.pth --weights-only
```

5. 最後に，前ステップのチェックポイントと3段階のネットワークから学習する．
370000回学習回数後，最終的なチェックポイントとした．

```bash
$ python3 train.py --train-images-folder <COCO_HOME>/train2017/ --prepared-train-labels prepared_train_annotation.pkl --val-labels val_subset.json --val-images-folder <COCO_HOME>/val2017/ --checkpoint-path <path_to>/checkpoint_iter_280000.pth --weights-only --num-refinement-stages 3
```

最適なチェックポイントを選択したわけではないため，より少ない学習回数で同様の結果が得られる可能性があります．

### 検証

1. 以下を実行する．
```bash
$ python3 val.py --labels <COCO_HOME>/annotations/person_keypoints_val2017.json --images-folder <COCO_HOME>/val2017 --checkpoint-path <CHECKPOINT>
```

### 学習済みモデル

このモデルは，平面BGR形式の正規化画像（mean=[128, 128, 128]，scale=[1/256, 1/256, 1/256] ）を想定している．
COCOで事前に訓練されたモデルは，[checkpoint_iter_370000.pth](https://download.01.org/opencv/openvino_training_extensions/models/human_pose_estimation/checkpoint_iter_370000.pth)であり，COCO検証セットで40％のAPを持っている（val *subset*では38.6％）．

### Pythonデモ

Pythonデモは，簡単な結果のプレビューのために提供しています．
最高のパフォーマンスを得るには，c++デモをご検討ください．ウェブカメラからpythonデモを実行する．
```bash
$ cd lightweight-human-pose-estimation/script
$ python3 demo.py --checkpoint-path checkpoints/checkpoint_iter_370000.pth --video 0
```
</details>


<!-- マイルストーン -->
## マイルストーン

- [x] OSS
    - [x] ドキュメンテーションの充実
    - [x] コーディングスタイルの統一

現時点のバグや新規機能の依頼を確認するために[Issueページ][issues-url] をご覧ください．


## 参考文献
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