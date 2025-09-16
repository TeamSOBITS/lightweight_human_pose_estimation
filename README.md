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
      <a href="#実行・操作方法">実行方法</a>
      <ul>
        <li><a href="#カメラの起動">カメラの起動</a></li>
        <li><a href="#推論パラメータ">推論パラメータ</a></li>
        <li><a href="#骨格検出の起動">骨格検出の起動</a></li>
        <li><a href="#subscribers--publishers">Subscribers & Publishers</a></li>
        <li><a href="#services">Services</a></li>
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
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>



## 概要

本パッケージは，論文 [Real-time 2D Multi-Person Pose Estimation on CPU: Lightweight OpenPose](https://arxiv.org/pdf/1811.12004.pdf) をベースに，ROS 2 環境に対応させた骨格推定ノードです．
カメラ画像から人物のポーズを認識し，「耳・目・鼻・首・肩・肘・手首・腰・膝・足首」の最大18か所のキーポイントをリアルタイムに推定します。  
さらに，RGB-Dセンサの点群情報を利用することで，2次元の骨格座標に加えて3次元の骨格座標も推定することができます。  


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


## 環境構築
ここで，本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 環境条件
まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS 2  | Humble Hawksbill |
| Python | 3.10 |
| OpenCV | 4.9系 |
| PyTorch | 2.2 以上 |
| NumPy | 1.26系 |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### インストール方法
1. ワークスペースの`src`フォルダに移動します．
    ```sh
    cd ~/colcon_ws/src/
    ```
2. 本リポジトリをcloneします．
    ```sh
    git clone -b humble-devel https://github.com/TeamSOBITS/lightweight_human_pose_estimation.git
    ```
3. レポジトリの中へ移動します．
    ```sh
    cd lightweight_human_pose_estimation/
    ```
4. 依存パッケージをインストールします．  
    ```sh
    bash install.sh
    ```
5. パッケージをコンパイルします．
    ```sh
    cd ~/colcon_ws/
    ```
    ```sh
    colcon build --symlink-install
    ```
    ```sh
    source ~/colcon_ws/install/setup.sh
    ```
<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 実行・操作方法 -->
## 実行・操作方法

### 内部カメラの起動

1. v4l2_camera のインストール  
   ```bash
   sudo apt update
   ```
   ```sh
   sudo apt install -y ros-humble-v4l2-camera
   ```
2. カメラの起動
  ```sh
  ros2 run v4l2_camera v4l2_camera_node
  ```


<details>
<summary>USBカメラエラーの対策法</summary>

USBカメラを起動した際に、以下のようなエラーが発生した場合：
```bash
[ERROR] [1663911409.917317256]: Permission denied opening /dev/bus/usb/001/002
```
この場合は、デバイスファイルに一時的に書き込み権限を与える．
```bash
$ sudo chmod o+w /dev/bus/usb/001/002
```

> [!NOTE]
> `/dev/bus/usb/001/002`が変わる可能性がある．表示に応じて，コマンドを修正してください．

</details>

> [!NOTE]
> [Azure_Kinect](https://github.com/TeamSOBITS/azure_kinect_ros_driver) や[RealSense](https://github.com/TeamSOBITS/realsense_ros/tree/humble-devel)，[Orbbec](https://github.com/TeamSOBITS/orbbecsdk_ros2)を使用する場合，それぞれのセットアップを済まし，カメラを起動してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### パラメータ

[human_pose.launch](https://github.com/TeamSOBITS/lightweight_human_pose_estimation/blob/humble-devel/launch/human_pose.launch.py)では以下のパラメーターを指定できます．

| パラメータ名 | 型 | 既定値 | 説明 |
|---|---|---|---|
| `input_image_topic` | string | `/image_raw` |入力RGB画像のトピック名．使用するカメラに合わせて変更する|
| `point_cloud_topic` | string | `/points2` |3D推定で点群を用いる場合ののトピック名|
| `depth_image_topic_name` | string | `/depth_to_rgb/image_raw` |3D推定で深度画像を用いる場合のトピック名|
| `info_topic_name` | string | `/rgb/camera_info` |カメラの内部パラメータのトピック名|
| `positioning_detection_mode` | enum(string) | `point_cloud` |`point_cloud` または `depth_image` の3D位置推定の方式を指定できる |
| `weight_file` | string | `<pkg_share>/weights/checkpoint_iter_370000.pth` |学習済みモデルファイルへのパス|
| `base_frame_name` | string | `base_footprint` |3D推定で参照するベースフレーム名|
| `enable_id` | bool | `False` |3D推定（image_to_position）側で人物にIDを付与するための設定|
| `execute_default` | bool |`True` | `True`：起動直後から推定を実行，`False`：待機状態で起動|
| `height_size` | int | `256` |入力画像の高さを基準にリサイズ．値を小さくすると処理が速くなるが，精度は下がる|
| `only_cpu` | bool | `False` |`True`：CPU，`False`：GPUを使用|
| `track` | bool | `True` |2Dで前フレーム結果を利用し，対応付けてトラッキングする|
| `smooth` | bool | `True` |推定した関節位置を前後フレームで平滑化して滑らかにする|
| `image_show` | bool | `False` |画像ウィンドウを表示する|
| `namespace` | string | `human_pose` |ノードの名前空間|
| `use_3d` | bool | `True` |2D推定結果を3D座標に変換する処理．内部カメラなどのRGBカメラのみの場合は`False`にする|


### 骨格検出の起動
launchファイルを実行する．
  ```bash
  ros2 launch lightweight_human_pose_estimationhuman_pose.launch.py
  ```

<details>
<summary>Pythonバージョンのエラーが出た場合</summary>

1) pip版 OpenCV を完全に除去
```sh
python3 -m pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless || true
# 残骸の cv2.so が /usr/local にあると読み込まれるため削除（存在すれば）
sudo rm -f /usr/local/lib/python3.10/dist-packages/cv2*.so || true
```
2) NumPy を 1.26 系に固定（必須）
```sh
python3 -m pip install -U "pip<25"
python3 -m pip uninstall -y numpy || true
python3 -m pip install "numpy<2,>=1.26.0"
```
3) OpenCV は APT 版を使用（cv_bridge と相性が良い）
```sh
sudo apt-get update
sudo apt-get install -y python3-opencv
```
4) pycocotools を NumPy 1.26 に合わせて入れ直し
```sh
python3 -m pip install --no-binary=pycocotools --no-build-isolation --force-reinstall pycocotools==2.0.10
```
5) 動作確認（同じシェルで実行）
```sh
python3 - <<'PY'
import numpy, cv2
print("NumPy:", numpy.__version__, "(OK: 1.26.x)")
print("OpenCV:", cv2.__version__, "path:", cv2.__file__, "(OK: /usr/lib/python3/dist-packages/...)")
PY
```
6) 再ビルドして起動
```sh
cd ~/colcon_ws
colcon build --symlink-install
source ~/colcon_ws/install/setup.sh
```
</details>


### Subscribers & Publishers

- Subscribers:

| トピック名 | 型 | 意味 |
| ---| ---| --- |
| `/image_raw`              | `sensor_msgs/msg/Image`       | 内部カメラやUSBカメラの入力画像                                      |
| `/points2`                | `sensor_msgs/msg/PointCloud2` | センサの点群    |
| `/depth_to_rgb/image_raw` | `sensor_msgs/msg/Image`       | 深度画像 |
| `/rgb/camera_info`        | `sensor_msgs/msg/CameraInfo`  | RGBカメラの内部パラメータ |


- Publishers:

| トピック名                           | 型                                     | 意味 |
| --- | --- | --- |
| `/human_pose/pose_array`        | `sobits_interfaces/msg/KeyPointArray` | 2次元の骨格キーポイント情報  |
| `/human_pose/pose_img`          | `sensor_msgs/msg/Image`               | 骨格を重畳した2次元画像 |
| `/human_pose/keypoint_3d_array` | `sobits_interfaces/msg/KeyPointArray` | 3次元の骨格キーポイント情報（`use_3d:=true` の場合）|

- System Topics:

| トピック名               | 型                                   | 意味              |
| --- | ---　| --- |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | ノードのパラメータ変更イベント |
| `/tf`               | `tf2_msgs/msg/TFMessage`            | 座標変換情報（動的）      |
| `/tf_static`        | `tf2_msgs/msg/TFMessage`            | 座標変換情報（静的）      |



### Services

| サービス名 | 型 | 意味 |
| --- | --- | -- |
| `/human_pose/run_ctr`                                  | `std_srvs/srv/SetBool`                       | 2D骨格検出の開始/停止（ON:`true`, OFF:`false`） |
| `/human_pose/keypoints/run_ctr`                        | `std_srvs/srv/SetBool`                       | 3D骨格変換の開始/停止（ON:`true`, OFF:`false`） |
| `/human_pose/human_pose_2d/describe_parameters`        | `rcl_interfaces/srv/DescribeParameters`      | 2Dノードのパラメータ情報を取得                     |
| `/human_pose/human_pose_2d/get_parameters`             | `rcl_interfaces/srv/GetParameters`           | 2Dノードのパラメータ値を取得                      |
| `/human_pose/human_pose_2d/get_parameter_types`        | `rcl_interfaces/srv/GetParameterTypes`       | 2Dノードのパラメータ型を取得                      |
| `/human_pose/human_pose_2d/list_parameters`            | `rcl_interfaces/srv/ListParameters`          | 2Dノードのパラメータ一覧を取得                     |
| `/human_pose/human_pose_2d/set_parameters`             | `rcl_interfaces/srv/SetParameters`           | 2Dノードのパラメータを設定                       |
| `/human_pose/human_pose_2d/set_parameters_atomically`  | `rcl_interfaces/srv/SetParametersAtomically` | 2Dノードのパラメータをまとめて設定                   |
| `/human_pose/keypoint_to_3d/describe_parameters`       | `rcl_interfaces/srv/DescribeParameters`      | 3Dノードのパラメータ情報を取得                     |
| `/human_pose/keypoint_to_3d/get_parameters`            | `rcl_interfaces/srv/GetParameters`           | 3Dノードのパラメータ値を取得                      |
| `/human_pose/keypoint_to_3d/get_parameter_types`       | `rcl_interfaces/srv/GetParameterTypes`       | 3Dノードのパラメータ型を取得                      |
| `/human_pose/keypoint_to_3d/list_parameters`           | `rcl_interfaces/srv/ListParameters`          | 3Dノードのパラメータ一覧を取得                     |
| `/human_pose/keypoint_to_3d/set_parameters`            | `rcl_interfaces/srv/SetParameters`           | 3Dノードのパラメータを設定                       |
| `/human_pose/keypoint_to_3d/set_parameters_atomically` | `rcl_interfaces/srv/SetParametersAtomically` | 3Dノードのパラメータをまとめて設定                   |



## 本モデルの学習

<details>
<summary>詳細</summary>

### 事前設定

ワークエリアを作成
```sh
cd ~/colcon_ws/src
```
```sh
mkdir -p pose_training && cd pose_training
```
```sh
git clone https://github.com/Daniil-Osokin/lightweight-human-pose-estimation.pytorch
```
```sh
cd lightweight-human-pose-estimation.pytorch
```
COCO2017データセットのダウンロード: [http://cocodataset.org/#download](http://cocodataset.org/#download) で(train, val, annotations) を取得する．

COCO を用意（ここを <COCO_HOME> とする）
```sh
cd ~/colcon_ws/src/pose_training
```
```sh
mkdir -p COCO2017/annotations
```
```sh
cd COCO2017
```
画像をダウンロード
train2017.zip(18GB：画像118,000枚)の場合：

```sh
wget -c http://images.cocodataset.org/zips/val2017.zip
```
画像をダウンロードサブセットval2017.zip(1GB：画像5000枚)の場合：
```sh
wget -c http://images.cocodataset.org/zips/val2017.zip
```

アノテーションをダウンロード
```sh
wget -c http://images.cocodataset.org/annotations/annotations_trainval2017.zip
```
zipファイルの解答と削除
```sh
unzip -q val2017.zip
```
```sh
unzip -q annotations_trainval2017.zip
```
```sh
rm -f val2017.zip annotations_trainval2017.zip
```
以下のような階層となる
```sh
COCO2017/
├── val2017/                         # 画像フォルダ (約5,000枚の検証用画像)
│   ├── 000000000139.jpg
│   ├── 000000000285.jpg
│   └── ...
└── annotations/                     # アノテーション JSON 一式
    ├── captions_train2017.json
    ├── captions_val2017.json
    ├── instances_train2017.json
    ├── instances_val2017.json
    ├── person_keypoints_train2017.json
    └── person_keypoints_val2017.json
```
サブセットでの作成と前処理を行う場合：
```sh
cd ~/colcon_ws/src/pose_training/lightweight-human-pose-estimation.pytorch
```
val のアノテーションから、ランダムで 250 枚のサブセット JSON を作成
```sh
python3 scripts/make_val_subset.py \
  --labels ../COCO2017/annotations/person_keypoints_val2017.json
```
擬似学習用の前処理を行う
```sh
python3 scripts/prepare_train_labels.py \
  --labels ../COCO2017/annotations/person_keypoints_val2017.json
```
### 学習

トレーニングは3つのステップ（完全な検証データセットのAP値が与えられます）:
- MobileNetの重みから学習．
このステップ後の予想APは～38%．
- 前のステップで得られた重みからのトレーニング．
このステップ後に期待されるAPは～39%です．
- 前のステップで得られた重みからのトレーニング．
このステップ後の期待されるAPは～40%です（洗練段階が1のネットワークでは，次の2段階は破棄されます）．

1. 学習済みのMobileNet v2 weightsをダウンロードする: 
```sh
cd ~/colcon_ws/src/pose_training/lightweight-human-pose-estimation.pytorch
```
```sh
wget -c https://download.pytorch.org/models/mobilenet_v2-b0353104.pth
```
torchvision の重みを想定形式に変換する
```sh
python3 - <<'PY'
import torch
from torchvision.models import mobilenet_v2, MobileNet_V2_Weights
```


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
val.py = 精度評価用
1. 以下を実行する．
```bash
$ python3 val.py --labels <COCO_HOME>/annotations/person_keypoints_val2017.json --images-folder <COCO_HOME>/val2017 --checkpoint-path <CHECKPOINT>
```

### 学習済みモデル

このモデルは，平面BGR形式の正規化画像（mean=[128, 128, 128]，scale=[1/256, 1/256, 1/256] ）を想定している．
COCOで事前に訓練されたモデルは，[checkpoint_iter_370000.pth](https://download.01.org/opencv/openvino_training_extensions/models/human_pose_estimation/checkpoint_iter_370000.pth)であり，COCO検証セットで40％のAPを持っている（val *subset*では38.6％）．

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
