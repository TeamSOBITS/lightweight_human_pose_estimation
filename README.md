<a name="readme-top"></a>

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
      <a href="#実行・操作方法">実行・操作方法</a>
      <ul>
        <li><a href="#カメラの起動">カメラの起動</a></li>
        <li><a href="#2次元骨格検出起動">2次元骨格検出起動</a></li>
        <li><a href="#2dデモ実行">2Dデモ実行</a></li>
        <li><a href="#2d-node">2D Node</a></li>
        <li><a href="#2d-subscriptions">2D Subscriptions</a></li>
        <li><a href="#2d-publications">2D Publications</a></li>
        <li><a href="#2d-parameters">2D Parameters</a></li>
        <li><a href="#3次元骨格検出起動">3次元骨格検出起動</a></li>
        <li><a href="#3dデモ実行">3Dデモ実行</a></li>
        <li><a href="#3d-node">3D Node</a></li>
        <li><a href="#3d-subscriptions">3D Subscriptions</a></li>
        <li><a href="#3d-publications">3D Publications</a></li>
        <li><a href="#3d-parameters">3D Parameters</a></li>
      </ul>
    </li>
    <li><a href="#既知の問題点">既知の問題点</a></li>
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>



## 概要
論文 [Real-time 2D Multi-Person Pose Estimation on CPU: Lightweight OpenPose](https://arxiv.org/pdf/1811.12004.pdf) の学習用コードが実装されたレポジトリをROS1に対応したforkレポジトリとなります。[OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) の手法を大幅に最適化し、CPU上で精度を落とさずにリアルタイム推論ができるようにしたものです。画像中にいる人物のポーズを特定するために、骨格（キーポイントとそれらの間の接続で構成される）を検出する。「耳、目、鼻、首、肩、肘、手首、腰、膝、足首」の最大18個の骨格のリアルタイムに推定できます。

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

1. Download COCO 2017 dataset: [http://cocodataset.org/#download](http://cocodataset.org/#download) (train, val, annotations) and unpack it to `<COCO_HOME>` folder.
2. Install requirements

```bash
$ python3 -m pip install -r requirements.txt
```

### 学習

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


### 検証

1. Run:

```bash
$ python3 val.py --labels <COCO_HOME>/annotations/person_keypoints_val2017.json --images-folder <COCO_HOME>/val2017 --checkpoint-path <CHECKPOINT>
```

### 学習済みモデル

The model expects normalized image (mean=[128, 128, 128], scale=[1/256, 1/256, 1/256]) in planar BGR format.
Pre-trained on COCO model is available at: https://download.01.org/opencv/openvino_training_extensions/models/human_pose_estimation/checkpoint_iter_370000.pth, it has 40% of AP on COCO validation set (38.6% of AP on the val *subset*).

### Pythonデモ

We provide python demo just for the quick results preview. Please, consider c++ demo for the best performance. To run the python demo from a webcam:

```bash
$ cd script
$ python3 demo.py --checkpoint-path checkpoints/checkpoint_iter_370000.pth --video 0
```
</details>


## 実行・操作方法


### カメラの起動
```bash
$ roslaunch lightweight_human_pose_estimation camera.launch
```

※以下のようなエラーが発生した場合：
```bash
[ERROR] [1663911409.917317256]: Permission denied opening /dev/bus/usb/001/002
```

以下のコードを実行してください：
```bash
$ sudo chmod o+w /dev/bus/usb/001/002
```

<details>
<summary>2次元の骨格情報を取得したい場合</summary>

### 2次元骨格検出起動
```bash
$ roslaunch lightweight_human_pose_estimation human_pose_estimation.launch
```

### 2Dデモ実行
```bash
$ roslaunch lightweight_human_pose_estimation demo.launch

```
### 2D Node
|ノード名|意味|
|---|---|
|/human_pose_estimation/lightweight_human_pose_estimation|LightWeight OpenPose 2Dのノード|

### 2D Subscriptions
|トピック名|型|意味|
|---|---|---|
|/camera/rgb/image_raw|sensor_msgs/Image|センサの画像|

### 2D Publications
|トピック名|型|意味|
|---|---|---|
|/human_pose_estimation/pose|lightweight_human_pose_estimation/KeyPoints|2次元の骨格情報|
|/human_pose_estimation/pose|sensor_msgs/Image|2次元の骨格画像|

### 2D Parameters
|パラメータ名|型|意味|
|---|---|---|
|/human_pose_estimation/lightweight_human_pose_estimation/checkpoint_path|string||
|/human_pose_estimation/lightweight_human_pose_estimation/cpu|string||
|/human_pose_estimation/lightweight_human_pose_estimation/needs_time_stamp|string||
|/human_pose_estimation/lightweight_human_pose_estimation/pose_image_topic_name|string||
|/human_pose_estimation/lightweight_human_pose_estimation/pose_img_show_flag|string||
|/human_pose_estimation/lightweight_human_pose_estimation/pose_pub_result_image|string||
|/human_pose_estimation/lightweight_human_pose_estimation/smooth|string||
|/human_pose_estimation/lightweight_human_pose_estimation/track|string||

</details>

<details>
<summary>3次元の骨格情報を取得したい場合</summary>

### 3次元骨格検出起動
```bash
$ roslaunch lightweight_human_pose_estimation human_pose_estimation_3d.launch
```

### 3Dデモ実行
今後：3d版を追加する
<!-- ```bash
$ roslaunch lightweight_human_pose_estimation demo.launch

``` -->
### 3D Node
|ノード名|意味|
|---|---|
|/human_pose_tf_broadcaster|LightWeight OpenPose 3Dのノード|

### 3D Subscriptions
|トピック名|型|意味|
|---|---|---|
|/camera/depth/points|sensor_msgs/PointCloud2|センサの点群|
|/human_pose_estimation/pose|lightweight_human_pose_estimation/KeyPoints|2次元の骨格情報|


### 3D Publications
|トピック名|型|意味|
|---|---|---|
|/lightweight_human_pose_estimation/human_pose_estimation/pose_3d|lightweight_human_pose_estimation/KeyPoints_3d|3次元の骨格情報|


### 3D Parameters
|パラメータ名|型|意味|
|---|---|---|
|/human_pose_tf_broadcaster/base_frame_name|string|ロボットのベースフレーム名|
|/human_pose_tf_broadcaster/cloud_topic_name|string|点群のトピック名|

</details>

## 既知の問題点

We observe this error with maximum number of open files (`ulimit -n`) equals to 1024:

```
  File "train.py", line 164, in <module>
    args.log_after, args.val_labels, args.val_images_folder, args.val_output_name, args.checkpoint_after, args.val_after)
  File "train.py", line 77, in train
    for _, batch_data in enumerate(train_loader):
  File "/<path>/python3.6/site-packages/torch/utils/data/dataloader.py", line 330, in __next__
    idx, batch = self._get_batch()
  File "/<path>/python3.6/site-packages/torch/utils/data/dataloader.py", line 309, in _get_batch
    return self.data_queue.get()
  File "/<path>/python3.6/multiprocessing/queues.py", line 337, in get
    return _ForkingPickler.loads(res)
  File "/<path>/python3.6/site-packages/torch/multiprocessing/reductions.py", line 151, in rebuild_storage_fd
    fd = df.detach()
  File "/<path>/python3.6/multiprocessing/resource_sharer.py", line 58, in detach
    return reduction.recv_handle(conn)
  File "/<path>/python3.6/multiprocessing/reduction.py", line 182, in recv_handle
    return recvfds(s, 1)[0]
  File "/<path>/python3.6/multiprocessing/reduction.py", line 161, in recvfds
    len(ancdata))
RuntimeError: received 0 items of ancdata
```

To get rid of it, increase the limit to bigger number, e.g. 65536, run in the terminal: `ulimit -n 65536`


## 参考文献

If this helps your research, please cite the paper:

```
@inproceedings{osokin2018lightweight_openpose,
    author={Osokin, Daniil},
    title={Real-time 2D Multi-Person Pose Estimation on CPU: Lightweight OpenPose},
    booktitle = {arXiv preprint arXiv:1811.12004},
    year = {2018}
}
```
