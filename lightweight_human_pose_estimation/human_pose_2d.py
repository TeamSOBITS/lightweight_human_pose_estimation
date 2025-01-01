import rclpy
from rclpy.node import Node #ROS2でのノード作成．
from geometry_msgs.msg import Point #3D空間でのポイントを表す
from sensor_msgs.msg import Image # 画像データを表す
from cv_bridge import CvBridge # ROSのImageメッセージとOpenCVのcv::Mat型を相互変換するためのライブラリ

from std_srvs.srv import SetBool
from sobits_interfaces.msg import KeyPoint
from sobits_interfaces.msg import KeyPointArray

import cv2 #OpenCVのPython
import numpy as np #数値計算に使用
import torch #ディープラーニングモデルの実行に使用

# import os
# import sys
# import getpass
# sys.path.append(os.path.join("/home/" + str(getpass.getuser()) + "/colcon_ws/src/lightweight_human_pose_estimation/lightweight_human_pose_estimation/script"))
from modules.model_with_mobilenet import PoseEstimationWithMobileNet #MobileNetを使用したポーズ推定モデル
from modules.keypoints import extract_keypoints, group_keypoints #関節点の抽出とグループ化に関する関数
from modules.load_state import load_state #モデルの状態をロードするための関数
from modules.pose import Pose, track_poses #ポーズ情報を扱うクラスと関数
from modules.val import normalize, pad_width #入力画像の正規化とパディングに関する関数



#img：入力画像
def infer_fast(net, img, net_input_height_size, stride, upsample_ratio, cpu,
               pad_value=(0, 0, 0), img_mean=np.array([128, 128, 128], np.float32), img_scale=np.float32(1/256)):
    height, width, _ = img.shape
    scale = net_input_height_size / height

    scaled_img = cv2.resize(img, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    scaled_img = normalize(scaled_img, img_mean, img_scale)
    min_dims = [net_input_height_size, max(scaled_img.shape[1], net_input_height_size)]
    padded_img, pad = pad_width(scaled_img, stride, pad_value, min_dims)

    tensor_img = torch.from_numpy(padded_img).permute(2, 0, 1).unsqueeze(0).float()
    if not cpu:
        tensor_img = tensor_img.cuda()

    stages_output = net(tensor_img)

    stage2_heatmaps = stages_output[-2]
    heatmaps = np.transpose(stage2_heatmaps.squeeze().cpu().data.numpy(), (1, 2, 0))
    heatmaps = cv2.resize(heatmaps, (0, 0), fx=upsample_ratio, fy=upsample_ratio, interpolation=cv2.INTER_CUBIC)

    stage2_pafs = stages_output[-1]
    pafs = np.transpose(stage2_pafs.squeeze().cpu().data.numpy(), (1, 2, 0))
    pafs = cv2.resize(pafs, (0, 0), fx=upsample_ratio, fy=upsample_ratio, interpolation=cv2.INTER_CUBIC)

    return heatmaps, pafs, scale, pad


class Flame(Node) :
    def __init__(self):
        super().__init__('human_pose_2d')

        self.declare_parameter("input_image_topic", "image_raw")
        self.declare_parameter("weight_file", "checkpoint_iter_370000.pth")
        self.declare_parameter('init_detection', True)
        self.declare_parameter("height_size", 256)
        self.declare_parameter("only_cpu", True)
        self.declare_parameter("track", True)
        self.declare_parameter("smooth", True)
        self.declare_parameter('image_show', True)
        self.declare_parameter("keypoint_name_list", [""])

        self.sub_img_topic_name = self.get_parameter("input_image_topic").get_parameter_value().string_value
        self.checkpoint_path = self.get_parameter("weight_file").get_parameter_value().string_value
        self.pose_2d_detect = self.get_parameter("init_detection").get_parameter_value().bool_value
        self.height_size = self.get_parameter("height_size").get_parameter_value().integer_value
        self.cpu = self.get_parameter("only_cpu").get_parameter_value().bool_value
        self.track = self.get_parameter("track").get_parameter_value().bool_value
        self.smooth = self.get_parameter("smooth").get_parameter_value().bool_value
        self.img_show_flag = self.get_parameter("image_show").get_parameter_value().bool_value
        self.keypoint_name_list = self.get_parameter("keypoint_name_list").get_parameter_value().string_array_value

        self.net = PoseEstimationWithMobileNet()
        self.checkpoint = torch.load(self.checkpoint_path, map_location='cpu')
        load_state(self.net, self.checkpoint)

        self.net = self.net.eval()
        if not self.cpu:
            self.net = self.net.cuda()

        self.stride = 8
        self.upsample_ratio = 4
        self.num_keypoints = Pose.num_kpts # 18
        self.previous_poses = []
        self.delay = 1

        if (len(self.keypoint_name_list) != int(self.num_keypoints)):
            self.get_logger().info("Not Much the number of keypoint, yaml and weight file...")
            self.destroy_node()
            rclpy.shutdown()

        # ROS2 Publisher and Subscriber
        self.pub_result_array = self.create_publisher(KeyPoint2DArray, 'pose_array', 1)
        self.pub_result_img = self.create_publisher(Image, 'pose_img', 1)
        self.sub_img = self.create_subscription(Image, self.sub_img_topic_name, self.img_cb, 10)

        # Start Run_control Service
        self.server = self.create_service(SetBool, 'run_ctr', self.run_ctrl_server)


    # SetBool : Process control
    def run_ctrl_server(self, request, response):
        self.pose_2d_detect = request.data
        response.success = True
        return response

    def img_cb(self, msg):

        if not self.pose_2d_detect:
            return
        
        orig_img = img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        heatmaps, pafs, scale, pad = infer_fast(self.net, img, self.height_size, self.stride, self.upsample_ratio, self.cpu)

        total_keypoints_num = 0
        all_keypoints_by_type = []
        for kpt_idx in range(self.num_keypoints):  # 19th for bg
            total_keypoints_num += extract_keypoints(heatmaps[:, :, kpt_idx], all_keypoints_by_type, total_keypoints_num)
        
        pose_entries, all_keypoints = group_keypoints(all_keypoints_by_type, pafs)
        for kpt_id in range(all_keypoints.shape[0]):
            all_keypoints[kpt_id, 0] = (all_keypoints[kpt_id, 0] * self.stride / self.upsample_ratio - pad[1]) / scale
            all_keypoints[kpt_id, 1] = (all_keypoints[kpt_id, 1] * self.stride / self.upsample_ratio - pad[0]) / scale
        
        current_poses = []
        keypoints_msg = KeyPointArray()
        keypoints_msg.header = msg.header

        for n in range(len(pose_entries)):
            if len(pose_entries[n]) == 0:
                continue

            pose_keypoints = np.ones((self.num_keypoints, 2), dtype=np.int32) * -1
            for kpt_id in range(self.num_keypoints):
                if pose_entries[n][kpt_id] != -1.0:  # keypoint was found
                    pose_keypoints[kpt_id, 0] = int(all_keypoints[int(pose_entries[n][kpt_id]), 0])
                    pose_keypoints[kpt_id, 1] = int(all_keypoints[int(pose_entries[n][kpt_id]), 1])
            
            pose = Pose(pose_keypoints, pose_entries[n][18])
            current_poses.append(pose)

            pose_keypoints_list = pose_keypoints.tolist()

            keypoint_msg = KeyPoint()
            keypoint_msg.score = float(pose_entries[n][18])
            keypoint_msg.key_names = []
            keypoint_msg.key_points = []

            for key_num in range(len(pose_keypoints_list)):
                key_pt = Point()
                key_pt.x = float(pose_keypoints_list[key_num][0])
                key_pt.y = float(pose_keypoints_list[key_num][1])
                key_pt.z = -1.0
                keypoint_msg.key_names += [str(self.keypoint_name_list[key_num])]
                keypoint_msg.key_points += [key_pt]

            keypoints_msg.key_points_array.append(keypoint_msg)


        if self.track:
            track_poses(self.previous_poses, current_poses, smooth=self.smooth)
            self.previous_poses = current_poses

        for pose in current_poses:
            pose.draw(img)
        img = cv2.addWeighted(orig_img, 0.6, img, 0.4, 0)
        for pose in current_poses:
            cv2.rectangle(img, (pose.bbox[0], pose.bbox[1]),
                        (pose.bbox[0] + pose.bbox[2], pose.bbox[1] + pose.bbox[3]), (0, 255, 0))
            if self.track:
                cv2.putText(img, 'id: {}'.format(pose.id), (pose.bbox[0], pose.bbox[1] - 16),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255))
        result_img_msg = CvBridge().cv2_to_imgmsg(img, "bgr8")
        result_img_msg.header = msg.header
        

        if self.img_show_flag:
            cv2.imshow('Lightweight Human Pose Estimation', img)
            key = cv2.waitKey(self.delay)
            # if key == 27:  # esc
            #     self.get_logger().info("Key [ESC] pressed to leave") #ESCキーが押されたことをログに記録
            #     self.destroy_node() #現在のノードを破棄
            #     rclpy.shutdown()
            #     return
            # elif key == 112:  # 'p'
            #     if self.delay == 1:
            #         self.delay = 0
            #     else:
            #         self.delay = 1

        self.pub_result_img.publish(result_img_msg)
        self.pub_result_array.publish(keypoints_msg)



def main():
    rclpy.init()
    flame_node = Flame()
    rclpy.spin(flame_node)
    flame_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()