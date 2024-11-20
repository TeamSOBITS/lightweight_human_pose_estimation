#! /usr/bin/env python3
# import rospy
import rclpy
from rclpy.node import Node #ROS2でのノード作成．
from geometry_msgs.msg import Point #3D空間でのポイントを表す
from sensor_msgs.msg import Image # 画像データを表す
from std_msgs.msg import Bool # ブール値
from cv_bridge import CvBridge # ROSのImageメッセージとOpenCVのcv::Mat型を相互変換するためのライブラリ
from sobits_interfaces.srv import RunCtrl # サービス通信のため
from lightweight_human_pose_estimation_msgs.msg import KeyPoint2D #検出された2Dの関節点情報を表す
from lightweight_human_pose_estimation_msgs.msg import KeyPoint2DArray #検出された2Dの関節点情報を表す

import cv2 #OpenCVのPython
import numpy as np #数値計算に使用
import torch #ディープラーニングモデルの実行に使用

import os
import sys
import getpass
sys.path.append(os.path.join("/home/" + str(getpass.getuser()) + "/colcon_ws/src/lightweight_human_pose_estimation/lightweight_human_pose_estimation/script"))
from models.with_mobilenet import PoseEstimationWithMobileNet #MobileNetを使用したポーズ推定モデル
from modules.keypoints import extract_keypoints, group_keypoints #関節点の抽出とグループ化に関する関数
from modules.load_state import load_state #モデルの状態をロードするための関数
from modules.pose import Pose, track_poses #ポーズ情報を扱うクラスと関数
from val import normalize, pad_width #入力画像の正規化とパディングに関する関数

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
        super().__init__('flame_node') # フレームノードとして設定
        self.declare_parameters(
            namespace='',
            parameters=[
                ('checkpoint_path', '/home/sobits/colcon_ws/src/lightweight_human_pose_estimation/lightweight_human_pose_estimation/script/checkpoints/checkpoint_iter_370000.pth'),
                ('height_size', 256),
                ('cpu', True),
                ('track', True),
                ('smooth', True),
                ('pose_2d_detect', True),
                ('pose_2d_img_show', True),
                ('pose_2d_log_show', True),
                ('pose_2d_img_pub', True),
                ('input_image_topic', '/camera/camera/color/image_raw')
            ]
        )
        self.checkpoint_path = self.get_parameter("checkpoint_path").get_parameter_value().string_value
        self.height_size = self.get_parameter("height_size").get_parameter_value().integer_value
        self.cpu = self.get_parameter("cpu").get_parameter_value().bool_value
        self.track = self.get_parameter("track").get_parameter_value().bool_value
        self.smooth = self.get_parameter("smooth").get_parameter_value().bool_value
        self.pose_2d_detect = self.get_parameter("pose_2d_detect").get_parameter_value().bool_value
        self.img_show_flag = self.get_parameter("pose_2d_img_show").get_parameter_value().bool_value
        self.log_show_flag = self.get_parameter("pose_2d_log_show").get_parameter_value().bool_value
        self.img_pub_flag = self.get_parameter("pose_2d_img_pub").get_parameter_value().bool_value
        self.sub_img_topic_name = self.get_parameter("input_image_topic").get_parameter_value().string_value

        print("\033[33m", self.checkpoint_path, "\033[0m")
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

        # ROS Publisher and Subscriber → ここもROS2化する．rospy.publisherとrospy.Subscriberを修正．
        """
        self.pub_result_array = rospy.Publisher("~pose_array", KeyPoint2DArray, queue_size=10) # KeyPoint2DArray型検出された2Dポーズの関節点情報を表す．
        self.pub_result_img   = rospy.Publisher("~pose_img", Image, queue_size=10) #Image型のメッセージを送信，処理後の画像データを表す．
        self.sub_img = rospy.Subscriber(self.sub_img_topic_name, Image, self.img_cb) #Image型のメッセージを受信，デフォルトでは、カメラの生画像トピックである"/camera/rgb/image_raw"が設定されてる．
        """
        # ROS2 Publisher and Subscriber
        self.pub_result_array = self.create_publisher(KeyPoint2DArray, '/human_pose_2d/pose_array', 10)
        self.pub_result_img = self.create_publisher(Image, '/human_pose_2d/pose_img', 10)
        self.sub_img = self.create_subscription(Image, self.sub_img_topic_name, self.img_cb, 10)
        
        """
        # Start Run_control Service
        self.server = rospy.Service("/human_pose_2d/run_ctr", RunCtrl, self.run_ctrl_server)
        self.sub_run_ctrl = rospy.Subscriber("/human_pose_2d/run_ctr", Bool, self.run_ctrl_callback)
        """
        # Start Run_control Service
        self.server = self.create_service(RunCtrl, '/human_pose_2d/run_ctr', self.run_ctrl_server)
        # self.sub_run_ctrl = self.create_subscription(Bool, '/human_pose_2d/run_ctr', self.run_ctrl_callback, 10)
        
        # self.sum = 0
        # self.counter = 0


    # RunCtrl Server→ROS2化
    """
    def run_ctrl_server(self, msg):
        if msg.request:
            self.pose_2d_detect = True
        else:
            self.pose_2d_detect = False
        return RunCtrlResponse(True)
    """
    def run_ctrl_server(self, request, response):
        self.pose_2d_detect = request.request
        return response
    
    """
    def run_ctrl_callback(self, msg):
        self.pose_2d_detect = msg.data
        rospy.loginfo("run ctrl -> {}".format(self.pose_2d_detect))
    """
    # def run_ctrl_callback(self, msg):
    #     self.pose_2d_detect = msg.data
    #     self.get_logger().info("run ctrl -> {}".format(self.pose_2d_detect))

        #↑ここまで修正

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
        keypoints_msg = KeyPoint2DArray()

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

            keypoint_msg = KeyPoint2D()
            keypoint_msg.name = 'person_{}'.format(n)
            keypoint_msg.confidence = pose_entries[n][18]
            # keypoint_msg.nose   = Point(pose_keypoints_list[0][0] , pose_keypoints_list[0][1] , -1)
            # keypoint_msg.neck   = Point(pose_keypoints_list[1][0] , pose_keypoints_list[1][1] , -1)
            # keypoint_msg.r_sho  = Point(pose_keypoints_list[2][0] , pose_keypoints_list[2][1] , -1)
            # keypoint_msg.r_elb  = Point(pose_keypoints_list[3][0] , pose_keypoints_list[3][1] , -1)
            # keypoint_msg.r_wri  = Point(pose_keypoints_list[4][0] , pose_keypoints_list[4][1] , -1)
            # keypoint_msg.l_sho  = Point(pose_keypoints_list[5][0] , pose_keypoints_list[5][1] , -1)
            # keypoint_msg.l_elb  = Point(pose_keypoints_list[6][0] , pose_keypoints_list[6][1] , -1)
            # keypoint_msg.l_wri  = Point(pose_keypoints_list[7][0] , pose_keypoints_list[7][1] , -1)
            # keypoint_msg.r_hip  = Point(pose_keypoints_list[8][0] , pose_keypoints_list[8][1] , -1)
            # keypoint_msg.r_knee = Point(pose_keypoints_list[9][0] , pose_keypoints_list[9][1] , -1)
            # keypoint_msg.r_ank  = Point(pose_keypoints_list[10][0], pose_keypoints_list[10][1], -1)
            # keypoint_msg.l_hip  = Point(pose_keypoints_list[11][0], pose_keypoints_list[11][1], -1)
            # keypoint_msg.l_knee = Point(pose_keypoints_list[12][0], pose_keypoints_list[12][1], -1)
            # keypoint_msg.l_ank  = Point(pose_keypoints_list[13][0], pose_keypoints_list[13][1], -1)
            # keypoint_msg.r_eye  = Point(pose_keypoints_list[14][0], pose_keypoints_list[14][1], -1)
            # keypoint_msg.l_eye  = Point(pose_keypoints_list[15][0], pose_keypoints_list[15][1], -1)
            # keypoint_msg.r_ear  = Point(pose_keypoints_list[16][0], pose_keypoints_list[16][1], -1)
            # keypoint_msg.l_ear  = Point(pose_keypoints_list[17][0], pose_keypoints_list[17][1], -1
            nose = Point()
            nose.x = float(pose_keypoints_list[0][0])
            nose.y = float(pose_keypoints_list[0][1])
            nose.z = -1.0
            keypoint_msg.nose = nose

            neck = Point()
            neck.x = float(pose_keypoints_list[1][0])
            neck.y = float(pose_keypoints_list[1][1])
            neck.z = -1.0
            keypoint_msg.neck = neck

            r_sho = Point()
            r_sho.x = float(pose_keypoints_list[2][0])
            r_sho.y = float(pose_keypoints_list[2][1])
            r_sho.z = -1.0
            keypoint_msg.r_sho = r_sho

            r_elb = Point()
            r_elb.x = float(pose_keypoints_list[3][0])
            r_elb.y = float(pose_keypoints_list[3][1])
            r_elb.z = -1.0
            keypoint_msg.r_elb = r_elb

            r_wri = Point()
            r_wri.x = float(pose_keypoints_list[4][0])
            r_wri.y = float(pose_keypoints_list[4][1])
            r_wri.z = -1.0
            keypoint_msg.r_wri = r_wri

            l_sho = Point()
            l_sho.x = float(pose_keypoints_list[5][0])
            l_sho.y = float(pose_keypoints_list[5][1])
            l_sho.z = -1.0
            keypoint_msg.l_sho = l_sho

            l_elb = Point()
            l_elb.x = float(pose_keypoints_list[6][0])
            l_elb.y = float(pose_keypoints_list[6][1])
            l_elb.z = -1.0
            keypoint_msg.l_elb = l_elb

            l_wri = Point()
            l_wri.x = float(pose_keypoints_list[7][0])
            l_wri.y = float(pose_keypoints_list[7][1])
            l_wri.z = -1.0
            keypoint_msg.l_wri = l_wri

            r_hip = Point()
            r_hip.x = float(pose_keypoints_list[8][0])
            r_hip.y = float(pose_keypoints_list[8][1])
            r_hip.z = -1.0
            keypoint_msg.r_hip = r_hip

            r_knee = Point()
            r_knee.x = float(pose_keypoints_list[9][0])
            r_knee.y = float(pose_keypoints_list[9][1])
            r_knee.z = -1.0
            keypoint_msg.r_knee = r_knee

            r_ank = Point()
            r_ank.x = float(pose_keypoints_list[10][0])
            r_ank.y = float(pose_keypoints_list[10][1])
            r_ank.z = -1.0
            keypoint_msg.r_ank = r_ank

            l_hip = Point()
            l_hip.x = float(pose_keypoints_list[11][0])
            l_hip.y = float(pose_keypoints_list[11][1])
            l_hip.z = -1.0
            keypoint_msg.l_hip = l_hip

            l_knee = Point()
            l_knee.x = float(pose_keypoints_list[12][0])
            l_knee.y = float(pose_keypoints_list[12][1])
            l_knee.z = -1.0
            keypoint_msg.l_knee = l_knee

            l_ank = Point()
            l_ank.x = float(pose_keypoints_list[13][0])
            l_ank.y = float(pose_keypoints_list[13][1])
            l_ank.z = -1.0
            keypoint_msg.l_ank = l_ank

            r_eye = Point()
            r_eye.x = float(pose_keypoints_list[14][0])
            r_eye.y = float(pose_keypoints_list[14][1])
            r_eye.z = -1.0
            keypoint_msg.r_eye = r_eye

            l_eye = Point()
            l_eye.x = float(pose_keypoints_list[15][0])
            l_eye.y = float(pose_keypoints_list[15][1])
            l_eye.z = -1.0
            keypoint_msg.l_eye = l_eye

            r_ear = Point()
            r_ear.x = float(pose_keypoints_list[16][0])
            r_ear.y = float(pose_keypoints_list[16][1])
            r_ear.z = -1.0
            keypoint_msg.r_ear = r_ear

            l_ear = Point()
            l_ear.x = float(pose_keypoints_list[17][0])
            l_ear.y = float(pose_keypoints_list[17][1])
            l_ear.z = -1.0
            keypoint_msg.l_ear = l_ear

            keypoints_msg.header = msg.header
            keypoints_msg.data.append(keypoint_msg)

            """表示方法をROS2化
        if self.log_show_flag:
            rospy.loginfo("pose2D_keypoint")
            rospy.loginfo(keypoints_msg)
            """
        if self.log_show_flag:
            self.get_logger().info("pose2D_keypoint")
            self.get_logger().info(str(keypoints_msg))


        self.pub_result_array.publish(keypoints_msg)


        if self.track:
            track_poses(self.previous_poses, current_poses, smooth=self.smooth)
            self.previous_poses = current_poses

        if self.img_pub_flag or self.img_show_flag:
            for pose in current_poses:
                pose.draw(img)

            img = cv2.addWeighted(orig_img, 0.6, img, 0.4, 0)

            for pose in current_poses:
                cv2.rectangle(img, (pose.bbox[0], pose.bbox[1]),
                            (pose.bbox[0] + pose.bbox[2], pose.bbox[1] + pose.bbox[3]), (0, 255, 0))
                if self.track:
                    cv2.putText(img, 'id: {}'.format(pose.id), (pose.bbox[0], pose.bbox[1] - 16),
                                cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255))

        if self.img_pub_flag:
            result_img_msg = CvBridge().cv2_to_imgmsg(img, "bgr8")
            # result_img_msg.header.seq = self.counter
            # result_img_msg.header.stamp = rospy.Time.now()
            result_img_msg.header = msg.header
            
            self.pub_result_img.publish(result_img_msg)

            # self.counter += 1

        if self.img_show_flag:
            cv2.imshow('Lightweight Human Pose Estimation Python Demo', img)
                
            key = cv2.waitKey(self.delay)
            if key == 27:  # esc
                # rospy.signal_shutdown("Key [ESC] pressed to leave")
                self.get_logger().info("Key [ESC] pressed to leave") #ESCキーが押されたことをログに記録
                self.destroy_node() #現在のノードを破棄
                rclpy.shutdown()
                return
            elif key == 112:  # 'p'
                if self.delay == 1:
                    self.delay = 0
                else:
                    self.delay = 1




def main():
    rclpy.init()
    flame_node = Flame()
    rclpy.spin(flame_node)
    flame_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
"""
if __name__ == '__main__':
    rospy.init_node("demo2_node")
    rclpy.init()
    poseflame = Flame()
    try:
        rospy.spin()
    finally:
        cv2.destroyAllWindows()

"""

"""
修正した箇所
ROS1: rospy.get_param() でパラメータを取得するが，ROS2: パラメータを使用するには，まずdeclare_parameter()で宣言し，その後get_parameter()で取得する必要がある．
ROS1: rospy.Publisher()とrospy.Subscriber()を使用するが，ROS2: self.create_publisher()とself.create_subscription()を使用する．
ROS1: rospy.Service() でサービスを作成するが，ROS2: self.create_service() でサービスを作成する．
ROS1: rospy.loginfo()で表示するが，ROS2: self.get_logger().info()で表示する．


"""