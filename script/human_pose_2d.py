#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from sobits_msgs.srv import RunCtrl, RunCtrlResponse
from lightweight_human_pose_estimation.msg import KeyPoint2D
from lightweight_human_pose_estimation.msg import KeyPoint2DArray

import cv2
import numpy as np
import torch

from models.with_mobilenet import PoseEstimationWithMobileNet
from modules.keypoints import extract_keypoints, group_keypoints
from modules.load_state import load_state
from modules.pose import Pose, track_poses
from val import normalize, pad_width


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

class Flame :
    def __init__(self):
        # Set params for Network
        self.checkpoint_path = rospy.get_param( rospy.get_name() + "/checkpoint_path", "checkpoints/checkpoint_iter_370000.pth")
        self.height_size     = rospy.get_param( rospy.get_name() + "/height_size", 256)
        self.cpu             = rospy.get_param( rospy.get_name() + "/cpu", True )
        self.track           = rospy.get_param( rospy.get_name() + "/track", True )
        self.smooth          = rospy.get_param( rospy.get_name() + "/smooth", True )

        # Set params for ROS
        self.pose_2d_detect  = rospy.get_param( rospy.get_name() + "/pose_2d_detect", True)
        self.img_show_flag   = rospy.get_param( rospy.get_name() + "/pose_2d_img_show", True )
        self.log_show_flag   = rospy.get_param( rospy.get_name() + "/pose_2d_log_show", True )
        self.img_pub_flag    = rospy.get_param( rospy.get_name() + "/pose_2d_img_pub", True )

        self.sub_img_topic_name = rospy.get_param( rospy.get_name() + "/input_image_topic", "/camera/rgb/image_raw" )

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

        # ROS Publisher and Subscriber
        self.pub_result_array = rospy.Publisher("~pose_array", KeyPoint2DArray, queue_size=10)
        self.pub_result_img   = rospy.Publisher("~pose_img", Image, queue_size=10)
        self.sub_img = rospy.Subscriber(self.sub_img_topic_name, Image, self.img_cb)

        # Start Run_control Service
        self.server = rospy.Service("/human_pose_2d/run_ctr", RunCtrl, self.run_ctrl_server)
        self.sub_run_ctrl = rospy.Subscriber("/human_pose_2d/run_ctr", Bool, self.run_ctrl_callback)

        # self.sum = 0
        # self.counter = 0


    # RunCtrl Server
    def run_ctrl_server(self, msg):
        if msg.request:
            self.pose_2d_detect = True
        else:
            self.pose_2d_detect = False
        return RunCtrlResponse(True)
    
    def run_ctrl_callback(self, msg):
        self.pose_2d_detect = msg.data
        rospy.loginfo("run ctrl -> {}".format(self.pose_2d_detect))


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
            keypoint_msg.nose   = Point(pose_keypoints_list[0][0] , pose_keypoints_list[0][1] , -1)
            keypoint_msg.neck   = Point(pose_keypoints_list[1][0] , pose_keypoints_list[1][1] , -1)
            keypoint_msg.r_sho  = Point(pose_keypoints_list[2][0] , pose_keypoints_list[2][1] , -1)
            keypoint_msg.r_elb  = Point(pose_keypoints_list[3][0] , pose_keypoints_list[3][1] , -1)
            keypoint_msg.r_wri  = Point(pose_keypoints_list[4][0] , pose_keypoints_list[4][1] , -1)
            keypoint_msg.l_sho  = Point(pose_keypoints_list[5][0] , pose_keypoints_list[5][1] , -1)
            keypoint_msg.l_elb  = Point(pose_keypoints_list[6][0] , pose_keypoints_list[6][1] , -1)
            keypoint_msg.l_wri  = Point(pose_keypoints_list[7][0] , pose_keypoints_list[7][1] , -1)
            keypoint_msg.r_hip  = Point(pose_keypoints_list[8][0] , pose_keypoints_list[8][1] , -1)
            keypoint_msg.r_knee = Point(pose_keypoints_list[9][0] , pose_keypoints_list[9][1] , -1)
            keypoint_msg.r_ank  = Point(pose_keypoints_list[10][0], pose_keypoints_list[10][1], -1)
            keypoint_msg.l_hip  = Point(pose_keypoints_list[11][0], pose_keypoints_list[11][1], -1)
            keypoint_msg.l_knee = Point(pose_keypoints_list[12][0], pose_keypoints_list[12][1], -1)
            keypoint_msg.l_ank  = Point(pose_keypoints_list[13][0], pose_keypoints_list[13][1], -1)
            keypoint_msg.r_eye  = Point(pose_keypoints_list[14][0], pose_keypoints_list[14][1], -1)
            keypoint_msg.l_eye  = Point(pose_keypoints_list[15][0], pose_keypoints_list[15][1], -1)
            keypoint_msg.r_ear  = Point(pose_keypoints_list[16][0], pose_keypoints_list[16][1], -1)
            keypoint_msg.l_ear  = Point(pose_keypoints_list[17][0], pose_keypoints_list[17][1], -1)

            keypoints_msg.header = msg.header
            keypoints_msg.data.append(keypoint_msg)

        if self.log_show_flag:
            rospy.loginfo("pose2D_keypoint")
            rospy.loginfo(keypoints_msg)

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
                rospy.signal_shutdown("Key [ESC] pressed to leave")
                return
            elif key == 112:  # 'p'
                if self.delay == 1:
                    self.delay = 0
                else:
                    self.delay = 1


if __name__ == '__main__':

    rospy.init_node("demo2_node")
    poseflame = Flame()
    try:
        rospy.spin()
    finally:
        cv2.destroyAllWindows()