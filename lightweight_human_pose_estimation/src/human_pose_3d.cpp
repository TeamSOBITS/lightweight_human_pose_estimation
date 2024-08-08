// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/msg/point.hpp>
// #include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "lightweight_human_pose_estimation_msgs/msg/key_point2_d.hpp"
#include "lightweight_human_pose_estimation_msgs/msg/key_point2_d_array.hpp"
#include "lightweight_human_pose_estimation_msgs/msg/key_point3_d.hpp"
#include "lightweight_human_pose_estimation_msgs/msg/key_point3_d_array.hpp"
#include "sobits_msgs/srv/run_ctrl.h"

// #include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <unordered_map>

typedef pcl::PointXYZ           PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<lightweight_human_pose_estimation::KeyPoint2DArray, sensor_msgs::PointCloud2>
    KeyPointSPointSyncPolicy;

class Pose3D {
    private:
        ros::NodeHandle nh_;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;
        tf2_ros::TransformBroadcaster dynamic_br_;

        std::string     base_frame_name_;
        std::string     pose_2d_topic_name_;
        std::string     cloud_topic_name_;
        std::string     camera_info_topic_name_;

        bool            pose_3d_detect_;
        bool            pose_3d_topic_pub_;
        bool            pose_3d_tf_pub_;
        bool            pose_3d_log_show_;   

        int32_t         camera_width_ = 0;
        PointCloud::Ptr cloud_transformed_;

        ros::Publisher     pub_result_3d_array_;
        ros::ServiceServer srv_subscriber_switch_;
        ros::Subscriber    sub_camera_info_;

        std::unique_ptr<message_filters::Subscriber<lightweight_human_pose_estimation::KeyPoint2DArray>> sub_result_2d_array_;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>                           sub_pcl_;
        std::shared_ptr<message_filters::Synchronizer<KeyPointSPointSyncPolicy>>                         sync_;


        bool callbackSubscriberSwitch( sobits_msgs::RunCtrl::Request &req, sobits_msgs::RunCtrl::Response &res ) {
            pose_3d_detect_ = req.request;
            res.response = true;

            return true;
        }

        void callbackCameraInfo( const sensor_msgs::CameraInfo msg ) {
            // Get the camera info
            camera_width_ = msg.width;

            return;
        }

        void callbackKeyPointsPCL(const lightweight_human_pose_estimation::KeyPoint2DArrayConstPtr &pose_2d_array_msg,
                                  const sensor_msgs::PointCloud2ConstPtr                           &pcl_msg) {
            // Check if the 3D Pose estimation is enabled
            if (!pose_3d_detect_) {
                return;
            }

            // Obtain the camera width
            if (!camera_width_){
                ROS_ERROR("[Human Pose 3D] `camera_width_` was not obtained. Please check the camera_info topic)");
                return;
            }
            else{
                sub_camera_info_.shutdown();
            }

            std::string target_frame_name  = pcl_msg->header.frame_id;
            ros::Time frame_stamp = pcl_msg->header.stamp;
            PointCloud cloud_src;

			// Transform ROS cloud to PCL
            pcl::fromROSMsg(*pcl_msg, cloud_src);

            bool can_tf = tfBuffer_.canTransform(base_frame_name_, target_frame_name, frame_stamp);
            if (!can_tf) {
                ROS_ERROR("[Human Pose 3D] canTransform() failed. `base_frame_name_` and `target_frame_name` might be wrong!)");
                return;
            }

            bool is_tf_pcl = pcl_ros::transformPointCloud(base_frame_name_, cloud_src, *cloud_transformed_, tfBuffer_);
            if (!is_tf_pcl) {
                ROS_ERROR("[Human Pose 3D] transformPointCloud() failed. PointCloud could not be transformed");
                return;
            }

			// Check if there is any keypoint
            lightweight_human_pose_estimation::KeyPoint2DArray pose_2d_array = *pose_2d_array_msg;
            if (pose_2d_array.data.size() == 0) {
                ROS_ERROR("[Human Pose 3D] No human was detected. Skipping the 3D Pose estimation");
                return;
            }

            lightweight_human_pose_estimation::KeyPoint3DArray pose_3d_array;
            lightweight_human_pose_estimation::KeyPoint3D pose_3d;
            std::string body_part_list[18] = {
                "nose", "neck",
                "r_sho", "r_elb", "r_wri", "l_sho", "l_elb", "l_wri",
                "r_hip", "r_knee", "r_ank", "l_hip", "l_knee", "l_ank",
                "r_eye", "l_eye", "r_ear", "l_ear"
            };

            // Human ID
            for (int human_id = 0; human_id < pose_2d_array.data.size(); human_id++) {
                for (std::string body_part : body_part_list) {
                    geometry_msgs::Point body_part_point;
                    int point_x, point_y;
                    
                    if (body_part == "r_eye"){
                        point_x = pose_2d_array.data[human_id].r_eye.x;
                        point_y = pose_2d_array.data[human_id].r_eye.y;
                    }
                    else if (body_part == "l_eye"){
                        point_x = pose_2d_array.data[human_id].l_eye.x;
                        point_y = pose_2d_array.data[human_id].l_eye.y;
                    }
                    else if (body_part == "r_ear"){
                        point_x = pose_2d_array.data[human_id].r_ear.x;
                        point_y = pose_2d_array.data[human_id].r_ear.y;
                    }
                    else if (body_part == "l_ear"){
                        point_x = pose_2d_array.data[human_id].l_ear.x;
                        point_y = pose_2d_array.data[human_id].l_ear.y;
                    }
                    else if (body_part == "nose"){
                        point_x = pose_2d_array.data[human_id].nose.x;
                        point_y = pose_2d_array.data[human_id].nose.y;
                    }
                    else if (body_part == "neck"){
                        point_x = pose_2d_array.data[human_id].neck.x;
                        point_y = pose_2d_array.data[human_id].neck.y;
                    }
                    else if (body_part == "r_sho"){
                        point_x = pose_2d_array.data[human_id].r_sho.x;
                        point_y = pose_2d_array.data[human_id].r_sho.y;
                    }
                    else if (body_part == "l_sho"){
                        point_x = pose_2d_array.data[human_id].l_sho.x;
                        point_y = pose_2d_array.data[human_id].l_sho.y;
                    }
                    else if (body_part == "r_elb"){
                        point_x = pose_2d_array.data[human_id].r_elb.x;
                        point_y = pose_2d_array.data[human_id].r_elb.y;
                    }
                    else if (body_part == "l_elb"){
                        point_x = pose_2d_array.data[human_id].l_elb.x;
                        point_y = pose_2d_array.data[human_id].l_elb.y;
                    }
                    else if (body_part == "r_wri"){
                        point_x = pose_2d_array.data[human_id].r_wri.x;
                        point_y = pose_2d_array.data[human_id].r_wri.y;
                    }
                    else if (body_part == "l_wri"){
                        point_x = pose_2d_array.data[human_id].l_wri.x;
                        point_y = pose_2d_array.data[human_id].l_wri.y;
                    }
                    else if (body_part == "r_hip"){
                        point_x = pose_2d_array.data[human_id].r_hip.x;
                        point_y = pose_2d_array.data[human_id].r_hip.y;
                    }
                    else if (body_part == "l_hip"){
                        point_x = pose_2d_array.data[human_id].l_hip.x;
                        point_y = pose_2d_array.data[human_id].l_hip.y;
                    }
                    else if (body_part == "r_knee"){
                        point_x = pose_2d_array.data[human_id].r_knee.x;
                        point_y = pose_2d_array.data[human_id].r_knee.y;
                    }
                    else if (body_part == "l_knee"){
                        point_x = pose_2d_array.data[human_id].l_knee.x;
                        point_y = pose_2d_array.data[human_id].l_knee.y;
                    }
                    else if (body_part == "r_ank"){
                        point_x = pose_2d_array.data[human_id].r_ank.x;
                        point_y = pose_2d_array.data[human_id].r_ank.y;
                    }
                    else if (body_part == "l_ank"){
                        point_x = pose_2d_array.data[human_id].l_ank.x;
                        point_y = pose_2d_array.data[human_id].l_ank.y;
                    }
                    else{
                        continue;
                    }

                    // Get the 3D Pose(x,y,z) from each 2D Pose(x,y) body part by refering to the Point Cloud
                    if (!(point_x < 0 || point_y < 0)) {
                        PointT transform_coords = cloud_transformed_->points[point_y * camera_width_ + point_x];
                        if (std::isnan(transform_coords.x) || std::isnan(transform_coords.y) || std::isnan(transform_coords.z)) {
                            continue;
                        }

                        body_part_point.x = transform_coords.x;
                        body_part_point.y = transform_coords.y;
                        body_part_point.z = transform_coords.z;
                    }
                    else {
                        body_part_point.x = -1.0;
                        body_part_point.y = -1.0;
                        body_part_point.z = -1.0;
                    }


                    // Copy the obtained data to the 3D Pose msg
                    if (body_part == "r_eye"){
                        pose_3d.r_eye.x = body_part_point.x;
                        pose_3d.r_eye.y = body_part_point.y;
                        pose_3d.r_eye.z = body_part_point.z;
                    }
                    else if (body_part == "l_eye"){
                        pose_3d.l_eye.x = body_part_point.x;
                        pose_3d.l_eye.y = body_part_point.y;
                        pose_3d.l_eye.z = body_part_point.z;
                    }
                    else if (body_part == "r_ear"){
                        pose_3d.r_ear.x = body_part_point.x;
                        pose_3d.r_ear.y = body_part_point.y;
                        pose_3d.r_ear.z = body_part_point.z;
                    }
                    else if (body_part == "l_ear"){
                        pose_3d.l_ear.x = body_part_point.x;
                        pose_3d.l_ear.y = body_part_point.y;
                        pose_3d.l_ear.z = body_part_point.z;
                    }
                    else if (body_part == "nose"){
                        pose_3d.nose.x = body_part_point.x;
                        pose_3d.nose.y = body_part_point.y;
                        pose_3d.nose.z = body_part_point.z;
                    }
                    else if (body_part == "neck"){
                        pose_3d.neck.x = body_part_point.x;
                        pose_3d.neck.y = body_part_point.y;
                        pose_3d.neck.z = body_part_point.z;
                    }
                    else if (body_part == "r_sho"){
                        pose_3d.r_sho.x = body_part_point.x;
                        pose_3d.r_sho.y = body_part_point.y;
                        pose_3d.r_sho.z = body_part_point.z;
                    }
                    else if (body_part == "l_sho"){
                        pose_3d.l_sho.x = body_part_point.x;
                        pose_3d.l_sho.y = body_part_point.y;
                        pose_3d.l_sho.z = body_part_point.z;
                    }
                    else if (body_part == "r_elb"){
                        pose_3d.r_elb.x = body_part_point.x;
                        pose_3d.r_elb.y = body_part_point.y;
                        pose_3d.r_elb.z = body_part_point.z;
                    }
                    else if (body_part == "l_elb"){
                        pose_3d.l_elb.x = body_part_point.x;
                        pose_3d.l_elb.y = body_part_point.y;
                        pose_3d.l_elb.z = body_part_point.z;
                    }
                    else if (body_part == "r_wri"){
                        pose_3d.r_wri.x = body_part_point.x;
                        pose_3d.r_wri.y = body_part_point.y;
                        pose_3d.r_wri.z = body_part_point.z;
                    }
                    else if (body_part == "l_wri"){
                        pose_3d.l_wri.x = body_part_point.x;
                        pose_3d.l_wri.y = body_part_point.y;
                        pose_3d.l_wri.z = body_part_point.z;
                    }
                    else if (body_part == "r_hip"){
                        pose_3d.r_hip.x = body_part_point.x;
                        pose_3d.r_hip.y = body_part_point.y;
                        pose_3d.r_hip.z = body_part_point.z;
                    }
                    else if (body_part == "l_hip"){
                        pose_3d.l_hip.x = body_part_point.x;
                        pose_3d.l_hip.y = body_part_point.y;
                        pose_3d.l_hip.z = body_part_point.z;
                    }
                    else if (body_part == "r_knee"){
                        pose_3d.r_knee.x = body_part_point.x;
                        pose_3d.r_knee.y = body_part_point.y;
                        pose_3d.r_knee.z = body_part_point.z;
                    }
                    else if (body_part == "l_knee"){
                        pose_3d.l_knee.x = body_part_point.x;
                        pose_3d.l_knee.y = body_part_point.y;
                        pose_3d.l_knee.z = body_part_point.z;
                    }
                    else if (body_part == "r_ank"){
                        pose_3d.r_ank.x = body_part_point.x;
                        pose_3d.r_ank.y = body_part_point.y;
                        pose_3d.r_ank.z = body_part_point.z;
                    }
                    else if (body_part == "l_ank"){
                        pose_3d.l_ank.x = body_part_point.x;
                        pose_3d.l_ank.y = body_part_point.y;
                        pose_3d.l_ank.z = body_part_point.z;
                    }
                    else{
                        ROS_ERROR("It is not a body part! (No idea if this happens)");
                        return;
                    }

                    // Publish the 3D Pose TF
                    if (pose_3d_tf_pub_) {
                        if (!(body_part_point.x < 0.0)) {
                            geometry_msgs::TransformStamped transformStamped;
                            transformStamped.header.stamp = frame_stamp;
                            transformStamped.header.frame_id = base_frame_name_;
                            transformStamped.child_frame_id = pose_2d_array.data[human_id].name + "_" + body_part;
                            transformStamped.transform.translation.x = body_part_point.x;
                            transformStamped.transform.translation.y = body_part_point.y;
                            transformStamped.transform.translation.z = body_part_point.z;
                            transformStamped.transform.rotation.w = 1.0;
                            dynamic_br_.sendTransform(transformStamped);
                        }
                    }

                }
                // Introduce the data into the msg
                pose_3d.name = pose_2d_array.data[human_id].name;
                pose_3d.confidence = pose_2d_array.data[human_id].confidence;

                pose_3d_array.header = pose_2d_array.header;
                pose_3d_array.data.push_back(pose_3d);

                // Show the 3D Pose data
                if (pose_3d_log_show_) {
                    ROS_INFO("Human ID: %d", human_id);
                    ROS_INFO("Name: %s", pose_2d_array.data[human_id].name.c_str());
                    ROS_INFO("Confidence: %f", pose_2d_array.data[human_id].confidence);
                    ROS_INFO("r_eye:  (%f, %f, %f)", pose_3d.r_eye.x , pose_3d.r_eye.y , pose_3d.r_eye.z);
                    ROS_INFO("l_eye:  (%f, %f, %f)", pose_3d.l_eye.x , pose_3d.l_eye.y , pose_3d.l_eye.z);
                    ROS_INFO("r_ear:  (%f, %f, %f)", pose_3d.r_ear.x , pose_3d.r_ear.y , pose_3d.r_ear.z);
                    ROS_INFO("l_ear:  (%f, %f, %f)", pose_3d.l_ear.x , pose_3d.l_ear.y , pose_3d.l_ear.z);
                    ROS_INFO("nose:   (%f, %f, %f)", pose_3d.nose.x  , pose_3d.nose.y  , pose_3d.nose.z);
                    ROS_INFO("neck:   (%f, %f, %f)", pose_3d.neck.x  , pose_3d.neck.y  , pose_3d.neck.z);
                    ROS_INFO("r_sho:  (%f, %f, %f)", pose_3d.r_sho.x , pose_3d.r_sho.y , pose_3d.r_sho.z);
                    ROS_INFO("l_sho:  (%f, %f, %f)", pose_3d.l_sho.x , pose_3d.l_sho.y , pose_3d.l_sho.z);
                    ROS_INFO("r_elb:  (%f, %f, %f)", pose_3d.r_elb.x , pose_3d.r_elb.y , pose_3d.r_elb.z);
                    ROS_INFO("l_elb:  (%f, %f, %f)", pose_3d.l_elb.x , pose_3d.l_elb.y , pose_3d.l_elb.z);
                    ROS_INFO("r_wri:  (%f, %f, %f)", pose_3d.r_wri.x , pose_3d.r_wri.y , pose_3d.r_wri.z);
                    ROS_INFO("l_wri:  (%f, %f, %f)", pose_3d.l_wri.x , pose_3d.l_wri.y , pose_3d.l_wri.z);
                    ROS_INFO("r_hip:  (%f, %f, %f)", pose_3d.r_hip.x , pose_3d.r_hip.y , pose_3d.r_hip.z);
                    ROS_INFO("l_hip:  (%f, %f, %f)", pose_3d.l_hip.x , pose_3d.l_hip.y , pose_3d.l_hip.z);
                    ROS_INFO("r_knee: (%f, %f, %f)", pose_3d.r_knee.x, pose_3d.r_knee.y, pose_3d.r_knee.z);
                    ROS_INFO("l_knee: (%f, %f, %f)", pose_3d.l_knee.x, pose_3d.l_knee.y, pose_3d.l_knee.z);
                    ROS_INFO("r_ank:  (%f, %f, %f)", pose_3d.r_ank.x , pose_3d.r_ank.y , pose_3d.r_ank.z);
                    ROS_INFO("l_ank:  (%f, %f, %f)", pose_3d.l_ank.x , pose_3d.l_ank.y , pose_3d.l_ank.z);
                }
            }

            // Publish the result of the 3D Pose estimation
            if (pose_3d_topic_pub_) pub_result_3d_array_.publish(pose_3d_array);
        }

    public:
        Pose3D() : nh_("~"), tfBuffer_(), tfListener_(tfBuffer_) {
            // Get params from launcher
            nh_.param<std::string>("base_frame_name", base_frame_name_, "base_footprint");
            nh_.param<std::string>("pose_2d_topic_name", pose_2d_topic_name_, "/human_pose_2d/pose_array");
            nh_.param<std::string>("cloud_topic_name", cloud_topic_name_, "/camera/depth/points");
            nh_.param<std::string>("camera_info_topic_name", camera_info_topic_name_, "/camera/rgb/camera_info");

            // check! it might be depth camera_info

            nh_.param<bool>("pose_3d_detect", pose_3d_detect_, true);
            nh_.param<bool>("pose_3d_topic_pub", pose_3d_topic_pub_, true);
            nh_.param<bool>("pose_3d_tf_pub", pose_3d_tf_pub_, true);
            nh_.param<bool>("pose_3d_log_show", pose_3d_log_show_, true);

            std::cout << "human_pose_3d[base_frame_name]: " << base_frame_name_ << std::endl;
            std::cout << "human_pose_3d[pose_2d_topic_name]: " << pose_2d_topic_name_ << std::endl;
            std::cout << "human_pose_3d[cloud_topic_name]: " << cloud_topic_name_ << std::endl;
            std::cout << "human_pose_3d[camera_info_topic_name]: " << camera_info_topic_name_ << std::endl;

            std::cout << "human_pose_3d[pose_3d_detect]: " << pose_3d_detect_ << std::endl;
            std::cout << "human_pose_3d[pose_3d_topic_pub]: " << pose_3d_topic_pub_ << std::endl;
            std::cout << "human_pose_3d[pose_3d_tf_pub]: " << pose_3d_tf_pub_ << std::endl;
            std::cout << "human_pose_3d[pose_3d_log_show]: " << pose_3d_log_show_ << std::endl;

            // ROS publishers and subscribers
            pub_result_3d_array_ = nh_.advertise<lightweight_human_pose_estimation::KeyPoint3DArray>("pose_array", 10);
            srv_subscriber_switch_ = nh_.advertiseService( "run_ctr", &Pose3D::callbackSubscriberSwitch, this);
            sub_camera_info_ = nh_.subscribe(camera_info_topic_name_, 1, &Pose3D::callbackCameraInfo, this);
            sub_result_2d_array_.reset(new message_filters::Subscriber<lightweight_human_pose_estimation::KeyPoint2DArray>(nh_, pose_2d_topic_name_, 5));
            sub_pcl_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud_topic_name_, 5));
            
            // Initialize the Point Cloud
            cloud_transformed_.reset(new PointCloud());

            // Synchronize the 2D Pose result and the Point Cloud
            sync_.reset(new message_filters::Synchronizer<KeyPointSPointSyncPolicy>(
                KeyPointSPointSyncPolicy(100), *sub_result_2d_array_, *sub_pcl_));
            sync_->registerCallback(boost::bind(&Pose3D::callbackKeyPointsPCL, this, _1, _2));
        }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "human_pose_3d");
    // ros::Rate rate(5);

    Pose3D           pose_3d;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}