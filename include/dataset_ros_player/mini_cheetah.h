#pragma once

#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Geometry>

class MiniCheetahData{
  public:
    MiniCheetahData(ros::NodeHandle& nh) : nh_(nh), scan_id_(0) {
      color_image_subscriber_ = nh_.subscribe("/camera/color/image_raw", 1, &MiniCheetahData::image_callback, this);
      color_camera_pose_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("color_camera_pose", 1);
    }

    void image_callback(const sensor_msgs::Image::ConstPtr& image_msg) {
      ros::Time stamp = image_msg->header.stamp;
      Eigen::Matrix4d color_camera_pose = color_camera_poses_[scan_id_];
      tf::Transform tf_pose;
      Eigen::Affine3d eigen_affine_pose(color_camera_pose);
      tf::transformEigenToTF(eigen_affine_pose, tf_pose);

      // publish tf and pose
      br_.sendTransform(tf::StampedTransform(tf_pose, stamp, "map", "camera_depth_optical_frame"));
      publish_pose_cov(eigen_affine_pose, stamp);
      scan_id_++;
    }

    void publish_pose_cov(Eigen::Affine3d pose, ros::Time t) {
      geometry_msgs::PoseWithCovarianceStamped pose_msg;
      pose_msg.header.frame_id = "camera_depth_optical_frame";
      pose_msg.header.stamp = t;

      // set pose
      tf::poseEigenToMsg(pose, pose_msg.pose.pose);

      // set cov
      pose_msg.pose.covariance[6*0+0] = 0.0;
      pose_msg.pose.covariance[6*0+1] = 0.0;
      pose_msg.pose.covariance[6*0+2] = 0.0;
      pose_msg.pose.covariance[6*0+3] = 0.0;
      pose_msg.pose.covariance[6*0+4] = 0.0;
      pose_msg.pose.covariance[6*0+5] = 0.0;
      pose_msg.pose.covariance[6*1+0] = 0.0;
      pose_msg.pose.covariance[6*1+1] = 0.0;
      pose_msg.pose.covariance[6*1+2] = 0.0;
      pose_msg.pose.covariance[6*1+3] = 0.0;
      pose_msg.pose.covariance[6*1+4] = 0.0;
      pose_msg.pose.covariance[6*1+5] = 0.0;
      pose_msg.pose.covariance[6*2+0] = 0.0;
      pose_msg.pose.covariance[6*2+1] = 0.0;
      pose_msg.pose.covariance[6*2+2] = 0.0;
      pose_msg.pose.covariance[6*2+3] = 0.0;
      pose_msg.pose.covariance[6*2+4] = 0.0;
      pose_msg.pose.covariance[6*2+5] = 0.0;
      pose_msg.pose.covariance[6*3+0] = 0.0;
      pose_msg.pose.covariance[6*3+1] = 0.0;
      pose_msg.pose.covariance[6*3+2] = 0.0;
      pose_msg.pose.covariance[6*3+3] = 0.0;
      pose_msg.pose.covariance[6*3+4] = 0.0;
      pose_msg.pose.covariance[6*3+5] = 0.0;
      pose_msg.pose.covariance[6*4+0] = 0.0;
      pose_msg.pose.covariance[6*4+1] = 0.0;
      pose_msg.pose.covariance[6*4+2] = 0.0;
      pose_msg.pose.covariance[6*4+3] = 0.0;
      pose_msg.pose.covariance[6*4+4] = 0.0;
      pose_msg.pose.covariance[6*4+5] = 0.0;
      pose_msg.pose.covariance[6*5+0] = 0.0;
      pose_msg.pose.covariance[6*5+1] = 0.0;
      pose_msg.pose.covariance[6*5+2] = 0.0;
      pose_msg.pose.covariance[6*5+3] = 0.0;
      pose_msg.pose.covariance[6*5+4] = 0.0;
      pose_msg.pose.covariance[6*5+5] = 0.0;

      // publish
      color_camera_pose_publisher_.publish(pose_msg);
    }

    bool read_color_camera_poses(const std::string pose_name) {
      if (std::ifstream(pose_name)) {
        std::ifstream fPoses;
        fPoses.open(pose_name.c_str());
        while (!fPoses.eof()) {
          std::string s;
          std::getline(fPoses, s);
          if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Identity();
            for (int i = 0; i < 3; ++i)
              for (int j = 0; j < 4; ++j)
                ss >> t_matrix(i, j);
            color_camera_poses_.push_back(t_matrix);
          }
        }
        fPoses.close();
        return true;
        } else {
         ROS_ERROR_STREAM("Cannot open pose file " << pose_name);
         return false;
      }
    }

  private:
    int scan_id_;
    std::vector<Eigen::Matrix4d> color_camera_poses_;

    ros::NodeHandle nh_;
    ros::Subscriber color_image_subscriber_;
    ros::Publisher color_camera_pose_publisher_;
    tf::TransformBroadcaster br_;
};
