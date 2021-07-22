#include "kitti_odom.h"

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "kitti_odom_node");
  ros::NodeHandle nh("~");

  int scan_num = 1200;
  std::string sequence_dir = "/media/ganlu/PERL-SSD/Datasets/KITTI/dataset/sequences/10/";

  KittiOdomData kitti_odom_data(nh);
  kitti_odom_data.read_left_camera_poses("/media/ganlu/PERL-SSD/Datasets/KITTI/dataset/poses/10.txt");
  kitti_odom_data.process_scans(sequence_dir, scan_num);
  ros::spin();

  return 0;
}
