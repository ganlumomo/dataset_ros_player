#include "mini_cheetah.h"

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "mini_cheetah_node");
  ros::NodeHandle nh("~");

  MiniCheetahData mini_cheetah_data(nh);
  std::string pose_file;
  nh.getParam("/mini_cheetah_node/pose_file", pose_file);
  mini_cheetah_data.read_color_camera_poses_tum(pose_file);
  
  ros::spin();

  return 0;
}
