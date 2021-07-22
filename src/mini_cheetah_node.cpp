#include "mini_cheetah.h"

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "mini_cheetah_node");
  ros::NodeHandle nh("~");

  MiniCheetahData mini_cheetah_data(nh);
  mini_cheetah_data.read_color_camera_poses("/media/ganlu/Samsung_T5/0000_mini-cheetah/2021-05-29_Forest_Sidewalk_Rock_Data/rgbd_orbslam_trajectory_MAir_to_forest_0-170.txt");
  
  ros::spin();

  return 0;
}
