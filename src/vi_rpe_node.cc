
#include "vi_rpe/vi_rpe.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "vi_rpe");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  amsf::ViRpe node(ros::NodeHandle(), ros::NodeHandle("~"));

  ros::spin();

  return 0;
}
