

#ifndef VI_RPE_VI_RPE_H_
#define VI_RPE_VI_RPE_H_

#include <glog/logging.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <kindr/minimal/quat-transformation.h>

namespace amsf {

enum class GoalRefFrame{
    MASTER,
    WORLD
};

enum class Mode{
    FORWARD,
    FIX_COV,
    LUT_COV,
    REF,
    REF_KF
};

class ViRpe {
 public:
  ViRpe(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~ViRpe() {}


void slaveImuCallback(const sensor_msgs::ImuConstPtr& msg);
void slaveOdometryCallback(const nav_msgs::Odometry& msg);
void slavePoseAfterUpdateCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
void slaveGoalCallback(const geometry_msgs::PoseStamped& msg);
void relativePoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
void masterOdometryCallback(const nav_msgs::Odometry& msg);
void masterTransformCallback(const geometry_msgs::TransformStamped& msg);
void resetRelativeOffsetCallback(const std_msgs::Empty& );

private:


  void getCovariance( const kindr::minimal::QuatTransformation& pose,  Eigen::Matrix<double, 6, 6>& covariance );
  void reset();
  ros::Publisher relative_curr_pose_pub_;
  ros::Publisher slave_command_pose_pub_;
  ros::Publisher slave_command_trajectory_pub_;

  ros::Subscriber slave_goal_pose_sub_;
  ros::Subscriber slave_imu_sub_;
  ros::Subscriber slave_rel_pose_sub_;
  ros::Subscriber slave_msf_pose_after_update_sub_;
  ros::Subscriber slave_msf_odom_sub_;
  ros::Subscriber master_odom_sub_;
  ros::Subscriber master_transform_sub_;


 ros::NodeHandle nh_;
 ros::NodeHandle nh_private_;

 kindr::minimal::QuatTransformation tf_marker_camera_last_;
 kindr::minimal::QuatTransformation tf_local_camera_last_;
 kindr::minimal::QuatTransformation tf_local_master_;
 kindr::minimal::QuatTransformation tf_local_world_;
 ros::Time timestamp_tf_world_local_;
 kindr::minimal::QuatTransformation tf_local_slave_;
 kindr::minimal::QuatTransformation tf_msf_world__slave_propagation_;

 kindr::minimal::QuatTransformation tf_ref_frame__slave_goal_pose_;
 Mode mode_ = Mode::FORWARD;
 GoalRefFrame goal_ref_frame_ = GoalRefFrame::MASTER; // 0 world // 1 uav_master(relative)

 kindr::minimal::QuatTransformation tf_master_marker_; //TODO: to config file
 kindr::minimal::QuatTransformation tf_camera_slave_; //TODO: to config file


 double max_time_gap_master_odom_ = 3;
 bool running_;
 bool has_valid_goal_;
 bool is_initialized_ = 0; //assume the traking start from a roughly gravity aligned configuration for both UAVs
 int  initialization_counter_ = 0; // wait for few msf iterations
 const int kInit_max_counter_ = 40; //number of iteration that are need to consider as initializade.
 void getYawOnlyTf(const kindr::minimal::QuatTransformation tf_gravity__rotated, kindr::minimal::QuatTransformation &tf_gravity__yaw_only);
};

} //namespece amsf

#endif
