
#include <vi_rpe/vi_rpe.h>
#include <mav_msgs/default_topics.h>
#include <minkindr_conversions/kindr_msg.h>
#include <tf/tf.h>

namespace amsf {

ViRpe::ViRpe(const ros::NodeHandle &nh,
             const ros::NodeHandle &nh_private): nh_private_(nh_private), nh_(nh) {

  slave_imu_sub_ = nh_.subscribe("slave_imu_in", 500,
                                 &ViRpe::slaveImuCallback, this);
  slave_rel_pose_sub_ = nh_.subscribe("slave_rel_pose_in", 1,
                                      &ViRpe::relativePoseCallback, this);
  slave_msf_pose_after_update_sub_ = nh_.subscribe("slave_msf_pose_after_update",
                                     1,
                                     &ViRpe::slavePoseAfterUpdateCallback, this);
  slave_msf_odom_sub_ = nh_.subscribe("slave_msf_odom", 1,
                                      &ViRpe::slaveOdometryCallback, this);
  master_odom_sub_ = nh_.subscribe("master_odom_in", 1,
                                   &ViRpe::masterOdometryCallback, this);

  master_transform_sub_ = nh_.subscribe("master_transform_in", 1,
                                   &ViRpe::masterTransformCallback, this);



  slave_goal_pose_sub_ = nh_.subscribe("command/rel_pose", 1,
                                       &ViRpe::slaveGoalCallback, this);

  int mode;
  nh_private.param<int>("mode", mode, 0);

  if (mode == 0) {//todo add the other modes on the public code
    mode_ = Mode::FORWARD;
    ROS_WARN("MODE FORWARD");
  } else {
    mode_ = Mode::REF;
    ROS_WARN("MODE REF");
    
  }

  slave_command_pose_pub_ = nh_private_.advertise < geometry_msgs::PoseStamped >
                            (mav_msgs::default_topics::COMMAND_POSE, 1);
  relative_curr_pose_pub_ = nh_private_.advertise
                            <  geometry_msgs::PoseWithCovarianceStamped > ("pose_with_covariance_out", 5);

  tf_master_marker_ = kindr::minimal::QuatTransformation(
                        kindr::minimal::RotationQuaternion(1, 0, 0, 0), kindr::minimal::Position(0.02,-0.4,-0.38)); // w,x,y,z- xyz
#if(0)//configuration for simulation with 22 angle
    tf_camera_slave_ = kindr::minimal::QuatTransformation(
                       kindr::minimal::RotationQuaternion(0.58633958, -0.3952289, 0.3952289, -0.58633958),
                       kindr::minimal::Position(0, 0, 0.04)).inverse(); // w,x,y,z- xyz
#else // icarus
    tf_camera_slave_ = kindr::minimal::QuatTransformation(
                       kindr::minimal::RotationQuaternion(0.592630180189, 0.411714154089, 0.38449958662, 0.575709121642),
                       kindr::minimal::Position(0.02, -0.005, 0.05)).inverse(); // w,x,y,z- xyz
#endif


  reset();
}

void ViRpe::slaveImuCallback(const sensor_msgs::ImuConstPtr &msg) {

}

void ViRpe::slaveOdometryCallback(const nav_msgs::Odometry &msg) {
  if (is_initialized_) {
    tf::poseMsgToKindr(msg.pose.pose, &tf_msf_world__slave_propagation_);
  } else {
    tf_msf_world__slave_propagation_.setIdentity();
  }
}

void ViRpe::slavePoseAfterUpdateCallback(const
    geometry_msgs::PoseWithCovarianceStamped &msg) {

  if (!is_initialized_) {
    initialization_counter_++;

    if (initialization_counter_ >= kInit_max_counter_) {
      is_initialized_ = true;
      initialization_counter_ = 0;
    }
  }
}

void ViRpe::slaveGoalCallback(const geometry_msgs::PoseStamped &msg) {

  if ((msg.pose.position.x < 1e-6 && msg.pose.position.y < 1e-6 &&
       msg.pose.position.z < 1e-6  && msg.header.frame_id == "/master") ||
      msg.header.frame_id == "/none") {
    ROS_INFO_STREAM("Disabling VI-RPE - goal " <<
                    tf_ref_frame__slave_goal_pose_.asVector());

    has_valid_goal_ = false;
  } else {
    has_valid_goal_ = true;

    tf::poseMsgToKindr(msg.pose, &tf_ref_frame__slave_goal_pose_);

    if (msg.header.frame_id == "/master") {
      goal_ref_frame_ = GoalRefFrame::MASTER;
    } else if (msg.header.frame_id == "/world") {
      goal_ref_frame_ = GoalRefFrame::WORLD;
    } else {
      ROS_ERROR("wrong frame id");
    }

    ROS_INFO_STREAM("new goal === " << msg.pose.position.x << " " <<
                    msg.pose.position.y << " " << msg.pose.position.z << " " << tf::getYaw(
                      msg.pose.orientation) << " === ");
  }
}

void ViRpe::getYawOnlyTf(const kindr::minimal::QuatTransformation
                         tf_gravity__rotated, kindr::minimal::QuatTransformation &tf_gravity__yaw_only) {
  geometry_msgs::Quaternion full_rotated;
  tf::quaternionKindrToMsg(tf_gravity__rotated.getRotation(), &full_rotated);
  double rotated_yaw = tf::getYaw(full_rotated);
  geometry_msgs::Quaternion yaw_only_rotated = tf::createQuaternionMsgFromYaw(
        rotated_yaw);
  kindr::minimal::RotationQuaternion yaw_only_quat;
  tf::quaternionMsgToKindr(yaw_only_rotated, &yaw_only_quat);
  tf_gravity__yaw_only = kindr::minimal::QuatTransformation(yaw_only_quat,
                         tf_gravity__rotated.getPosition());
}

void ViRpe::relativePoseCallback(const
                                 geometry_msgs::PoseWithCovarianceStamped &msg) {


  switch (mode_) {
  case Mode::FORWARD:
  {
    relative_curr_pose_pub_.publish(msg);

    break;
}
  case Mode::REF:
{

    tf::poseMsgToKindr(msg.pose.pose, &tf_marker_camera_last_);

    tf_local_camera_last_ = tf_local_master_ *
                            tf_master_marker_ * tf_marker_camera_last_;
    geometry_msgs::PoseWithCovarianceStamped local_pose_msg = msg;

    tf::poseKindrToMsg(tf_local_camera_last_, &(local_pose_msg.pose.pose));

    relative_curr_pose_pub_.publish(local_pose_msg);

    if (has_valid_goal_) {
      kindr::minimal::QuatTransformation goal;

      if (goal_ref_frame_ == GoalRefFrame::MASTER) {
        kindr::minimal::QuatTransformation tf_local_master_gravity_aligned;
        getYawOnlyTf(tf_local_master_, tf_local_master_gravity_aligned);
        goal = tf_local_master_gravity_aligned * tf_ref_frame__slave_goal_pose_;

      } else if (goal_ref_frame_ == GoalRefFrame::WORLD) {
        goal = tf_local_world_ * tf_ref_frame__slave_goal_pose_;
      }

      geometry_msgs::PoseStamped goalMsg;
      goalMsg.header = local_pose_msg.header;
      tf::poseKindrToMsg(goal, &goalMsg.pose);
      slave_command_pose_pub_.publish(goalMsg);
    }

    break;
}
  default:
{
    ROS_ERROR("invalide mode");
    break;
}
  }


}

void ViRpe::masterOdometryCallback(const nav_msgs::Odometry &msg) {

  kindr::minimal::QuatTransformation tf_world_master;

  tf::poseMsgToKindr(msg.pose.pose, &tf_world_master);

  if (msg.header.stamp.toSec() - timestamp_tf_world_local_.toSec() >
      max_time_gap_master_odom_) {
    if (timestamp_tf_world_local_.toSec() < 1e-9) {
      ROS_INFO("First master odometry msg");
    } else {
      ROS_WARN("Large gap between master odometry messages, offset adjusted");
    }

    tf_local_master_ = tf_local_camera_last_ * tf_marker_camera_last_.inverse() *
                       tf_master_marker_.inverse(); // avoid to move the local frame with relation of the slave

    tf_local_world_ = tf_local_master_ * tf_world_master.inverse();
  } else {

    tf_local_master_ = tf_local_world_ * tf_world_master;
  }

  timestamp_tf_world_local_ = msg.header.stamp;

  return;

  
}

void ViRpe::masterTransformCallback(const geometry_msgs::TransformStamped &msg)
{
    nav_msgs::Odometry odom_msg;
    odom_msg.header = msg.header;
    odom_msg.pose.pose.position.x = msg.transform.translation.x;
    odom_msg.pose.pose.position.y = msg.transform.translation.y;
    odom_msg.pose.pose.position.z = msg.transform.translation.z;
    odom_msg.pose.pose.orientation.x = msg.transform.rotation.x;
    odom_msg.pose.pose.orientation.y = msg.transform.rotation.y;
    odom_msg.pose.pose.orientation.z = msg.transform.rotation.z;
    odom_msg.pose.pose.orientation.w = msg.transform.rotation.w;
    //TODO need to mark that the other fields cannot be used.
    masterOdometryCallback(odom_msg);
}

void ViRpe::resetRelativeOffsetCallback(const std_msgs::Empty &) {
  reset();
}

void ViRpe::reset() {
  timestamp_tf_world_local_ = ros::Time(0, 0);
  tf_local_master_.setIdentity();
  tf_local_world_.setIdentity();
  tf_local_slave_.setIdentity();
  tf_ref_frame__slave_goal_pose_.setIdentity();
  running_ = false;
  is_initialized_ = false;
  initialization_counter_ = 0;
  has_valid_goal_ = false;
}


}//namespace
