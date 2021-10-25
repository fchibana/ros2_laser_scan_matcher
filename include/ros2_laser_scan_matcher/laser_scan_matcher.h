/*
 * Copyright (c) 2011, Ivan Dryanovski, William Morris
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the CCNY Robotics Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*  This package uses Canonical Scan Matcher [1], written by
 *  Andrea Censi
 *
 *  [1] A. Censi, "An ICP variant using a point-to-line metric"
 *  Proceedings of the IEEE International Conference
 *  on Robotics and Automation (ICRA), 2008
 */

#ifndef LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
#define LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include "rclcpp/rclcpp.hpp"

#include <csm/csm.h>  // csm defines min and max, but Eigen complains
#include <boost/thread.hpp>

#include <boost/assign.hpp>   // @fchibana
#include <mutex>              // @fchibana


namespace scan_tools
{
class LaserScanMatcher: public rclcpp::Node
{
public:
  LaserScanMatcher();
  ~LaserScanMatcher();

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg); // fabio
private:
  // Ros handle

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_filter_sub_;
  
  std::shared_ptr<tf2_ros::TransformListener> tf_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfB_;
  tf2::Transform base_to_laser_;  // static, cached
  tf2::Transform laser_to_base_; 
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  // Coordinate parameters
  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string laser_frame_;
  std::string odom_topic_;
  
  // Keyframe parameters
  double kf_dist_linear_;
  double kf_dist_linear_sq_;
  double kf_dist_angular_;

  // TODO(me): Update this comment
  // What predictions are available to speed up the ICP?
  // 1) imu - [theta] from imu yaw angle - /imu topic
  // 2) odom - [x, y, theta] from wheel odometry - /odom topic
  // 3) velocity [vx, vy, vtheta], usually from ab-filter - /vel.
  // If more than one is enabled, priority is imu > odom > velocity 

  // For calculating odometry
  double prev_x;
  double prev_y;
  double prev_angle;

  bool initialized_;
  bool publish_odom_;
  bool publish_tf_;

  tf2::Transform f2b_;     // fixed-to-base tf (pose of base frame in fixed frame)
  tf2::Transform f2b_kf_;  // pose of the last keyframe scan in fixed frame


  tf2::Transform odom_to_base_tf;

  sm_params input_;
  sm_result output_;
  LDP prev_ldp_scan_;

  // Grid map parameters
  double resolution_;

  std::vector<double> a_cos_;
  std::vector<double> a_sin_;


  rclcpp::Time last_icp_time_;

  /*************************************************************************************************
   * (@fchibana) New stuff
   * 
   * Notes
   * - If we create and publish the edge history edge messages in the same scope, we don't need
   *   to declare them as member variables.
  *************************************************************************************************/ 
  
  // For odomCallback()
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry latest_odom_msg_;
  nav_msgs::msg::Odometry last_used_odom_msg_;
  bool use_odom_;
  bool received_odom_;
  std::mutex mutex_;
  std::mutex prediction_mutex_;

  // For messages with covariance 
  std::vector<double> position_covariance_;
  std::vector<double> orientation_covariance_;
  
  // Publishing pose stamped (for mapping?)
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_publisher_;
  std::string pose_stamped_topic_;
  bool publish_pose_stamped_;

  // # stuff for slam (move somewhere else?)
    
  // ## publish edge
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr edge_publisher_;
  geometry_msgs::msg::PoseWithCovarianceStamped edge_stamped_msg_;  // FIXME(): needs to be member var?
  

  // ## publish history edges
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr h_edge_publisher_;
  geometry_msgs::msg::PoseWithCovarianceStamped h_edge_stamped_msg_[50];  // FIXME(): needs to be member var?
  int history_;
  LDP history_pre_ldp_scan_[50];
  tf2::Transform corr_ch_l_old_[50];
  // sensor_msgs::msg::LaserScan::ConstPtr scan_msg_global_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_msg_global_;
  // sensor_msgs::msg::LaserScan::ConstPtr scan_msgs_[50];
  sensor_msgs::msg::LaserScan::SharedPtr scan_msgs_[50];
  /************************************************************************************************/
 
  bool getBaseToLaserTf (const std::string& frame_id);

  bool processScan(LDP& curr_ldp_scan, const rclcpp::Time& time);
  void laserScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr& scan, LDP& ldp);
  void createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t);

  bool newKeyframeNeeded(const tf2::Transform& d);

  void add_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const std::string & description = "", const std::string & additional_constraints = "",
    bool read_only = false);
  void createCache (const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg);

  void getPrediction(double& pr_ch_x, double& pr_ch_y, double& pr_ch_a, double dt);

};  // LaserScanMatcher

}  // namespace scan_tools

#endif  // LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
