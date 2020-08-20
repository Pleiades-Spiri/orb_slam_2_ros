#pragma once

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "System.h"
#include "Node.h"

class StereoNode : public Node
{
  public:
    cv::Mat M1l_,M2l_,M1r_,M2r_;
    cv::Mat last_confirmed_pose_;
    StereoNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport,cv::Mat M1l_,cv::Mat M2l_,cv::Mat M1r_,cv::Mat M2r_);
    ~StereoNode ();
    void ImageCallback (const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight,const geometry_msgs::PoseStampedConstPtr& msgLoPose);

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped> sync_pol;
    message_filters::Subscriber<sensor_msgs::Image> *left_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *right_sub_;
    message_filters::Synchronizer<sync_pol> *sync_;

	  //Adding subscriber to the pose published by flight controller localPose
    message_filters::Subscriber<geometry_msgs::PoseStamped> *local_pose_sub_;



};

