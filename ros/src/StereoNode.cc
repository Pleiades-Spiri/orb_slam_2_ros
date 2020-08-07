#include "StereoNode.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    cv::Mat M1l,M2l,M1r,M2r;

    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() || rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    // initilaize
    StereoNode node (ORB_SLAM2::System::STEREO, node_handle, image_transport,M1l,M2l,M1r,M2r);

    ros::spin();

    return 0;
}


StereoNode::StereoNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, cv::Mat M1l, cv::Mat M2l, cv::Mat M1r, cv::Mat M2r) : Node (sensor, node_handle, image_transport) {
    
    M1l_ = M1l;
    M2l_ = M2l;
    M1r_ = M1r;
    M2r_ = M2r;
    last_confirmed_pose_= cv::Mat::eye(4,4,CV_32F);

    left_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "image_left/image_color_rect", 1);
    right_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "image_right/image_color_rect", 1);

    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *left_sub_, *right_sub_);
    sync_->registerCallback(boost::bind(&StereoNode::ImageCallback, this, _1, _2));
}


StereoNode::~StereoNode () {
    delete left_sub_;
    delete right_sub_;
    delete sync_;
}


void StereoNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight) {
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try {
      cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try {
      cv_ptrRight = cv_bridge::toCvShare(msgRight);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  cv::Mat imLeft, imRight;
  cv::remap(cv_ptrLeft->image,imLeft,this->M1l_,this->M2l_,cv::INTER_LINEAR);
  cv::remap(cv_ptrRight->image,imRight,this->M1r_,this->M2r_,cv::INTER_LINEAR);
  std::cout<<"remapping"<<std::endl;
  std::cout<<msgLeft->header.stamp.toNSec()<<std::endl;
  orb_slam_->TrackStereo(imLeft,imRight,msgLeft->header.stamp.toNSec());
  std::cout<<orb_slam_->GetCurrentPosition()<<std::endl;

  if (orb_slam_->GetCurrentPosition().empty())
  {
	std::cout<<"No possition will reset the map"<<std::endl;
        orb_slam_->Reset();
        orb_slam_->SetCurrentPosition(this->last_confirmed_pose_);
	orb_slam_->SetTrackerPosition(this->last_confirmed_pose_);
	orb_slam_->SetTrackerHasPose();

  }
  else 
  {
	this->last_confirmed_pose_ = orb_slam_->GetCurrentPosition();
  }
  Update ();
}
