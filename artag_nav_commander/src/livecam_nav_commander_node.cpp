/**

**/

#include "artag_nav_commander/livecam_nav_commander_node.h"

#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Header.h>

namespace artag_nav_commander
{

LiveCamNavCommander::LiveCamNavCommander(ros::NodeHandle& nh, ros::NodeHandle& pnh): tag_detector_(pnh){ 
  //testFunc();
  getRosParams(pnh);

  capture_image_analysis_service_ =
    nh.advertiseService("livecam_tag_detection",
                        &LiveCamNavCommander::triggerImageCapture, this);

  tag_detections_publisher_ =
    nh.advertise<apriltag_ros::AprilTagDetectionArray>("tag_detections", 1);

  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));

  std::string transport_hint;
  pnh.param<std::string>("transport_hint", transport_hint, "raw");

  int queue_size;
  pnh.param<int>("queue_size", queue_size, 1);
  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", queue_size,
                          &LiveCamNavCommander::imageCallback, this,
                          image_transport::TransportHints(transport_hint));

  pnh.param<bool>("draw_tag_detections_image_", draw_tag_detections_image_, true);
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }

  ROS_INFO("LiveCamNavCommander Initialized!"); 

}

bool LiveCamNavCommander::triggerImageCapture(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) { 
  //testFunc();
  analyzeImage(image_cur_);
  response.success = true;
  response.message = "Triggerd!";
  std::cout << "triggerCallBack!" << std::endl; 
  ROS_INFO("triggered  ImageCapture!"); 
  return true;
}

LiveCamNavCommander::~LiveCamNavCommander(){}

void LiveCamNavCommander::waitForNavstack(MoveBaseClient& mv_ac) {
    
  while(!mv_ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server ... ");
  }   
}

void LiveCamNavCommander::sendGoal(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal goal) {
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);   

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The ifollow AMR has reached the target!");
  else
    ROS_INFO("Failed to reach the target! Exiting");
         
}


void LiveCamNavCommander::getRosParams(ros::NodeHandle& pnh){
  double fx, fy, cx, cy;
  ros::param::param<double>("fx", fx, 643.651478);
  ros::param::param<double>("fy", fy, 644.265346);
  ros::param::param<double>("cx", cx, 304.4428);
  ros::param::param<double>("cy", cy, 226.340608);

  pnh.getParam("fx", fx);
  pnh.getParam("fy", fy);
  pnh.getParam("cx", cx);
  pnh.getParam("cy", cy);

  std::string distortion_model;
  ros::param::param<std::string>("distortion_model", distortion_model, "plumb_bob");
  pnh.getParam("distortion_model", distortion_model);
 
  camera_info_.distortion_model = distortion_model;
  camera_info_.K[0] = fx;
  camera_info_.K[2] = cx;
  camera_info_.K[4] = fy;
  camera_info_.K[5] = cy;
  camera_info_.K[8] = 1.0;
  camera_info_.P[0] = fx;
  camera_info_.P[2] = cx;
  camera_info_.P[5] = fy;
  camera_info_.P[6] = cy;
  camera_info_.P[10] = 1.0;

 
  ROS_INFO("Ros Param updated!"); 
  
}

bool LiveCamNavCommander::analyzeImage(sensor_msgs::ImageConstPtr  ros_img){

 try
  {
    cv_image_ = cv_bridge::toCvCopy(ros_img, ros_img->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  // apriltag detection
  tag_detections_ =  tag_detector_.detectTags( cv_image_, sensor_msgs::CameraInfoConstPtr(
          new sensor_msgs::CameraInfo(camera_info_)) );
  tag_detections_publisher_.publish(tag_detections_);
  std::cout << "Img tag_detections_: " <<tag_detections_.detections.size()  << std::endl; 
  
  if (tag_detections_.detections.size()) {
      ROS_INFO("April Tag detected. Decoding and sending Goal... ");
      // Send goal to the navstack
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>move_ac("move_base", true);
      // Wait for the action server to start!.
      while(!move_ac.waitForServer(ros::Duration(0.50))){
        ROS_INFO("Waiting for the move_base action server ... ");
      }

      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      // Convert  pose in camera frame to the base_link
      goal.target_pose.pose.position.x = tag_detections_.detections[0].pose.pose.pose.position.z;
      goal.target_pose.pose.position.y = -tag_detections_.detections[0].pose.pose.pose.position.x;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.w = 1.0;

      sendGoal(move_ac, goal);
  }
  else {
    ROS_INFO("No tags found in the image or not detected!");
  }


  // If chosen true, draw border lines on the detected tag and publish the payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_.drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }

  return true;

}

void LiveCamNavCommander::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{

  image_cur_ = image_rect;

}

} // namespace artag_nav_commander



int main(int argc, char **argv)
{
  ros::init(argc, argv, "artag_nav_commander");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  artag_nav_commander::LiveCamNavCommander ar_commander(nh, pnh);
    
  ros::spin();
}
