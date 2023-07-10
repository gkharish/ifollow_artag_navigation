/**

**/

#include "artag_nav_commander/artag_nav_commander_node.h"

#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Header.h>

namespace artag_nav_commander
{

ArtagNavCommander::ArtagNavCommander(ros::NodeHandle& nh, ros::NodeHandle& pnh): tag_detector_(pnh){ 
  //testFunc();
  single_image_analysis_service_ =
    nh.advertiseService("single_image_tag_detection",
                        &ArtagNavCommander::triggerCallBack, this);
  tag_detections_publisher_ =
    nh.advertise<apriltag_ros::AprilTagDetectionArray>("tag_detections", 1);
  ROS_INFO_STREAM("Ready to do tag detection on single images");

  getRosParams(pnh);
  //MoveBaseClient ac_local("fdf", true);
  //move_ac_ = ac_local;
  //waitForNavstack(mv);

}

bool ArtagNavCommander::triggerCallBack(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) { 
  //testFunc();
  analyzeImage();
  response.success = true;
  response.message = "Triggerd!";
  return true;
}


void ArtagNavCommander::waitForNavstack(MoveBaseClient& mv_ac) {
    
  while(!mv_ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server ... ");
  }   
}

void ArtagNavCommander::sendGoal(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal goal) {
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);   

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The ifollow AMR has reached the target!");
  else
    ROS_INFO("Failed to reach the target! Exiting");
         
}

void ArtagNavCommander::getRosParams(ros::NodeHandle& pnh) {
  ros::param::param<double>("fx", cam_intrinsics_.fx, 643.651478);
  ros::param::param<double>("fy", cam_intrinsics_.fy, 644.265346);
  ros::param::param<double>("cx", cam_intrinsics_.cx, 304.4428);
  ros::param::param<double>("cy", cam_intrinsics_.cy, 226.340608);
  ros::param::param<std::string>("distortion_model", cam_intrinsics_.distortion_model, "plumb_bob");

  ros::param::param<std::string>("src_image_path", src_img_, "/home/devcyclair/noetic_ws/src/ifollow_artag_navigation/artag_nav_commander/ar_tags/ar_tag_3.JPG");
  ros::param::param<std::string>("saved_image_path", saved_img_, "/home/devcyclair/noetic_ws/src/ifollow_artag_navigation/artag_nav_commander/ar_tags/ar_webcam_r_drawn3.JPG");

  pnh.getParam("src_image_path", src_img_);
  pnh.getParam("saved_image_path", saved_img_);

  pnh.getParam("fx", cam_intrinsics_.fx);
  pnh.getParam("fy", cam_intrinsics_.fy);
  pnh.getParam("cx", cam_intrinsics_.cx);
  pnh.getParam("cy", cam_intrinsics_.cy);
}

bool ArtagNavCommander::analyzeImage(){

  //std::string full_path_where_to_get_image="/home/devcyclair/noetic_ws/src/ifollow_artag_navigation/apriltag_ros/apriltag_ros/ar_tags/ar_webcam.png";
  //std::string full_path_where_to_save_image="/home/devcyclair/noetic_ws/src/ifollow_artag_navigation/apriltag_ros/apriltag_ros/ar_tags/ar_webcam_drawn.png";
  sensor_msgs::CameraInfo camera_info;
  camera_info.distortion_model = cam_intrinsics_.distortion_model;

  // double fx = 643.651478;
  // double fy = 644.265346;
  // double cx = 304.4428;
  // double cy = 226.340608;


  camera_info.K[0] = cam_intrinsics_.fx;
  camera_info.K[2] = cam_intrinsics_.cx;
  camera_info.K[4] = cam_intrinsics_.fy;
  camera_info.K[5] = cam_intrinsics_.cy;
  camera_info.K[8] = 1.0;
  camera_info.P[0] = cam_intrinsics_.fx;
  camera_info.P[2] = cam_intrinsics_.cx;
  camera_info.P[5] = cam_intrinsics_.fy;
  camera_info.P[6] = cam_intrinsics_.cy;
  camera_info.P[10] = 1.0;

  // getRosParams();

  ROS_INFO("Image load path: %s",
           src_img_.c_str());
  ROS_INFO("Image save path: %s",
           saved_img_.c_str());

  ROS_INFO("Analyzing image ... ");
  // Read the image
  cv::Mat image = cv::imread(src_img_,
                             cv::IMREAD_COLOR);
  if (image.data == NULL) {
    // Cannot read image
    ROS_ERROR_STREAM("Could not read image " <<
                     src_img_.c_str());
    return false;
  }

  // Detect tags in the image
  cv_bridge::CvImagePtr loaded_image(new cv_bridge::CvImage(std_msgs::Header(),
                                                            "bgr8", image));
  loaded_image->header.frame_id = "camera";

  tag_detections_ =
      tag_detector_.detectTags(loaded_image,sensor_msgs::CameraInfoConstPtr(
          new sensor_msgs::CameraInfo(camera_info)));

  
  if(!tag_detections_.detections.size()) {
    ROS_INFO("Tags not detected ... ");

  }
  // Publish detected tags (AprilTagDetectionArray, basically an array of
  // geometry_msgs/PoseWithCovarianceStamped)
  tag_detections_publisher_.publish(tag_detections_);

  // Save tag detections image
  tag_detector_.drawDetections(loaded_image);
  cv::imwrite(saved_img_, loaded_image->image);
  
  // Send goal to the navstack
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>move_ac("move_base", true);

  // Wait for the action server to come up so that we can begin processing goals.
  // while(!move_ac.waitForServer(ros::Duration(0.50))){
  //   ROS_INFO("Waiting for the move_base action server to come up");
  // }

  waitForNavstack(move_ac);


  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  // Convert  pose in camera frame to the base_link
  goal.target_pose.pose.position.x = tag_detections_.detections[0].pose.pose.pose.position.z;
  goal.target_pose.pose.position.y = -tag_detections_.detections[0].pose.pose.pose.position.x;
  goal.target_pose.pose.position.z = 0.0;

  goal.target_pose.pose.orientation.w = 1.0;

  sendGoal(move_ac, goal);



  ROS_INFO("Done!\n");

  return true;
}

} // namespace artag_nav_commander



int main(int argc, char **argv)
{
  ros::init(argc, argv, "artag_nav_commander");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  artag_nav_commander::ArtagNavCommander ar_commander(nh, pnh);
  ar_commander.analyzeImage();
  
  ros::spin();
}
