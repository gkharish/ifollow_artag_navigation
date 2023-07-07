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
                        &LiveCamNavCommander::triggerCallBack, this);

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

  std::cout << "LiveCamNavCommander Initialized!" << std::endl; 

}

bool LiveCamNavCommander::triggerCallBack(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) { 
  //testFunc();
  analyzeImage2(image_cur_);
  response.success = true;
  response.message = "Triggerd!";
  std::cout << "triggerCallBack!" << std::endl; 
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

// bool LiveCamNavCommander::analyzeImage(){

//   std::string full_path_where_to_get_image="/home/devcyclair/noetic_ws/src/ifollow_artag_navigation/apriltag_ros/apriltag_ros/ar_tags/ar_webcam.png";
//   std::string full_path_where_to_save_image="/home/devcyclair/noetic_ws/src/ifollow_artag_navigation/apriltag_ros/apriltag_ros/ar_tags/ar_webcam_drawn.png";
//   sensor_msgs::CameraInfo camera_info;
//   camera_info.distortion_model = "plumb_bob";

//   double fx = 643.651478;
//   double fy = 644.265346;
//   double cx = 304.4428;
//   double cy = 226.340608;


//   camera_info.K[0] = fx;
//   camera_info.K[2] = cx;
//   camera_info.K[4] = fy;
//   camera_info.K[5] = cy;
//   camera_info.K[8] = 1.0;
//   camera_info.P[0] = fx;
//   camera_info.P[2] = cx;
//   camera_info.P[5] = fy;
//   camera_info.P[6] = cy;
//   camera_info.P[10] = 1.0;

//   ROS_INFO("[ Summoned to analyze image ]");
//   ROS_INFO("Image load path: %s",
//            full_path_where_to_get_image.c_str());
//   ROS_INFO("Image save path: %s",
//            full_path_where_to_save_image.c_str());

//   // Read the image
//   cv::Mat image = cv::imread(full_path_where_to_get_image,
//                              cv::IMREAD_COLOR);
//   if (image.data == NULL) {
//     // Cannot read image
//     ROS_ERROR_STREAM("Could not read image " <<
//                      full_path_where_to_get_image.c_str());
//     return false;
//   }

//   // Detect tags in the image
//   cv_bridge::CvImagePtr loaded_image(new cv_bridge::CvImage(std_msgs::Header(),
//                                                             "bgr8", image));
//   loaded_image->header.frame_id = "camera";

//   tag_detections_ =
//       tag_detector_.detectTags(loaded_image,sensor_msgs::CameraInfoConstPtr(
//           new sensor_msgs::CameraInfo(camera_info)));

//   // Publish detected tags (AprilTagDetectionArray, basically an array of
//   // geometry_msgs/PoseWithCovarianceStamped)
//   tag_detections_publisher_.publish(tag_detections_);

//   // Send goal to the navstack
//   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>move_ac("move_base", true);
//   // Wait for the action server to come up so that we can begin processing goals.
//   while(!move_ac.waitForServer(ros::Duration(0.50))){
//     ROS_INFO("Waiting for the move_base action server to come up");
//   }

//   move_base_msgs::MoveBaseGoal goal;
//   goal.target_pose.header.frame_id = "map";
//   goal.target_pose.header.stamp = ros::Time::now();

//   // Convert  pose in camera frame to the base_link
//   goal.target_pose.pose.position.x = tag_detections_.detections[0].pose.pose.pose.position.z;
//   goal.target_pose.pose.position.y = -tag_detections_.detections[0].pose.pose.pose.position.x;
//   goal.target_pose.pose.position.z = 0.0;
//   goal.target_pose.pose.orientation.w = 1.0;

//   sendGoal(move_ac, goal);

//   // Save tag detections image
//   tag_detector_.drawDetections(loaded_image);
//   cv::imwrite(full_path_where_to_save_image, loaded_image->image);

//   ROS_INFO("Done!\n");

//   return true;
// }

void LiveCamNavCommander::getRosParams(ros::NodeHandle& pnh){

  camera_info_.distortion_model = "plumb_bob";

  double fx = 643.651478;  // TODO: Get it from param server
  double fy = 644.265346;
  double cx = 304.4428;
  double cy = 226.340608;


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

 
  std::cout << "Ros Param updated!" << std::endl; 
  
}

bool LiveCamNavCommander::analyzeImage2(sensor_msgs::ImageConstPtr  ros_img){
 //std::scoped_lock<std::mutex> lock(detection_mutex_);
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
  tag_detections_publisher_.publish(
      tag_detector_.detectTags(cv_image_, sensor_msgs::CameraInfoConstPtr(
          new sensor_msgs::CameraInfo(camera_info_)) ));

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
  //std::cout << "Img Callback!" << std::endl; 

}

} // namespace artag_nav_commander



int main(int argc, char **argv)
{
  ros::init(argc, argv, "artag_nav_commander");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  artag_nav_commander::LiveCamNavCommander ar_commander(nh, pnh);
  // ar_commander.analyzeImage();
  
  ros::spin();
}
