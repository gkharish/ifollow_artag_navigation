/**

**/

#ifndef LIVE_CAM_NAV_COMMANDER_NODE_H
#define LIVE_CAM_NAV_COMMANDER_NODE_H

#include <apriltag_ros/common_functions.h>
#include <apriltag_ros/AnalyzeSingleImage.h>
#include <std_srvs/Trigger.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <memory>
#include <mutex>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace artag_nav_commander
{
    struct camera_intrisics {
        double fx = 643.651478;
        double fy = 644.265346;
        double cx = 304.4428;
        double cy = 226.340608;

    };

class LiveCamNavCommander
{
    public:
     LiveCamNavCommander(ros::NodeHandle& nh, ros::NodeHandle& pnh);
     ~LiveCamNavCommander();

    bool triggerCallBack(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
    void testFunc();
    
    //bool analyzeImage();
    void waitForNavstack(MoveBaseClient& ac);
    void sendGoal(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal goal);

    void analyzeImage2(sensor_msgs::ImageConstPtr  ros_img );

    void imageCallback(const sensor_msgs::ImageConstPtr& image_rect,
                     const sensor_msgs::CameraInfoConstPtr& camera_info);

    void getRosParams(ros::NodeHandle& pnh);

    private:
     apriltag_ros::TagDetector tag_detector_;
     apriltag_ros::AprilTagDetectionArray tag_detections_;
     ros::ServiceServer capture_image_analysis_service_;
     ros::Publisher tag_detections_publisher_;
     sensor_msgs::CameraInfo camera_info_;

     std::mutex detection_mutex_;
     bool draw_tag_detections_image_;
     cv_bridge::CvImagePtr cv_image_;

     std::shared_ptr<image_transport::ImageTransport> it_;
     image_transport::CameraSubscriber camera_image_subscriber_;
     image_transport::Publisher tag_detections_image_publisher_;

     sensor_msgs::ImageConstPtr image_cur_;

     //MoveBaseClient move_ac_;
};

} // artag_nav_commander

#endif // ARTAG_NAV_COMMANDER_NODE_H
