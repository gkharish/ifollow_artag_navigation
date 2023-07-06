/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 *
 ** single_image_detector.h ****************************************************
 *
 * Wrapper class of TagDetector class which calls TagDetector::detectTags on a
 * an image stored at a specified load path and stores the output at a specified
 * save path.
 *
 * $Revision: 1.0 $
 * $Date: 2017/12/17 13:33:40 $
 * $Author: dmalyuta $
 *
 * Originator:        Danylo Malyuta, JPL
 ******************************************************************************/

#ifndef ARTAG_NAV_COMMANDER_NODE_H
#define ARTAG_NAV_COMMANDER_NODE_H

#include "apriltag_ros/common_functions.h"
#include <apriltag_ros/AnalyzeSingleImage.h>
#include <std_srvs/Trigger.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace apriltag_ros
{

class SingleImageDetector
{
 private:
  TagDetector tag_detector_;
  ros::ServiceServer single_image_analysis_service_;

  ros::Publisher tag_detections_publisher_;
  
 public:
  SingleImageDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  // The function which provides the single image analysis service
  bool analyzeImage(apriltag_ros::AnalyzeSingleImage::Request& request,
                     apriltag_ros::AnalyzeSingleImage::Response& response);
};

class ArtagNavCommander
{
    public:
     ArtagNavCommander(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    bool triggerCallBack(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
    void testFunc();
    
    bool analyzeImage();
    void waitForNavstack(MoveBaseClient& ac);
    void sendGoal(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal goal);

    private:
     TagDetector tag_detector_;
     AprilTagDetectionArray tag_detections_;
     ros::ServiceServer single_image_analysis_service_;
     ros::Publisher tag_detections_publisher_;
     sensor_msgs::CameraInfo camera_info_;

     //MoveBaseClient move_ac_;
};

} // namespace apriltag_ros

#endif // APRILTAG_ROS_SINGLE_IMAGE_DETECTOR_H
