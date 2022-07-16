/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2021
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Fernando González fergonzaramos@yahoo.es  */

#ifndef OBJECTS_TRACKER_OBJECTSTRACKER_H
#define OBJECTS_TRACKER_OBJECTSTRACKER_H

#include <ros/ros.h>

#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <gb_visual_detection_3d_msgs/BoundingBox3d.h>
#include <string>
#include <vector>
#include <memory>
#include <map>

#include "objects_tracker/Tracker.h"

namespace objects_tracker
{
class ObjectsTracker
{
public:
  ObjectsTracker();

  void update();

private:
  void configureParams();
  void bboxes3dCallback(
    const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr & msg);

  ros::NodeHandle nh_;
  ros::Subscriber bboxes3d_sub_;
  std::map<std::string, ros::Publisher> pubs_;  // One publisher per object class
  std::string bboxes3d_topic_;
  std::vector<std::string> interested_classes_;
  std::vector<gb_visual_detection_3d_msgs::BoundingBox3d> bboxes_received_;
  std::shared_ptr<Tracker> tracker_;
};
}   // end namespace objects_tracker
#endif  // OBJECTS_TRACKER_OBJECTSTRACKER_H
