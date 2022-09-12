/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2022
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

#include <string>
#include <vector>
#include "bbox3d_tracker/Bbox3dTracker.h"

using gb_visual_detection_3d_msgs::BoundingBoxes3d;
using gb_visual_detection_3d_msgs::BoundingBox3d;

namespace bbox3d_tracker
{
Bbox3dTracker::Bbox3dTracker():
  nh_("~"), target_class_(""), last_detection_ts_(0.0)
{
  tgts_list_.clear();
  ROS_INFO("[%s] STARTED\n", ros::this_node::getName().c_str());
}

void Bbox3dTracker::update()
{
  ROS_INFO("UPDATE");
}

/*
  * @brief Callback function to handle the bounding boxes reception
  *
  * @param msg: BoundingBoxes3d. ROS message received
  */
void Bbox3dTracker::bboxes3dCallback(const BoundingBoxes3d::ConstPtr & msg)
{
  // Clear detected targets
  tgts_list_.clear();
  if (target_class_ == "")
    return;

  working_frame_ = msg->header.frame_id;
  std::vector<BoundingBox3d> bboxes = msg->bounding_boxes;
  if (targetsDetected(bboxes))
  {
    ROS_INFO("=== Targets Detected ===");

    // For each target, update the stimation
    for (auto bbox : bboxes)
    {
      if (isTarget(bbox.Class))
      {
        ROS_INFO("=== Update Stimation EKF ===");
      }
    }
  }
  else
  {
    ROS_INFO("+++ No Targets +++");
  }
}

bool Bbox3dTracker::targetsDetected(
  const std::vector<BoundingBox3d> & targets)
{
  bool found = false;
  for (int i = 0; i < targets.size() && !found; i++)
  {
    found = isTarget(targets.at(i).Class);
  }

  return found;
}

bool Bbox3dTracker::isTarget(const std::string & bbox_class)
{
  return bbox_class == target_class_;
}

/*
  * @brief Set up the interested class for the tracking
  *
  * @param class_name: string. Name of the interested class
  */
void Bbox3dTracker::setTarget(const std::string & class_name, const std::string & topic,
  const std::string & fixed_frame)
{
  target_class_ = class_name;
  fixed_frame_ = fixed_frame;

  bboxes3d_sub_ = nh_.subscribe(topic, 1,
    &Bbox3dTracker::bboxes3dCallback, this);
}
}  // namespace bbox3d_tracker
