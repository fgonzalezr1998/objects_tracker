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

#include "objects_tracker/ObjectsTracker.h"

namespace objects_tracker
{

ObjectsTracker::ObjectsTracker()
: nh_("~"), tracker_(new Tracker())
{
  configureParams();
  bboxes3d_sub_ = nh_.subscribe(
    bboxes3d_topic_, 1, &ObjectsTracker::bboxes3dCallback, this);
}

void
ObjectsTracker::update()
{
  double x, y, z;
  for (const auto & bbox : bboxes_received_)
  {
    ROS_INFO("Do something with [%s]\n", bbox.Class.c_str());

    // Get the center

    x = (bbox.xmin + bbox.xmax) / 2.0;
    y = (bbox.ymin + bbox.ymax) / 2.0;
    z = (bbox.zmin + bbox.zmax) / 2.0;

    tracker_->update(bbox.Class, x, y, z);
  }
}

void
ObjectsTracker::configureParams()
{
  bboxes3d_topic_ = "/darknet_ros_3d/bounding_boxes";

  nh_.param("bboxes3d_topic", bboxes3d_topic_, bboxes3d_topic_);
  nh_.param("interested_classes", interested_classes_, interested_classes_);

  for (auto obj : interested_classes_)
  {
    tracker_->add(obj);
  }
}

void
ObjectsTracker::bboxes3dCallback(
  const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr & msg)
{
  // Clear vector

  bboxes_received_.clear();

  // Set the new vector

  for (auto bbox : msg->bounding_boxes)
  {
    if (std::find(interested_classes_.begin(), interested_classes_.end(),
      bbox.Class) != interested_classes_.end())
    {
      bboxes_received_.push_back(bbox);
    }
  }
}

}  // namespace objects_tracker
