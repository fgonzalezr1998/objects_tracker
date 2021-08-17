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

#include <string>
#include <map>
#include "objects_tracker/Tracker.h"

namespace objects_tracker
{

Tracker::Tracker()
{
}

bool
Tracker::confObjExists(const std::string & class_name)
{
  for (const auto & elem : tracked_objs_)
  {
    if (elem.first == class_name)
    {
      return true;
    }
  }
  return false;
}

bool
Tracker::poseIsEqual(const StimationType & obj_stimation,
  double x, double y, double z)
{
  // CODE THIS

  return true;
}

void
Tracker::add(const std::string & class_name)
{
  // if class_name already exists, do nothing

  if (confObjExists(class_name))
  {
    return;
  }

  // Else, add it in the map table

  std::map<int, StimationType> value;
  value.clear();

  tracked_objs_[class_name] = value;
}

void
Tracker::update(const std::string & class_name, double x, double y, double z)
{
  // Get the objects whose class is 'class_name'

  std::map<int, StimationType> objs = tracked_objs_[class_name];
  bool found = false;
  for (auto & elem : objs)
  {
    if (poseIsEqual(elem.second, x, y, z))
    {
      // Update control and state data

      found = true;
    }
  }

  if (!found)
  {
    // Compose a new element and intert it in the map table 'objs'
  }

  // Update the tracked objects

  tracked_objs_[class_name] = objs;
}

void
Tracker::printObjects()
{
  for (const auto & elem : tracked_objs_)
  {
    printf("%s\n", elem.first.c_str());
  }
  printf("--------\n");
}

}  // end namespace objects_tracker
