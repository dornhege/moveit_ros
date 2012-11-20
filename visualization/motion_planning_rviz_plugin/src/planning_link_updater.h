/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Ioan Sucan */

#ifndef MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_PLANNING_LINK_UPDATER_
#define MOVEIT_MOTION_PLANNING_RVIZ_PLUGIN_PLANNING_LINK_UPDATER_

#include <rviz/robot/link_updater.h>
#include <moveit/kinematic_state/kinematic_state.h>

namespace motion_planning_rviz_plugin
{

/** \brief Update the links of an rviz::Robot using a kinematic_state::KinematicState */
class PlanningLinkUpdater : public rviz::LinkUpdater
{
public:
  
  PlanningLinkUpdater(const kinematic_state::KinematicStateConstPtr &state)
    : kinematic_state_(state)
  {
  }
  
  virtual bool getLinkTransforms(const std::string& link_name, Ogre::Vector3& visual_position, Ogre::Quaternion& visual_orientation,
                                 Ogre::Vector3& collision_position, Ogre::Quaternion& collision_orientation) const;
  
private:
  kinematic_state::KinematicStateConstPtr kinematic_state_;
};

}

#endif