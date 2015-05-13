/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/robot_path_rviz_plugin/robot_path_display.h>
#include <moveit/robot_state/conversions.h>

#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>

#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

namespace moveit_rviz_plugin
{

RobotCnt::RobotCnt(robot_model::RobotModelPtr model, robot_state::RobotStatePtr state, RobotStateVisualizationPtr robot){
  this->model_ = model;
  this->state_ = state;
  this->robot_ = robot;
}
// ******************************************************************************************
// Base class contructor
// ******************************************************************************************
RobotPathDisplay::RobotPathDisplay() :
  Display(),
  update_state_(false)
{
  robot_path_topic_property_ =
    new rviz::RosTopicProperty( "Robot Path Topic", "",
                                ros::message_traits::datatype<nav_msgs::Path>(),
                                "The topic on which the nav_msgs::Path messages are received",
                                this,
                                SLOT( changedRobotPathTopic() ), this );
    
  robot_description_property_ =
    new rviz::StringProperty( "Robot Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded",
                              this,
                              SLOT( changedRobotDescription() ), this );
  
  // Planning scene category -------------------------------------------------------------------------------------------
  robot_alpha_property_ =
    new rviz::FloatProperty( "Robot Alpha", 0.5f, "Specifies the alpha for the robot links",
                             this,
                             SLOT( changedRobotSceneAlpha() ), this );
  robot_alpha_property_->setMin( 0.0 );
  robot_alpha_property_->setMax( 1.0 );

  attached_body_color_property_ = new rviz::ColorProperty( "Attached Body Color", QColor(150, 50, 150), "The color for the attached bodies",
                                                           this,
                                                           SLOT( changedAttachedBodyColor() ), this );
  
  /*enable_link_highlight_ = new rviz::BoolProperty("Show Highlights", true, "Specifies whether link highlighting is enabled",
                                                  this, SLOT( changedEnableLinkHighlight() ), this);
  enable_visual_visible_ = new rviz::BoolProperty("Visual Enabled", true, "Whether to display the visual representation of the robot.",
                                                  this, SLOT( changedEnableVisualVisible() ), this);
  enable_collision_visible_ = new rviz::BoolProperty("Collision Enabled", false, "Whether to display the collision representation of the robot.",
                                                  this, SLOT( changedEnableCollisionVisible() ), this);

  show_all_links_ = new rviz::BoolProperty("Show All Links", true, "Toggle all links visibility on or off.",
                                           this, SLOT( changedAllLinks() ), this);*/
}

// ******************************************************************************************
// Deconstructor
// ******************************************************************************************
RobotPathDisplay::~RobotPathDisplay()
{
}

void RobotPathDisplay::onInitialize()
{
  Display::onInitialize();
}

void RobotPathDisplay::reset()
{  
  robots_.clear();
  rdf_loader_.reset();

  loadRobotModel();
  Display::reset();
}

static bool operator!=(const std_msgs::ColorRGBA& a, const std_msgs::ColorRGBA& b)
{
  return a.r != b.r ||
         a.g != b.g ||
         a.b != b.b ||
         a.a != b.a;
}

void RobotPathDisplay::newRobotPathCallback(nav_msgs::PathConstPtr path)
{
  robots_.clear();
  if(!path){
    setStatus( rviz::StatusProperty::Error, "RobotPath", "Path is null" );
    return;
  }
  ROS_INFO_STREAM("Path length: " << path->poses.size());
  
  if (!rdf_loader_)
    rdf_loader_.reset(new rdf_loader::RDFLoader(robot_description_property_->getStdString()));
  if (rdf_loader_->getURDF())
  {
    const boost::shared_ptr<srdf::Model> &srdf = rdf_loader_->getSRDF() ? rdf_loader_->getSRDF() : boost::shared_ptr<srdf::Model>(new srdf::Model());
    const boost::shared_ptr<urdf::ModelInterface> &mdliface = rdf_loader_->getURDF();
    robot_model::RobotModelPtr rmodel = robot_model::RobotModelPtr(new robot_model::RobotModel(mdliface, srdf));
    
    int i = 0;
    forEach(const geometry_msgs::PoseStamped ps, path->poses){
      i++;
      if(i < path->poses.size())
        continue;
      
      robot_state::RobotStatePtr rstate = robot_state::RobotStatePtr(new robot_state::RobotState(rmodel));
      rstate->setToDefaultValues();
      std::string rootlinkname = rmodel->getRootLinkName();
      Eigen::Affine3d base_tf = rstate->getGlobalLinkTransform(rootlinkname);
      Eigen::Affine3d epose;
      tf::poseMsgToEigen(ps.pose, epose);
      base_tf = base_tf * epose;
      
      rstate->setJointPositions(rmodel->getRootJoint(), base_tf);
      RobotStateVisualizationPtr rrobot = RobotStateVisualizationPtr(new RobotStateVisualization(scene_node_, context_, "Robot Path",this));
      rrobot->load(*rmodel->getURDF());
      
      rstate->update();
      rrobot->update(rstate);
      rrobot->setVisible(true);
      
      QColor color = attached_body_color_property_->getColor();
      std_msgs::ColorRGBA color_msg;
      color_msg.r = color.redF();
      color_msg.g = color.greenF();
      color_msg.b = color.blueF();
      color_msg.a = robot_alpha_property_->getFloat();
      rrobot->setDefaultAttachedObjectColor(color_msg);
      
      robots_.push_back(RobotCntConstPtr(new RobotCnt(rmodel, rstate, rrobot)));
    }
    setStatus( rviz::StatusProperty::Ok, "RobotPath", "Path Visualization Successfull" );  
  }
  else
    setStatus( rviz::StatusProperty::Error, "RobotPath", "Couldn't load Robot Model" );
  
  ROS_INFO_STREAM("robots_ length: " << robots_.size());
//   TODO get current state from PlanningScene and set our state from that.
//   possibly use TF to construct a robot_state::Transforms object to pass in to the conversion functio?  
  update_state_ = true;
}

void RobotPathDisplay::changedAttachedBodyColor()
{
  if (&robots_)
  {
    forEach(RobotCntConstPtr rb, robots_){
      rb->robot_->setAlpha(robot_alpha_property_->getFloat());
      QColor color = attached_body_color_property_->getColor();
      std_msgs::ColorRGBA color_msg;
      color_msg.r = color.redF();
      color_msg.g = color.greenF();
      color_msg.b = color.blueF();
      color_msg.a = robot_alpha_property_->getFloat();
      rb->robot_->setDefaultAttachedObjectColor(color_msg);    
    }
    update_state_ = true;
  }
}

void RobotPathDisplay::changedRobotSceneAlpha()
{
  if (&robots_)
  {
    forEach(RobotCntConstPtr rb, robots_){
      rb->robot_->setAlpha(robot_alpha_property_->getFloat());
      QColor color = attached_body_color_property_->getColor();
      std_msgs::ColorRGBA color_msg;
      color_msg.r = color.redF();
      color_msg.g = color.greenF();
      color_msg.b = color.blueF();
      color_msg.a = robot_alpha_property_->getFloat();
      rb->robot_->setDefaultAttachedObjectColor(color_msg);    
    }
    update_state_ = true;
  }
}

void RobotPathDisplay::changedRobotDescription()
{
  if (isEnabled())
    reset();
}

void RobotPathDisplay::changedRobotPathTopic()
{
  robot_path_subscriber_.shutdown();
  robot_path_subscriber_ = root_nh_.subscribe(robot_path_topic_property_->getStdString(), 10, &RobotPathDisplay::newRobotPathCallback, this);
  robots_.clear();
}

// ******************************************************************************************
// Load
// ******************************************************************************************
void RobotPathDisplay::loadRobotModel()
{
  if (!rdf_loader_)
    rdf_loader_.reset(new rdf_loader::RDFLoader(robot_description_property_->getStdString()));

  if (rdf_loader_->getURDF())
  {
    const boost::shared_ptr<srdf::Model> &srdf = rdf_loader_->getSRDF() ? rdf_loader_->getSRDF() : boost::shared_ptr<srdf::Model>(new srdf::Model());
    kmodel_.reset(new robot_model::RobotModel(rdf_loader_->getURDF(), srdf));
    setStatus( rviz::StatusProperty::Ok, "RobotModel", "Model Loaded Successfully" );
  }
  else
    setStatus( rviz::StatusProperty::Error, "RobotModel", "No  Model Loaded" );

  /*highlights_.clear();*/
}

void RobotPathDisplay::onEnable()
{
  Display::onEnable();
  loadRobotModel();
  calculateOffsetPosition();
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void RobotPathDisplay::onDisable()
{
  robots_.clear();
  Display::onDisable();
}

void RobotPathDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
  if (&robots_ && update_state_)
  {
    forEach(RobotCntConstPtr rb, robots_){
      rb->state_->update();
      rb->robot_->update(rb->state_);      
    }
  }
  update_state_ = false;
}

// ******************************************************************************************
// Calculate Offset Position
// ******************************************************************************************
void RobotPathDisplay::calculateOffsetPosition()
{
  if (!kmodel_)
    return;

  ros::Time stamp;
  std::string err_string;
  if (context_->getTFClient()->getLatestCommonTime(fixed_frame_.toStdString(), kmodel_->getModelFrame(), stamp, &err_string) != tf::NO_ERROR)
    return;

  tf::Stamped<tf::Pose> pose(tf::Pose::getIdentity(), stamp, kmodel_->getModelFrame());

  if (context_->getTFClient()->canTransform(fixed_frame_.toStdString(), kmodel_->getModelFrame(), stamp))
  {
    try
    {
      context_->getTFClient()->transformPose(fixed_frame_.toStdString(), pose, pose);
    }
    catch (tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming from frame '%s' to frame '%s'", pose.frame_id_.c_str(), fixed_frame_.toStdString().c_str() );
    }
  }

  Ogre::Vector3 position(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());
  const tf::Quaternion &q = pose.getRotation();
  Ogre::Quaternion orientation( q.getW(), q.getX(), q.getY(), q.getZ() );
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void RobotPathDisplay::fixedFrameChanged()
{
  Display::fixedFrameChanged();
  calculateOffsetPosition();
}


} // namespace moveit_rviz_plugin
