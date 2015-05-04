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

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

namespace moveit_rviz_plugin
{

// ******************************************************************************************
// Base class contructor
// ******************************************************************************************
RobotPathDisplay::RobotPathDisplay() :
  Display(),
  update_state_(false)
{
  robot_description_property_ =
    new rviz::StringProperty( "Robot Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded",
                              this,
                              SLOT( changedRobotDescription() ), this );

  robot_path_topic_property_ =
    new rviz::RosTopicProperty( "Robot Path Topic", "display_robot_path",
                                ros::message_traits::datatype<nav_msgs::Path<std::vector<geometry_msgs::PoseStamped>>>(),
                                "The topic on which the nav_msgs::Path messages are received",
                                this,
                                SLOT( changedRobotPathTopic() ), this );

  // Planning scene category -------------------------------------------------------------------------------------------
  /*root_link_name_property_ =
    new rviz::StringProperty( "Robot Root Link", "", "Shows the name of the root link for the robot model",
                              this,
                              SLOT( changedRootLinkName() ), this );
  root_link_name_property_->setReadOnly(true);*/

  robot_alpha_property_ =
    new rviz::FloatProperty( "Robot Alpha", 1.0f, "Specifies the alpha for the robot links",
                             this,
                             SLOT( changedRobotSceneAlpha() ), this );
  robot_alpha_property_->setMin( 0.0 );
  robot_alpha_property_->setMax( 1.0 );

  /*attached_body_color_property_ = new rviz::ColorProperty( "Attached Body Color", QColor(150, 50, 150), "The color for the attached bodies",
                                                           this,
                                                           SLOT( changedAttachedBodyColor() ), this );

  enable_link_highlight_ = new rviz::BoolProperty("Show Highlights", true, "Specifies whether link highlighting is enabled",
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
  robot_.reset(new RobotStateVisualization(scene_node_, context_, "Robot Path", this));
  changedEnableVisualVisible();
  changedEnableCollisionVisible();
  robot_->setVisible(false);
}

void RobotPathDisplay::reset()
{
  robot_->clear();
  rdf_loader_.reset();

  loadRobotModel();
  Display::reset();

  changedEnableVisualVisible();
  changedEnableCollisionVisible();
  robot_->setVisible(true);
}

void RobotPathDisplay::changedAllLinks()
{
  Property *links_prop = subProp("Links");
  QVariant value(show_all_links_->getBool());

  for (int i=0 ; i<links_prop->numChildren() ; ++i)
  {
    Property *link_prop = links_prop->childAt(i);
    link_prop->setValue(value);
  }
}

void RobotPathDisplay::setHighlight(const std::string& link_name, const std_msgs::ColorRGBA& color)
{
  rviz::RobotLink *link = robot_->getRobot().getLink(link_name);
  if (link)
  {
    link->setColor(color.r, color.g, color.b);
    link->setRobotAlpha(color.a * robot_alpha_property_->getFloat());
  }
}

void RobotPathDisplay::unsetHighlight(const std::string& link_name)
{
  rviz::RobotLink *link = robot_->getRobot().getLink(link_name);
  if (link)
  {
    link->unsetColor();
    link->setRobotAlpha(robot_alpha_property_->getFloat());
  }
}

void RobotPathDisplay::changedEnableLinkHighlight()
{
  if (enable_link_highlight_->getBool())
  {
    for (std::map<std::string, std_msgs::ColorRGBA>::iterator it = highlights_.begin() ;
         it != highlights_.end() ;
         ++it)
    {
      setHighlight(it->first, it->second);
    }
  }
  else
  {
    for (std::map<std::string, std_msgs::ColorRGBA>::iterator it = highlights_.begin() ;
         it != highlights_.end() ;
         ++it)
    {
      unsetHighlight(it->first);
    }
  }
}

void RobotPathDisplay::changedEnableVisualVisible()
{
  robot_->setVisualVisible(enable_visual_visible_->getBool());
}

void RobotPathDisplay::changedEnableCollisionVisible()
{
  robot_->setCollisionVisible(enable_collision_visible_->getBool());
}

static bool operator!=(const std_msgs::ColorRGBA& a, const std_msgs::ColorRGBA& b)
{
  return a.r != b.r ||
         a.g != b.g ||
         a.b != b.b ||
         a.a != b.a;
}

void RobotPathDisplay::setRobotHighlights(const moveit_msgs::DisplayRobotState::_highlight_links_type& highlight_links)
{
  /*if (highlight_links.empty() && highlights_.empty())
    return;

  std::map<std::string, std_msgs::ColorRGBA> highlights;
  for (moveit_msgs::DisplayRobotState::_highlight_links_type::const_iterator it = highlight_links.begin() ;
       it != highlight_links.end() ;
       ++it)
  {
    highlights[it->id] = it->color;
  }

  if (enable_link_highlight_->getBool())
  {
    std::map<std::string, std_msgs::ColorRGBA>::iterator ho = highlights_.begin();
    std::map<std::string, std_msgs::ColorRGBA>::iterator hn = highlights.begin();
    while (ho != highlights_.end() || hn != highlights.end())
    {
      if (ho == highlights_.end())
      {
        setHighlight(hn->first, hn->second);
        ++hn;
      }
      else if (hn == highlights.end())
      {
        unsetHighlight(ho->first);
        ++ho;
      }
      else if (hn->first < ho->first)
      {
        setHighlight(hn->first, hn->second);
        ++hn;
      }
      else if (hn->first > ho->first)
      {
        unsetHighlight(ho->first);
        ++ho;
      }
      else if (hn->second != ho->second)
      {
        setHighlight(hn->first, hn->second);
        ++ho;
        ++hn;
      }
      else
      {
        ++ho;
        ++hn;
      }
    }
  }

  swap(highlights, highlights_);*/
}

void RobotPathDisplay::changedAttachedBodyColor()
{
  if (robot_)
  {
    QColor color = attached_body_color_property_->getColor();
    std_msgs::ColorRGBA color_msg;
    color_msg.r = color.redF();
    color_msg.g = color.greenF();
    color_msg.b = color.blueF();
    color_msg.a = robot_alpha_property_->getFloat();
    robot_->setDefaultAttachedObjectColor(color_msg);
    update_state_ = true;
  }
}

void RobotPathDisplay::changedRobotDescription()
{
  if (isEnabled())
    reset();
}

void RobotPathDisplay::changedRootLinkName()
{
}

void RobotPathDisplay::changedRobotSceneAlpha()
{
  if (robot_)
  {
    robot_->setAlpha(robot_alpha_property_->getFloat());
    QColor color = attached_body_color_property_->getColor();
    std_msgs::ColorRGBA color_msg;
    color_msg.r = color.redF();
    color_msg.g = color.greenF();
    color_msg.b = color.blueF();
    color_msg.a = robot_alpha_property_->getFloat();
    robot_->setDefaultAttachedObjectColor(color_msg);
    update_state_ = true;
  }
}

void RobotPathDisplay::changedRobotPathTopic()
{
  robot_path_subscriber_.shutdown();
  robot_path_subscriber_ = root_nh_.subscribe(robot_path_topic_property_->getStdString(), 10, &RobotPathDisplay::newRobotPathCallback, this);
  robot_->clear();
  loadRobotModel();
}
//
void RobotPathDisplay::newRobotPathCallback(const nav_msgs::Path<std::vector<geometry_msgs::PoseStamped>>& path_poses)
{
  if (!kmodel_)
    return;
  if (!kstate_)
    kstate_.reset(new robot_state::RobotState(kmodel_));
  // possibly use TF to construct a robot_state::Transforms object to pass in to the conversion functio?
  
  // TODO: NEED POSE TO STATE!
  /*robot_state::robotStateMsgToRobotState(state_msg->state, *kstate_);
  setRobotHighlights(state_msg->highlight_links);*/
  update_state_ = true;
}

void RobotPathDisplay::setLinkColor(const std::string& link_name, const QColor &color)
{
  setLinkColor(&robot_->getRobot(), link_name, color );
}

void RobotPathDisplay::unsetLinkColor(const std::string& link_name)
{
  unsetLinkColor(&robot_->getRobot(), link_name);
}

void RobotPathDisplay::setLinkColor(rviz::Robot* robot,  const std::string& link_name, const QColor &color )
{
  rviz::RobotLink *link = robot->getLink(link_name);

  // Check if link exists
  if (link)
    link->setColor( color.redF(), color.greenF(), color.blueF() );
}

void RobotPathDisplay::unsetLinkColor(rviz::Robot* robot, const std::string& link_name )
{
  rviz::RobotLink *link = robot->getLink(link_name);

  // Check if link exists
  if (link)
    link->unsetColor();
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
    robot_->load(*kmodel_->getURDF());
    kstate_.reset(new robot_state::RobotState(kmodel_));
    kstate_->setToDefaultValues();
    bool oldState = root_link_name_property_->blockSignals(true);
    root_link_name_property_->setStdString(getRobotModel()->getRootLinkName());
    root_link_name_property_->blockSignals(oldState);
    update_state_ = true;
    setStatus( rviz::StatusProperty::Ok, "RobotPath", "Planning Model Loaded Successfully" );
  }
  else
    setStatus( rviz::StatusProperty::Error, "RobotPath", "No Planning Model Loaded" );

  highlights_.clear();
}

void RobotPathDisplay::onEnable()
{
  Display::onEnable();
  loadRobotModel();
  if (robot_)
  {
    changedEnableVisualVisible();
    changedEnableCollisionVisible();
    robot_->setVisible(true);
  }
  calculateOffsetPosition();
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void RobotPathDisplay::onDisable()
{
  if (robot_)
    robot_->setVisible(false);
  Display::onDisable();
}

void RobotPathDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
  if (robot_ && update_state_)
  {
    update_state_ = false;
    kstate_->update();
    robot_->update(kstate_);
  }
}

// ******************************************************************************************
// Calculate Offset Position
// ******************************************************************************************
void RobotPathDisplay::calculateOffsetPosition()
{
  if (!getRobotModel())
    return;

  ros::Time stamp;
  std::string err_string;
  if (context_->getTFClient()->getLatestCommonTime(fixed_frame_.toStdString(), getRobotModel()->getModelFrame(), stamp, &err_string) != tf::NO_ERROR)
    return;

  tf::Stamped<tf::Pose> pose(tf::Pose::getIdentity(), stamp, getRobotModel()->getModelFrame());

  if (context_->getTFClient()->canTransform(fixed_frame_.toStdString(), getRobotModel()->getModelFrame(), stamp))
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
