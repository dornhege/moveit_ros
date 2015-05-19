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

/* Author: Philipp Jankov*/
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
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH
#define PI 3.14159265358979323846

namespace moveit_rviz_plugin
{

RobotCnt::RobotCnt(robot_state::RobotStatePtr state, RobotStateVisualizationPtr robot){
  state_ = state;
  robot_ = robot;
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
                                "The topic on which the nav_msgs::Path messages are received", this,
                                SLOT( changedRobotPathTopic() ), this );
  robot_description_property_ =
    new rviz::StringProperty( "Robot Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded", this,
                              SLOT( changedRobotDescription() ), this );
  
  // Planning scene category -------------------------------------------------------------------------------------------
  robot_alpha_property_ =
    new rviz::FloatProperty( "Robot Max Alpha", 0.8, "Specifies the alpha for the last drawn robot. First starts with 0.1", this,
                             SLOT( redrawPath() ), this );
  robot_alpha_property_->setMin( 0.1 );
  robot_alpha_property_->setMax( 1.0 );
    
  robot_deltatheta_property_ =
    new rviz::FloatProperty( "Theta Draw Threshold", 15.0, "Specifies the angle in degree that triggers a draw",  this,
                             SLOT( redrawPath() ), this );
  robot_deltatheta_property_->setMin( 0.0 );
  robot_deltatheta_property_->setMax( 360.0 );
    
  robot_deltadist_property_ =
    new rviz::FloatProperty( "Distance Draw Threshold", 1.0, "Specifies the distance in degree that triggers a draw", this,
                             SLOT( redrawPath() ), this );
  robot_deltadist_property_->setMin( 0.025 );
    
  
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
  last_known_path_ = nav_msgs::PathConstPtr();
  rdf_loader_.reset();
  loadRobotModel();
  Display::reset();
}

void RobotPathDisplay::newRobotPathCallback(nav_msgs::PathConstPtr path)
{
  if(!kmodel_)
      return;

  last_known_path_ = path;
  robots_.clear();
  if(!path || path->poses.size() < 1){
    setStatus( rviz::StatusProperty::Warn, "RobotPath", "Path is null or empty" );
    return;
  }
  ROS_DEBUG_STREAM("Path length: " << path->poses.size());

  
  int i = 0;
  float ralpha = 0.1;  
  const geometry_msgs::PoseStamped* lastDrawnPoseStamped = NULL;
  forEach(const geometry_msgs::PoseStamped ps, path->poses){
    i++;
    if(lastDrawnPoseStamped != NULL){
      
      tf::Quaternion tfquatp;
      tf::quaternionMsgToTF(ps.pose.orientation, tfquatp);
      double pyaw = tf::getYaw(tfquatp);      
      tf::Quaternion tfquatlp;
      tf::quaternionMsgToTF(lastDrawnPoseStamped->pose.orientation, tfquatlp);      
      double lpyaw = tf::getYaw(tfquatlp);

      float deltatheta = fabs(pyaw - lpyaw) * 180 / PI;
      float deltadist = hypot(ps.pose.position.x - lastDrawnPoseStamped->pose.position.x,
                              ps.pose.position.y - lastDrawnPoseStamped->pose.position.y);
      if(i != path->poses.size() 
          && deltatheta < robot_deltatheta_property_->getFloat() 
          && deltadist < robot_deltadist_property_->getFloat())
        continue;
      
      ralpha = robot_alpha_property_->getFloat() - 0.1 * i / path->poses.size();
      ROS_DEBUG_STREAM("##################################" << i << "/" << path->poses.size());
      ROS_DEBUG_STREAM("Hue: " << ((float)i / path->poses.size()*300));
      ROS_DEBUG_STREAM("DDistance: " << deltadist);
      ROS_DEBUG_STREAM("Alpha: " << ralpha);
      ROS_DEBUG_STREAM("DTheta(Rads): " << deltatheta / 180 * PI << " DTheta(Deg): " << deltatheta);
    }    
//     if(i == path->poses.size() && !robots_.empty()){
//       robots_.pop_back();
//     }
    
    // Setup State
    robot_state::RobotStatePtr rstate = robot_state::RobotStatePtr(new robot_state::RobotState(kmodel_));
    rstate->setToDefaultValues();
    Eigen::Affine3d base_tf = rstate->getGlobalLinkTransform(kmodel_->getRootLinkName());
    Eigen::Affine3d epose;
    tf::poseMsgToEigen(ps.pose, epose);
    base_tf = base_tf * epose;    
    rstate->setJointPositions(kmodel_->getRootJoint(), base_tf);
    
    // Setup Robot
    RobotStateVisualizationPtr rrobot = RobotStateVisualizationPtr(new RobotStateVisualization(scene_node_, context_, "Robot Path",this));
    rrobot->load(*kmodel_->getURDF());
    const QColor color = QColor::fromHsv ((float)i / path->poses.size() * 300, 255, 255, ralpha);
    rrobot->setAlpha(ralpha);
    forEach(std::string ln, kmodel_->getLinkModelNames()){
        rviz::RobotLink *link = rrobot->getRobot().getLink(ln); 
        // Check if link exists
        if (link)
          link->setColor(color.redF(), color.greenF(), color.blueF());
    }
    
    robots_.push_back(RobotCntConstPtr(new RobotCnt(rstate, rrobot)));
    if(lastDrawnPoseStamped != NULL)
      ROS_DEBUG_STREAM("##################################" << i << "/" << path->poses.size());
    lastDrawnPoseStamped = &path->poses[i - 1];
  }
  setStatus( rviz::StatusProperty::Ok, "RobotPath", "Path Visualization Successfull" );
  update_state_ = true;
}

void RobotPathDisplay::redrawPath()
{
  newRobotPathCallback(last_known_path_);
  update_state_ = true;
}

void RobotPathDisplay::changedRobotDescription()
{
  if (isEnabled())
    reset();
}

void RobotPathDisplay::changedRobotPathTopic()
{  
  robots_.clear();
  last_known_path_ = nav_msgs::PathConstPtr();
  robot_path_subscriber_.shutdown();
  robot_path_subscriber_ = root_nh_.subscribe(robot_path_topic_property_->getStdString(), 10, &RobotPathDisplay::newRobotPathCallback, this);
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
  robot_path_subscriber_ = root_nh_.subscribe(robot_path_topic_property_->getStdString(), 10, &RobotPathDisplay::newRobotPathCallback, this);
  robots_.clear();
  last_known_path_ = nav_msgs::PathConstPtr();
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void RobotPathDisplay::onDisable()
{
  robots_.clear();
  last_known_path_ = nav_msgs::PathConstPtr();
  robot_path_subscriber_.shutdown();
  Display::onDisable();
}

void RobotPathDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
  if (update_state_)
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
