/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
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
 *   * Neither the name of JSK, The University of Tokyo nor the names of its
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

/* Author: Dave Coleman, Masaki Murooka */

#include <moveit/srv_kinematics_plugin/srv_kinematics_plugin.h>
#include <class_loader/class_loader.h>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <moveit/robot_state/conversions.h>

#include <moveit/rdf_loader/rdf_loader.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

//register SRVKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(srv_kinematics_plugin::SrvKinematicsPlugin, kinematics::KinematicsBase)

namespace srv_kinematics_plugin
{

SrvKinematicsPlugin::SrvKinematicsPlugin():active_(false)
{
  ROS_ERROR_STREAM_NAMED("temp","testing SrvKinematicsPlugin loading");
}

bool SrvKinematicsPlugin::isRedundantJoint(unsigned int index) const
{
  for (std::size_t j=0; j < redundant_joint_indices_.size(); ++j)
    if (redundant_joint_indices_[j] == index)
      return true;
  return false;
}

bool SrvKinematicsPlugin::initialize(const std::string &robot_description,
  const std::string& group_name,
  const std::string& base_frame,
  const std::vector<std::string>& tip_frames,
  double search_discretization)
{
  ROS_ERROR_STREAM_NAMED("temp","testing SrvKinematicsPlugin initializing");

  setValues(robot_description, group_name, base_frame, tip_frames, search_discretization);

  ros::NodeHandle private_handle("~");
  rdf_loader::RDFLoader rdf_loader(robot_description_);
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader.getURDF();

  if (!urdf_model || !srdf)
  {
    ROS_ERROR_NAMED("srv","URDF and SRDF must be loaded for SRV kinematics solver to work."); // TODO: is this true?
    return false;
  }

  robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
    return false;

  if(!joint_model_group_->isSingleDOFJoints())
  {
    ROS_ERROR_NAMED("srv","Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
    //return false;
  }

  // DEBUG
  std::cout << std::endl << "Joint Model Variable Names: ------------------------------------------- " << std::endl;
  const std::vector<std::string> jm_names = joint_model_group_->getVariableNames();
  std::copy(jm_names.begin(), jm_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
  std::cout << std::endl;

  //dimension_ = joint_model_group_->getActiveJointModels().size() + joint_model_group_->getMimicJointModels().size();
  dimension_ = joint_model_group_->getVariableCount(); // TODO: exclude mimic joints
  ROS_INFO_STREAM_NAMED("srv","Dimension planning group '" << group_name << "': " << dimension_
    << ". Active Joints Models: " << joint_model_group_->getActiveJointModels().size()
    << ". Mimic Joint Models: " << joint_model_group_->getMimicJointModels().size());

  // Copy joint names
  std::cout << std::endl << "Saved Joint Model Names: ------------------------------------------- " << std::endl;
  for (std::size_t i=0; i < joint_model_group_->getJointModels().size(); ++i)
  {
    ik_group_info_.joint_names.push_back(joint_model_group_->getJointModelNames()[i]);
    // DEBUG remove
    std::cout << "  " << joint_model_group_->getJointModelNames()[i] << std::endl;
  }

  //if(joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::REVOLUTE ||
  //   joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::PRISMATIC)

  // Make sure all the tip links are in the link_names vector
  for (std::size_t i = 0; i < tip_frames_.size(); ++i)
  {
    if(!joint_model_group_->hasLinkModel(tip_frames_[i]))
    {
      ROS_ERROR_NAMED("srv","Could not find tip name in joint group '%s'", group_name.c_str());
      return false;
    }
    ik_group_info_.link_names.push_back(tip_frames_[i]);
  }

  // Choose what ROS service to send IK requests to
  ROS_DEBUG_STREAM_NAMED("srv","Looking for ROS service name on rosparm server at location: " <<
    private_handle.getNamespace() << "/" << group_name_ << "/kinematics_solver_service_name");
  std::string ik_service_name;
  private_handle.param(group_name_ + "/kinematics_solver_service_name", ik_service_name, std::string("solve_ik"));

  // Setup the joint state groups that we need
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues(); // TODO: this makes assumptions we shouldn't be making

  // Create the ROS service client
  // TODO: use private handle
  ros::NodeHandle nonprivate_handle("");
  ik_service_client_ = boost::make_shared<ros::ServiceClient>(nonprivate_handle.serviceClient
                       <moveit_msgs::GetPositionIK>(ik_service_name));
  ROS_INFO_STREAM_NAMED("srv","Service client started with ROS service name: " << ik_service_client_->getService());
  // TODO: check if service is up?

  active_ = true;
  ROS_DEBUG_NAMED("srv","ROS service-based kinematics solver initialized");
  return true;
}

bool SrvKinematicsPlugin::setRedundantJoints(const std::vector<unsigned int> &redundant_joints)
{
  if(num_possible_redundant_joints_ < 0)
  {
    ROS_ERROR_NAMED("srv","This group cannot have redundant joints");
    return false;
  }
  if(redundant_joints.size() > num_possible_redundant_joints_)
  {
    ROS_ERROR_NAMED("srv","This group can only have %d redundant joints", num_possible_redundant_joints_);
    return false;
  }


  return true;
}

int SrvKinematicsPlugin::getJointIndex(const std::string &name) const
{
  for (unsigned int i=0; i < ik_group_info_.joint_names.size(); i++) {
    if (ik_group_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

bool SrvKinematicsPlugin::timedOut(const ros::WallTime &start_time, double duration) const
{
  return ((ros::WallTime::now()-start_time).toSec() >= duration);
}

bool SrvKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
  const std::vector<double> &ik_seed_state,
  std::vector<double> &solution,
  moveit_msgs::MoveItErrorCodes &error_code,
  const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
    ik_seed_state,
    default_timeout_,
    solution,
    solution_callback,
    error_code,
    consistency_limits,
    options);
}

bool SrvKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
  const std::vector<double> &ik_seed_state,
  double timeout,
  std::vector<double> &solution,
  moveit_msgs::MoveItErrorCodes &error_code,
  const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
    ik_seed_state,
    timeout,
    solution,
    solution_callback,
    error_code,
    consistency_limits,
    options);
}

bool SrvKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
  const std::vector<double> &ik_seed_state,
  double timeout,
  const std::vector<double> &consistency_limits,
  std::vector<double> &solution,
  moveit_msgs::MoveItErrorCodes &error_code,
  const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose,
    ik_seed_state,
    timeout,
    solution,
    solution_callback,
    error_code,
    consistency_limits,
    options);
}

bool SrvKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
  const std::vector<double> &ik_seed_state,
  double timeout,
  std::vector<double> &solution,
  const IKCallbackFn &solution_callback,
  moveit_msgs::MoveItErrorCodes &error_code,
  const kinematics::KinematicsQueryOptions &options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose,
    ik_seed_state,
    timeout,
    solution,
    solution_callback,
    error_code,
    consistency_limits,
    options);
}

bool SrvKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
  const std::vector<double> &ik_seed_state,
  double timeout,
  const std::vector<double> &consistency_limits,
  std::vector<double> &solution,
  const IKCallbackFn &solution_callback,
  moveit_msgs::MoveItErrorCodes &error_code,
  const kinematics::KinematicsQueryOptions &options) const
{
  return searchPositionIK(ik_pose,
    ik_seed_state,
    timeout,
    solution,
    solution_callback,
    error_code,
    consistency_limits,
    options);
}

bool SrvKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
  const std::vector<double> &ik_seed_state,
  double timeout,
  std::vector<double> &solution,
  const IKCallbackFn &solution_callback,
  moveit_msgs::MoveItErrorCodes &error_code,
  const std::vector<double> &consistency_limits,
  const kinematics::KinematicsQueryOptions &options) const
{
  // Convert single pose into a vector of one pose
  std::vector<geometry_msgs::Pose> ik_poses;
  ik_poses.push_back(ik_pose);

  return searchPositionIK(ik_poses,
    ik_seed_state,
    timeout,
    consistency_limits,
    solution,
    solution_callback,
    error_code,
    options);
}

// RequestIK from remote node
bool SrvKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
  const std::vector<double> &ik_seed_state,
  double timeout,
  const std::vector<double> &consistency_limits,
  std::vector<double> &solution,
  const IKCallbackFn &solution_callback,
  moveit_msgs::MoveItErrorCodes &error_code,
  const kinematics::KinematicsQueryOptions &options) const
{

  if(!active_)
  {
    ROS_ERROR_NAMED("srv","kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if(ik_seed_state.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED("srv","Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }
  // Debug: what is the order of the ik seed state?
  ROS_ERROR_STREAM_NAMED("temp","ik seed state for group " << group_name_ << ":");
  std::copy(ik_seed_state.begin(), ik_seed_state.end(), std::ostream_iterator<double>(std::cout, "\n"));

  // Create the service message
  moveit_msgs::GetPositionIK ik_srv;
  //  ik_srv.request.ik_request.group_name = ":fullbody-inverse-kinematics";
  //ik_srv.request.ik_request.group_name = ":fullbody-inverse-kinematics (:fix-limbs (:rleg :lleg))"; // TODO fix this: getGroupName();
  ik_srv.request.ik_request.group_name = ":fullbody-inverse-kinematics (:fix-limbs (:rleg :lleg) :debug-view nil)";
  //ik_srv.request.ik_request.group_name = ":fullbody-inverse-kinematics (:fix-limbs (:rleg :lleg) :rotation-axis nil)";

  // Copy in our seed state into our virtual robot state
  robot_state_->setVariablePositions(getVariableNames(), ik_seed_state); // TODO: make more efficient  @isucan?
  // Convert our robot state into a msg we can send
  moveit::core::robotStateToRobotStateMsg(*robot_state_, ik_srv.request.ik_request.robot_state);

  // Check that we have the same number of poses as tips
  if (tip_frames_.size() != ik_poses.size())
  {
    ROS_ERROR_STREAM_NAMED("srv","Mismatched number of pose requests (" << ik_poses.size()
      << ") to tip frames (" << tip_frames_.size() << ") in searchPositionIK");
    return false;
  }

  // Create a stamped pose
  geometry_msgs::PoseStamped ik_pose_st;

  // Load the poses into the request in difference places depending if there is more than one or not
  if (tip_frames_.size() > 1)
  {
    // Load into vector of poses
    for (std::size_t i = 0; i < tip_frames_.size(); ++i)
    {
      ik_pose_st.pose = ik_poses[i];
      ik_pose_st.header.frame_id = base_frame_;
      ik_srv.request.ik_request.pose_stamped_vector.push_back(ik_pose_st);
      ik_srv.request.ik_request.ik_link_names.push_back(tip_frames_[i]);
    }
  }
  else
  {
    // Load into single pose value
    ik_pose_st.pose = ik_poses[0];
    ik_pose_st.header.frame_id = base_frame_;
    ik_srv.request.ik_request.pose_stamped = ik_pose_st;
    ik_srv.request.ik_request.ik_link_name = getTipFrame();
    ROS_ERROR_STREAM_NAMED("temp","tip frame is " << getTipFrame());
  }

  ROS_INFO_STREAM("Request is: \n" << ik_srv.request.ik_request);

  ROS_INFO_STREAM("Calling service: " << ik_service_client_->getService() );
  if (ik_service_client_->call(ik_srv))
  {
    //result_angle = ik_srv.response.solution.joint_state.position;
    ROS_INFO_STREAM("Service response recieved, message: \n" << ik_srv.response.solution);
  }
  else
  {
    ROS_ERROR_STREAM("SERVICE FAILED");
  }

  error_code.val = ik_srv.response.error_code.val;

  if(error_code.val != 1)
  {
    ROS_DEBUG_NAMED("srv","An IK that satisifes the constraints and is collision free could not be found.");
    ROS_ERROR_STREAM_NAMED("srv","Exited with error code: " << error_code.val);
    return false;
  }

  ROS_INFO_STREAM_NAMED("srv","SUCCESS.");

  // Get a pointer to the solution
  sensor_msgs::JointState *joint_state = &ik_srv.response.solution.joint_state;

  ROS_ERROR_STREAM_NAMED("temp","TODO: remove these conversions!");
  // Extract rpy
  double angle_r = joint_state->position[joint_state->position.size() - 3];
  double angle_p = joint_state->position[joint_state->position.size() - 2];
  double angle_y = joint_state->position[joint_state->position.size() - 1];

  // Convert recieved rpy to quaternions
  Eigen::Quaternion<float> q;
  Eigen::AngleAxis<float> aaZ(angle_r, Eigen::Vector3f::UnitZ());
  Eigen::AngleAxis<float> aaY(angle_p, Eigen::Vector3f::UnitY());
  Eigen::AngleAxis<float> aaX(angle_y, Eigen::Vector3f::UnitX());
  q = aaZ * aaY * aaX;

  // Erase the rotation matrix because they are in the wrong format
  joint_state->name.erase(joint_state->name.end()-3, joint_state->name.end());
  joint_state->position.erase(joint_state->position.end()-3, joint_state->position.end());

  // Insert new values
  joint_state->name.push_back("virtual_joint/rot_x");
  joint_state->name.push_back("virtual_joint/rot_y");
  joint_state->name.push_back("virtual_joint/rot_z");
  joint_state->name.push_back("virtual_joint/rot_w");
  joint_state->position.push_back(q.x());
  joint_state->position.push_back(q.y());
  joint_state->position.push_back(q.z());
  joint_state->position.push_back(q.w());

  //std::vector<std::string> testing_names = robot_state_->getVariableNames();
  //ROS_ERROR_STREAM_NAMED("temp","variable names are:");
  //std::copy(testing_names.begin(), testing_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

  // Update our robot with the service response
  robot_state_->setVariablePositions(joint_state->name, joint_state->position);

  // Get just the joints we are concerned about in our planning group
  robot_state_->copyJointGroupPositions(joint_model_group_, solution);

  return true;
}

bool SrvKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
  const std::vector<double> &joint_angles,
  std::vector<geometry_msgs::Pose> &poses) const
{
  ros::WallTime n1 = ros::WallTime::now();
  if(!active_)
  {
    ROS_ERROR_NAMED("srv","kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if(joint_angles.size() != dimension_)
  {
    ROS_ERROR_NAMED("srv","Joint angles vector must have size: %d",dimension_);
    return false;
  }

  return true; //TODO
}

const std::vector<std::string>& SrvKinematicsPlugin::getJointNames() const
{
  ROS_DEBUG_STREAM_NAMED("srv","getJointNames called");
  return ik_group_info_.joint_names;
}

const std::vector<std::string>& SrvKinematicsPlugin::getLinkNames() const
{
  ROS_DEBUG_STREAM_NAMED("srv","getLinkNames called");
  return ik_group_info_.link_names;
}

const std::vector<std::string>& SrvKinematicsPlugin::getVariableNames() const
{
  ROS_DEBUG_STREAM_NAMED("srv","getVariableNames called");
  return joint_model_group_->getVariableNames();
}

} // namespace
