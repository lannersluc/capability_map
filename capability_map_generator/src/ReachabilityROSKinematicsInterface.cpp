// Copyright (c) 2014, Jochen Kempfle
// All rights reserved.

/*
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/


#include "capability_map_generator/ReachabilityROSKinematicsInterface.h"
#include "capability_map/CapabilityOcTreeNode.h"
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(capability_map_generator::ReachabilityROSKinematicsInterface, capability_map_generator::ReachabilityInterface)

static pluginlib::ClassLoader<kinematics::KinematicsBase>* s_Kinematics = NULL;

namespace capability_map_generator
{

ReachabilityROSKinematicsInterface::ReachabilityROSKinematicsInterface()
{
    ros::NodeHandle nhP("~");

    std::string robot_description;
    nhP.param("robot_description", robot_description, std::string("robot_description"));

    std::string group_name;
    if(!nhP.getParam("group_name", group_name))
    {
        ROS_ERROR("No group_name defined!");
        ros::shutdown();
        exit(1);
    }

    std::string base_name;
    if(!nhP.getParam("base_name", base_name))
    {
        ROS_ERROR("No base_name defined!");
        ros::shutdown();
        exit(1);
    }

    loadKinematics(group_name, robot_description, base_name);

    if(!nhP.getParam("tip_name", tip_name))
    {
        ROS_ERROR("No tip_name defined!");
        ros::shutdown();
        exit(1);
    }

    nhP.param("ik_attempts", attempts, 5);
    nhP.param("ik_timeout", timeout, 0.5);
}

void ReachabilityROSKinematicsInterface::loadKinematics(const std::string & group,
        const std::string & robot_description,
        const std::string & base_frame)
{
    robot_model_loader::RobotModelLoader robot_model_loader(robot_description);
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    kinematic_state.reset(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    joint_model_group = kinematic_model->getJointModelGroup(group);

    // isReachable is in base_frame
    // setFromIK is in model frame
    // get: model -> base_frame
    ikBaseTransform = kinematic_state->getGlobalLinkTransform(base_frame);
}

bool ReachabilityROSKinematicsInterface::isReachable(const octomath::Pose6D &pose) const
{
    Eigen::Affine3d poseEigen = ikBaseTransform *
        Eigen::Translation3d(
                pose.x(),
                pose.y(),
                pose.z()) *
        Eigen::Quaterniond(
                pose.rot().u(),
                pose.rot().x(),
                pose.rot().y(),
                pose.rot().z());

    bool reachable = kinematic_state->setFromIK(joint_model_group, poseEigen, tip_name, attempts, timeout);
    //ROS_INFO_STREAM("IK Query Translation: " << poseEigen.translation());
    //ROS_INFO_STREAM("Rotation: " << poseEigen.rotation());
    //ROS_INFO("is: %d", reachable);
    return reachable;
}

} // namespace

