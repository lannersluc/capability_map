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


#ifndef REACHABILITY_ROS_KINEMATICS_H
#define REACHABILITY_ROS_KINEMATICS_H

#include <ros/ros.h>
#include "capability_map_generator/ReachabilityInterface.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace capability_map_generator
{

class ReachabilityROSKinematicsInterface : public ReachabilityInterface
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ReachabilityROSKinematicsInterface();

    bool isReachable(const octomath::Pose6D &pose) const;

  private:
    void loadKinematics(const std::string & group, const std::string & robot_description,
            const std::string & base_frame);

  private:
    robot_state::RobotStatePtr kinematic_state;
    const robot_state::JointModelGroup* joint_model_group;

    std::string tip_name;

    int attempts;
    double timeout;

    Eigen::Affine3d ikBaseTransform;
};

}

#endif
