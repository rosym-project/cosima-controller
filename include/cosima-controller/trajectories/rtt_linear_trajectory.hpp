/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2020 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <Eigen/Dense>
#include <iostream>

// ROS TYPE includes
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <geometry_msgs/Pose.h>

// KDL includes
#include <kdl/velocityprofile_trap.hpp>

namespace cosima
{

  namespace trajectories
  {

    class LinearTrajectory : public RTT::TaskContext
    {
    public:
      LinearTrajectory(std::string const &name);

      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();

      void setJointTargetEigen(const Eigen::VectorXd &target);

    private:
      double getOrocosTime();

      ////////////// JOINT SPACE //////////////
      RTT::InputPort<sensor_msgs::JointState> in_robotstatus_port;
      RTT::FlowStatus in_robotstatus_flow;
      sensor_msgs::JointState in_robotstatus_var;

      RTT::OutputPort<trajectory_msgs::JointTrajectoryPoint> out_joint_cmd_port;
      trajectory_msgs::JointTrajectoryPoint out_joint_cmd_var;

      Eigen::VectorXd js_target;
      Eigen::VectorXd js_current;


      ////////////// TASK SPACE //////////////




      unsigned int DoF;
      bool new_js_command;
      bool once;

      double max_rad;
    };

  } // namespace trajectories

} // namespace cosima