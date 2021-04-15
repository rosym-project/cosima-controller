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
#include <geometry_msgs/Pose.h>

// KDL includes
#include <kdl/velocityprofile_trap.hpp>

// header for introspection
#include "../introspection/rtt-introspection-base.hpp"

namespace cosima
{

  namespace trajectories
  {

    class PoseInterpolator : public cogimon::RTTIntrospectionBase
    {
    public:
      PoseInterpolator(std::string const &name);

      ///////////////////////////////////////////
      // Internal mirrors of the default Orocos
      // life cycle from the introspection base.
      ///////////////////////////////////////////
      bool configureHookInternal();
      bool startHookInternal();
      void updateHookInternal();
      void stopHookInternal();
      void cleanupHookInternal();
      ///////////////////////////////////////////

    private:
      double getOrocosTime();

      // InputPort to receive the pose command
      RTT::InputPort<geometry_msgs::Pose> in_pose_port;
      RTT::FlowStatus in_pose_flow;
      geometry_msgs::Pose in_pose_var;

      // InputPort to receive the pass-through pose commands (use for high frequency)
      RTT::InputPort<geometry_msgs::Pose> in_pose_pt_port;
      RTT::FlowStatus in_pose_pt_flow;
      geometry_msgs::Pose in_pose_pt_var;

      // InputPort to receive the current pose matrix
      // RTT::InputPort<Eigen::MatrixXd> in_current_pose_port;
      // RTT::FlowStatus in_current_pose_flow;
      // Eigen::MatrixXd in_current_pose_var;
      RTT::InputPort<geometry_msgs::Pose> in_current_pose_port;
      RTT::FlowStatus in_current_pose_flow;
      geometry_msgs::Pose in_current_pose_var;

      // OutputPort for the interpolated pose matrix commands
      RTT::OutputPort<Eigen::MatrixXd> out_pose_matrix_port;
      Eigen::MatrixXd out_pose_matrix_var;

      // OutputPort for the interpolated pose commands
      RTT::OutputPort<geometry_msgs::Pose> out_pose_port;
      geometry_msgs::Pose out_pose_var;

      KDL::VelocityProfile_Trap trap_gen;

      double st;

      double traj_max_vel, traj_max_acc, traj_time;

      bool once;
      bool first_iter;
      bool ignore_traj;

      Eigen::Vector3d first;
      Eigen::Vector3d start;
      Eigen::Vector3d goal;
      Eigen::Vector3d diff;
      Eigen::Quaterniond qFirst;
      Eigen::Quaterniond qStart;
      Eigen::Quaterniond qGoal;
    };

  } // namespace trajectories

} // namespace cosima