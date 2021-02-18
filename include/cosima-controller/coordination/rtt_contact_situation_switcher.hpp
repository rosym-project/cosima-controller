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

#include "monitor_wrench.hpp"

// ROS TYPE includes
#include <geometry_msgs/Pose.h>

namespace cosima
{

  namespace coordination
  {

    class ContactSituationSwitcher : public RTT::TaskContext
    {
    public:
      ContactSituationSwitcher(std::string const &name);

      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();

      void addWrenchMonitor(std::string const &name);
      void setWrenchMonitorBounds(std::string const &name, unsigned int const &dimension, double lower, double upper);

    private:
      double getOrocosTime();

      // // InputPort to receive the pose command
      // RTT::InputPort<geometry_msgs::Pose> in_pose_port;
      // RTT::FlowStatus in_pose_flow;
      // geometry_msgs::Pose in_pose_var;

      // // InputPort to receive the pass-through pose commands (use for high frequency)
      // RTT::InputPort<geometry_msgs::Pose> in_pose_pt_port;
      // RTT::FlowStatus in_pose_pt_flow;
      // geometry_msgs::Pose in_pose_pt_var;

      // // InputPort to receive the current pose matrix
      // // RTT::InputPort<Eigen::MatrixXd> in_current_pose_port;
      // // RTT::FlowStatus in_current_pose_flow;
      // // Eigen::MatrixXd in_current_pose_var;
      // RTT::InputPort<geometry_msgs::Pose> in_current_pose_port;
      // RTT::FlowStatus in_current_pose_flow;
      // geometry_msgs::Pose in_current_pose_var;

      // // OutputPort for the interpolated pose matrix commands
      // RTT::OutputPort<Eigen::MatrixXd> out_pose_matrix_port;
      // Eigen::MatrixXd out_pose_matrix_var;

      // // OutputPort for the interpolated pose commands
      // RTT::OutputPort<geometry_msgs::Pose> out_pose_port;
      // geometry_msgs::Pose out_pose_var;

      std::map<std::string, std::shared_ptr<MonitorWrench> > map_monitors_wrench;

    };

  } // namespace coordination

} // namespace cosima