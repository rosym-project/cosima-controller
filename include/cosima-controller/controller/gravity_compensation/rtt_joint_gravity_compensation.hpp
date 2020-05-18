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

namespace cosima
{

  namespace controller
  {

    class RTTJointGravComp : public RTT::TaskContext
    {
    public:
      RTTJointGravComp(std::string const &name);

      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();

      // This controller can handle stacked robots.
      // For this simple controller however, we do not need this.
      // But I will still keep it here to show the idea.
      bool addRobot(const unsigned int &dof);

    private:
      double getOrocosTime();

      // InputPort to receive the robot's feedback
      RTT::InputPort<sensor_msgs::JointState> in_robotstatus_port;
      RTT::FlowStatus in_robotstatus_flow;
      sensor_msgs::JointState in_robotstatus_var;

      // InputPort to receive the gravity component solved for the robot
      RTT::InputPort<Eigen::VectorXd> in_coriolisAndGravity_port;
      RTT::FlowStatus in_coriolisAndGravity_flow;
      Eigen::VectorXd in_coriolisAndGravity_var;

      // OutputPort for the torque command
      RTT::OutputPort<Eigen::VectorXd> out_torques_port;
      Eigen::VectorXd out_torques_var;

      // Store the DoF information for the added robots
      std::vector<unsigned int> vec_robot_dof;
      unsigned int total_dof_size;

      // Allow to add gravity, otherwise send zero.
      // This might be useful to deactivate,
      // if another component down the seanse-react chain,
      // adds the gravity component at the very end.
      bool include_gravity;
    };

  } // namespace controller

} // namespace cosima