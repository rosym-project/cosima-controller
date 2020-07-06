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

#include "rtt_stacked_controller.hpp"

namespace cosima
{

  namespace controller
  {

    class RTTCartImpCtrl : public RTTStackedCtrl
    {
    public:
      RTTCartImpCtrl(std::string const &name);

      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();

      // This controller can handle stacked robots.
      bool addRobot(const unsigned int &task_dof, const unsigned int &joint_dof);

      void setGains(const double &kp, const double &kd);

      // void setGainsPos(const Eigen::VectorXd &kp, const Eigen::VectorXd &kd);
      // void setGainsOrn(const Eigen::VectorXd &kp, const Eigen::VectorXd &kd);

    private:
      double getOrocosTime();

      // InputPort to receive the joint space inertia matrix for the controlled link (EEF)
      RTT::InputPort<Eigen::MatrixXd> in_inertia_port;
      RTT::FlowStatus in_inertia_flow;
      Eigen::MatrixXd in_inertia_var;

      // InputPort to receive cartesian position commands, if stacked: (3 pos_1,4 orn_1),(3 pos_2,4 orn_2),...,(3 pos_n,4 orn_n)
      RTT::InputPort<Eigen::VectorXd> in_cart_pos_cmd_port;
      RTT::FlowStatus in_cart_pos_cmd_flow;
      Eigen::VectorXd in_cart_pos_cmd_var;

      // InputPort to receive cartesian velocity commands, if stacked: (3 pos_1,4 orn_1),(3 pos_2,4 orn_2),...,(3 pos_n,4 orn_n)
      RTT::InputPort<Eigen::VectorXd> in_cart_vel_cmd_port;
      RTT::FlowStatus in_cart_vel_cmd_flow;
      Eigen::VectorXd in_cart_vel_cmd_var;

      // InputPort to receive cartesian acceleration commands, if stacked: (3 pos_1,4 orn_1),(3 pos_2,4 orn_2),...,(3 pos_n,4 orn_n)
      RTT::InputPort<Eigen::VectorXd> in_cart_acc_cmd_port;
      RTT::FlowStatus in_cart_acc_cmd_flow;
      Eigen::VectorXd in_cart_acc_cmd_var;

      // InputPort to receive the robot's feedback
      RTT::InputPort<sensor_msgs::JointState> in_robotstatus_port;
      RTT::FlowStatus in_robotstatus_flow;
      sensor_msgs::JointState in_robotstatus_var;

      // InputPort to receive cartesian position feedback, if stacked: (3 pos_1,4 orn_1),(3 pos_2,4 orn_2),...,(3 pos_n,4 orn_n)
      RTT::InputPort<Eigen::VectorXd> in_cart_pos_fdb_port;
      RTT::FlowStatus in_cart_pos_fdb_flow;
      Eigen::VectorXd in_cart_pos_fdb_var;

      // InputPort to receive cartesian velocity feedback, if stacked: (3 pos_1,4 orn_1),(3 pos_2,4 orn_2),...,(3 pos_n,4 orn_n)
      RTT::InputPort<Eigen::VectorXd> in_cart_vel_fdb_port;
      RTT::FlowStatus in_cart_vel_fdb_flow;
      Eigen::VectorXd in_cart_vel_fdb_var;

      // InputPort to receive cartesian acceleration feedback, if stacked: (3 pos_1,4 orn_1),(3 pos_2,4 orn_2),...,(3 pos_n,4 orn_n)
      RTT::InputPort<Eigen::VectorXd> in_cart_acc_fdb_port;
      RTT::FlowStatus in_cart_acc_fdb_flow;
      Eigen::VectorXd in_cart_acc_fdb_var;

      // InputPort to receive the gravity component solved for the robot
      RTT::InputPort<Eigen::VectorXd> in_coriolisAndGravity_port;
      RTT::FlowStatus in_coriolisAndGravity_flow;
      Eigen::VectorXd in_coriolisAndGravity_var;

      // InputPort to receive the jacobian(s)
      RTT::InputPort<Eigen::VectorXd> in_jacobian_port;
      RTT::FlowStatus in_jacobian_flow;
      Eigen::VectorXd in_jacobian_var;

      // InputPort to receive the jacobian dot(s)
      RTT::InputPort<Eigen::VectorXd> in_jacobianDot_port;
      RTT::FlowStatus in_jacobianDot_flow;
      Eigen::VectorXd in_jacobianDot_var;

      // InputPort to receive the projection matrix
      RTT::InputPort<Eigen::VectorXd> in_projection_port;
      RTT::FlowStatus in_projection_flow;
      Eigen::VectorXd in_projection_var;

      // InputPort to receive the projection matrix
      RTT::InputPort<Eigen::VectorXd> in_projectionDot_port;
      RTT::FlowStatus in_projectionDot_flow;
      Eigen::VectorXd in_projectionDot_var;


      // OutputPort for the torque command
      RTT::OutputPort<Eigen::VectorXd> out_torques_port;
      Eigen::VectorXd out_torques_var;

      // Allow to add gravity, otherwise send zero.
      // This might be useful to deactivate,
      // if another component down the seanse-react chain,
      // adds the gravity component at the very end.
      bool include_gravity;


      // Helpers

      double kp, kd;
    };

  } // namespace controller

} // namespace cosima