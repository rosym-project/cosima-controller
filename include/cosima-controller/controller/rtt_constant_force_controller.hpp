/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2017 by Niels Dehio,
 *                       Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
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
#include <Eigen/Geometry>

// ROS TYPE includes
#include <sensor_msgs/JointState.h>

// header for introspection
#include "../introspection/rtt-introspection-base.hpp"

namespace cosima
{
  namespace controller
  {

    class ConstantForceController : public cogimon::RTTIntrospectionBase
    {
    public:
      ConstantForceController(std::string const &name);

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

      void compute(
          Eigen::VectorXd &in_direction,
          Eigen::MatrixXd &in_jacobian,
          double current_force,
          Eigen::VectorXd &out_torques,
          Eigen::VectorXd &out_force);
      void setDOFsize(unsigned int DOFsize);
      void setTaskSpaceDimension(const unsigned int TaskSpaceDimension);
      void setConstantForce(double new_force);
      void setConstantForceVector(const Eigen::VectorXd &new_force);
      void preparePorts();
      void displayStatus();

    private:
      // Declare input ports and their datatypes
      RTT::InputPort<double> in_force_port;
      RTT::InputPort<Eigen::VectorXd> in_direction_port;
      RTT::InputPort<Eigen::MatrixXd> in_jacobian_port;
      RTT::InputPort<Eigen::MatrixXd> in_P_port;
      RTT::InputPort<Eigen::MatrixXd> in_Pdot_port;
      RTT::InputPort<Eigen::MatrixXd> in_M_port;
      RTT::InputPort<Eigen::MatrixXd> in_Mc_port;
      RTT::InputPort<Eigen::VectorXd> in_h_port;
      RTT::InputPort<Eigen::VectorXd> in_tauM_port;
      RTT::InputPort<sensor_msgs::JointState> in_robotstatus_port;

      // Declare output ports and their datatypes
      RTT::OutputPort<Eigen::VectorXd> out_torques_port;
      RTT::OutputPort<Eigen::VectorXd> out_force_port; //for logging

      // Data flow:
      RTT::FlowStatus in_force_flow;
      RTT::FlowStatus in_direction_flow;
      RTT::FlowStatus in_jacobian_flow;
      RTT::FlowStatus in_P_flow;
      RTT::FlowStatus in_Pdot_flow;
      RTT::FlowStatus in_M_flow;
      RTT::FlowStatus in_Mc_flow;
      RTT::FlowStatus in_h_flow;
      RTT::FlowStatus in_tauM_flow;
      RTT::FlowStatus in_robotstatus_flow;

      // variables
      double in_force_var;
      Eigen::VectorXd in_direction_var;
      Eigen::MatrixXd in_jacobian_var;
      Eigen::MatrixXd in_P_var;
      Eigen::MatrixXd in_Pdot_var;
      Eigen::MatrixXd in_M_var;
      Eigen::MatrixXd in_Mc_var;
      Eigen::VectorXd in_h_var;
      Eigen::VectorXd in_tauM_var;
      sensor_msgs::JointState in_robotstatus_var;

      double current_force;
      Eigen::VectorXd out_torques_var;
      Eigen::VectorXd out_force_var;
      unsigned int DOFsize;
      unsigned int TaskSpaceDimension;
      bool portsArePrepared;

      bool include_compensation, include_gravity, include_projection, make_orthogonal_projection;
    };

  } // namespace controller

} // namespace cosima