/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2019 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
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

#include <memory>
#include <string>
#include <Eigen/Dense>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>

#include <sensor_msgs/JointState.h>

class ControlComponentContainer
{
public:
  ControlComponentContainer(const std::string &name);
  // ~ControlComponentContainer();

  void sendJMG();

  void initializeDimensions(RTT::TaskContext *tc, const unsigned int j_p, const unsigned int j_q, const unsigned int m_p, const unsigned int m_q, const unsigned int gc_p);

  void printDebug();

  void setVM(bool vm);

  std::string getName();

  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd jacobian_dot_;
  Eigen::MatrixXd inertia_;
  Eigen::VectorXd gc_;

  Eigen::MatrixXd inertia_c_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd P_dot_;

  Eigen::VectorXd int_wrench_;

  Eigen::VectorXd vm_fdb_cart_pos_;
  Eigen::VectorXd vm_fdb_cart_vel_;

  Eigen::VectorXd robotstate_pos_;
  Eigen::VectorXd robotstate_vel_;
  Eigen::VectorXd robotstate_trq_;

private:
  std::string name_;
  bool is_vm_;

  sensor_msgs::JointState robotstate_;

  RTT::OutputPort<Eigen::MatrixXd> out_Jacobian_co_;
  RTT::OutputPort<Eigen::MatrixXd> out_Jacobian_dot_co_;
  RTT::OutputPort<Eigen::MatrixXd> out_Inertia_co_;
  RTT::OutputPort<Eigen::VectorXd> out_GC_co_;

  RTT::OutputPort<Eigen::MatrixXd> out_Inertia_c_co_;
  RTT::OutputPort<Eigen::MatrixXd> out_P_co_;
  RTT::OutputPort<Eigen::MatrixXd> out_P_dot_co_;

  RTT::OutputPort<Eigen::VectorXd> out_int_wrench_co_;

  RTT::OutputPort<Eigen::VectorXd> out_vm_fdb_cart_pos_;
  RTT::OutputPort<Eigen::VectorXd> out_vm_fdb_cart_vel_;

  RTT::OutputPort<sensor_msgs::JointState> out_robotstate_co_;
};
