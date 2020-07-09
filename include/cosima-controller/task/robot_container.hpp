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
#include <rtt/Logger.hpp>

#include "manipulator_data_supplier.hpp"

class RobotContainer : public ManipulatorDataSupplier
{
public:
  RobotContainer(const std::string &name);
  // ~RobotContainer();

  void setJacobianPositionInGlobalJacobian(const unsigned int j_i, const unsigned int j_j, const unsigned int j_p, const unsigned int j_q);
  void setInertiaPositionInGlobalJacobian(const unsigned int m_i, const unsigned int m_j, const unsigned int m_p, const unsigned int m_q);
  void setGCPositionInGlobalJacobian(const unsigned int gc_i, const unsigned int gc_p);
  void setCartPosInGlobal(const unsigned int cart_pos_i);
  void updateJMG(const Eigen::MatrixXd &in_J_var, const Eigen::MatrixXd &in_J_dot_var, const Eigen::MatrixXd &in_M_var, const Eigen::VectorXd &in_G_var, const Eigen::VectorXd &in_robotstatus_pos, const Eigen::VectorXd &in_robotstatus_vel, const Eigen::VectorXd &in_robotstatus_trq, const Eigen::VectorXd &in_cart_pos_var, const Eigen::VectorXd &in_cart_vel_var);

  std::pair<unsigned int, unsigned int> getTaskAndJointDof();

  // Overridden from ManipulatorDataSupplier
  std::string getRobotName();

  // Eigen::MatrixXd jacobian_;
  // Eigen::MatrixXd jacobian_dot_;
  // Eigen::MatrixXd inertia_;
  // Eigen::VectorXd gc_;

  //////////////////////////////////////////////////////////
  void getDebug_ReadJacobianFromGlobalPort(unsigned int &j_i, unsigned int &j_j, unsigned int &j_p, unsigned int &j_q);
  void getDebug_ReadInertiaFromGlobalPort(unsigned int &m_i, unsigned int &m_j, unsigned int &m_p, unsigned int &m_q);
  void getDebug_ReadGCFromGlobalPort(unsigned int &gc_i, unsigned int &gc_p);

private:
  std::string name_;

  unsigned int j_i_, j_j_, j_p_, j_q_;
  unsigned int m_i_, m_j_, m_p_, m_q_;
  unsigned int gc_i_, gc_p_;
  unsigned int cart_pos_i;
};
