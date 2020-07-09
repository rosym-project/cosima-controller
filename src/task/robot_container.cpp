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

#include "../../include/cosima-controller/task/robot_container.hpp"

RobotContainer::RobotContainer(const std::string &name)
{
    this->name_ = name;
}

void RobotContainer::setJacobianPositionInGlobalJacobian(const unsigned int j_i, const unsigned int j_j, const unsigned int j_p, const unsigned int j_q)
{
    this->j_i_ = j_i;
    this->j_j_ = j_j;
    this->j_p_ = j_p;
    this->j_q_ = j_q;
    this->jacobian_ = Eigen::MatrixXd::Zero(this->j_p_, this->j_q_);
    this->jacobian_dot_ = Eigen::MatrixXd::Zero(this->j_p_, this->j_q_);
    // Should not be used outside of a VM, but just so that nothing is uninitialized I initialize it here.
    this->jacobian_internal_ = Eigen::MatrixXd::Zero(this->j_p_, this->j_q_);
    this->jacobian_internal_dot_ = Eigen::MatrixXd::Zero(this->j_p_, this->j_q_);
}

void RobotContainer::setInertiaPositionInGlobalJacobian(const unsigned int m_i, const unsigned int m_j, const unsigned int m_p, const unsigned int m_q)
{
    this->m_i_ = m_i;
    this->m_j_ = m_j;
    this->m_p_ = m_p;
    this->m_q_ = m_q;
    this->inertia_ = Eigen::MatrixXd::Zero(this->m_p_, this->m_q_);
}

void RobotContainer::setGCPositionInGlobalJacobian(const unsigned int gc_i, const unsigned int gc_p)
{
    this->gc_i_ = gc_i;
    this->gc_p_ = gc_p;
    this->gc_ = Eigen::VectorXd::Zero(this->gc_p_);

    this->robotstatus_pos_ = Eigen::VectorXd::Zero(this->gc_p_);
    this->robotstatus_vel_ = Eigen::VectorXd::Zero(this->gc_p_);
    this->robotstatus_trq_ = Eigen::VectorXd::Zero(this->gc_p_);
}

void RobotContainer::setCartPosInGlobal(const unsigned int cart_pos_i)
{
    this->cart_pos_i = cart_pos_i;

    this->cart_pos_ = Eigen::VectorXd::Zero(7); // 7 because of rotation represented as quaternion
    this->cart_vel_ = Eigen::VectorXd::Zero(6);
}

void RobotContainer::updateJMG(const Eigen::MatrixXd &in_J_var, const Eigen::MatrixXd &in_J_dot_var, const Eigen::MatrixXd &in_M_var, const Eigen::VectorXd &in_G_var, const Eigen::VectorXd &in_robotstatus_pos, const Eigen::VectorXd &in_robotstatus_vel, const Eigen::VectorXd &in_robotstatus_trq, const Eigen::VectorXd &in_cart_pos_var, const Eigen::VectorXd &in_cart_vel_var)
{
    // RTT::log(RTT::Error) << "Update internal J from " << this->j_i_ << ", " << this->j_j_ << ", " << this->j_p_ << ", " << this->j_q_ << RTT::endlog();
    this->jacobian_ = in_J_var.block(this->j_i_, this->j_j_, this->j_p_, this->j_q_);

    // RTT::log(RTT::Error) << "Update internal J_dot from " << this->j_i_ << ", " << this->j_j_ << ", " << this->j_p_ << ", " << this->j_q_ << RTT::endlog();
    this->jacobian_dot_ = in_J_dot_var.block(this->j_i_, this->j_j_, this->j_p_, this->j_q_);

    // RTT::log(RTT::Error) << "Update internal M from " << this->m_i_ << ", " << this->m_j_ << ", " << this->m_p_ << ", " << this->m_q_ << RTT::endlog();
    this->inertia_ = in_M_var.block(this->m_i_, this->m_j_, this->m_p_, this->m_q_);

    // RTT::log(RTT::Error) << "Update internal GC from " << this->gc_i_ << ", " << this->gc_p_ << RTT::endlog();
    this->gc_ = in_G_var.segment(this->gc_i_, this->gc_p_);

    this->robotstatus_pos_ = in_robotstatus_pos.segment(this->gc_i_, this->gc_p_);
    this->robotstatus_trq_ = in_robotstatus_vel.segment(this->gc_i_, this->gc_p_);
    this->robotstatus_pos_ = in_robotstatus_trq.segment(this->gc_i_, this->gc_p_);

    this->cart_pos_ = in_cart_pos_var.segment<7>(this->cart_pos_i);
    this->cart_vel_ = in_cart_vel_var.segment<6>(this->j_i_);

    // RTT::log(RTT::Error) << "this->cart_pos_i " << this->cart_pos_i << RTT::endlog();
    // RTT::log(RTT::Error) << "this->cart_pos_ =\n"
    //                      << this->cart_pos_ << RTT::endlog();
    // RTT::log(RTT::Error) << "this->cart_vel_ =\n"
    //                      << this->cart_vel_ << RTT::endlog();
    // RTT::log(RTT::Error) << "-----------------------------" << RTT::endlog();
}

std::pair<unsigned int, unsigned int> RobotContainer::getTaskAndJointDof()
{
    return std::make_pair(this->j_p_, this->j_q_);
}

std::string RobotContainer::getRobotName()
{
    return this->name_;
}

//////////////////////////////////////////

void RobotContainer::getDebug_ReadJacobianFromGlobalPort(unsigned int &j_i, unsigned int &j_j, unsigned int &j_p, unsigned int &j_q)
{
    j_i = this->j_i_;
    j_j = this->j_j_;
    j_p = this->j_p_;
    j_q = this->j_q_;
}

void RobotContainer::getDebug_ReadInertiaFromGlobalPort(unsigned int &m_i, unsigned int &m_j, unsigned int &m_p, unsigned int &m_q)
{
    m_i = this->m_i_;
    m_j = this->m_j_;
    m_p = this->m_p_;
    m_q = this->m_q_;
}

void RobotContainer::getDebug_ReadGCFromGlobalPort(unsigned int &gc_i, unsigned int &gc_p)
{
    gc_i = this->gc_i_;
    gc_p = this->gc_p_;
}