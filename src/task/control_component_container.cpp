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

#include "../../include/cosima-controller/task/control_component_container.hpp"

ControlComponentContainer::ControlComponentContainer(const std::string &name)
{
    this->name_ = name;
    // Let us leave this like that for now
    this->is_vm_ = true;
}

void ControlComponentContainer::setVM(bool vm)
{
    this->is_vm_ = vm;
}

void ControlComponentContainer::sendJMG()
{
    for (unsigned int i = 0; i < this->robotstate_.position.size(); i++)
    {
        // Is there something smarter?
        this->robotstate_.position[i] = this->robotstate_pos_[i];
        this->robotstate_.velocity[i] = this->robotstate_vel_[i];
        this->robotstate_.effort[i] = this->robotstate_trq_[i];
    }

    this->out_Jacobian_co_.write(this->jacobian_);
    this->out_Jacobian_dot_co_.write(this->jacobian_dot_);
    this->out_Inertia_co_.write(this->inertia_);
    this->out_GC_co_.write(this->gc_);

    this->out_Inertia_c_co_.write(this->inertia_c_);
    this->out_P_co_.write(this->P_);
    this->out_P_dot_co_.write(this->P_dot_);
    this->out_robotstate_co_.write(this->robotstate_);

    if (this->is_vm_)
    {
        this->out_int_wrench_co_.write(this->int_wrench_);

        this->out_vm_fdb_cart_pos_.write(this->vm_fdb_cart_pos_);
        this->out_vm_fdb_cart_vel_.write(this->vm_fdb_cart_vel_);
    }
}

void ControlComponentContainer::initializeDimensions(RTT::TaskContext *tc, const unsigned int j_p, const unsigned int j_q, const unsigned int m_p, const unsigned int m_q, const unsigned int gc_p)
{
    this->jacobian_ = Eigen::MatrixXd::Zero(j_p, j_q);
    this->jacobian_dot_ = Eigen::MatrixXd::Zero(j_p, j_q);
    this->inertia_ = Eigen::MatrixXd::Zero(m_p, m_q);
    this->gc_ = Eigen::VectorXd::Zero(gc_p);

    this->inertia_c_ = Eigen::MatrixXd::Zero(m_p, m_q);
    this->P_ = Eigen::MatrixXd::Zero(m_p, m_q);
    this->P_dot_ = Eigen::MatrixXd::Zero(m_p, m_q);

    this->int_wrench_ = Eigen::VectorXd::Zero(j_p);

    this->vm_fdb_cart_pos_ = Eigen::VectorXd::Zero(7); // TODO remove hardcoded
    this->vm_fdb_cart_vel_ = Eigen::VectorXd::Zero(6); // TODO remove hardcoded

    this->robotstate_ = sensor_msgs::JointState();
    this->robotstate_pos_ = Eigen::VectorXd::Zero(gc_p);
    this->robotstate_vel_ = Eigen::VectorXd::Zero(gc_p);
    this->robotstate_trq_ = Eigen::VectorXd::Zero(gc_p);
    for (unsigned int i = 0; i < gc_p; i++)
    {
        this->robotstate_.position.push_back(0.0);
        this->robotstate_.velocity.push_back(0.0);
        this->robotstate_.effort.push_back(0.0);
    }

    out_Jacobian_co_.setName("out_" + this->name_ + "_Jacobian_port");
    out_Jacobian_co_.doc("Jacobian Output Port for Ctrl " + this->name_);
    out_Jacobian_co_.setDataSample(this->jacobian_);
    out_Jacobian_dot_co_.setName("out_" + this->name_ + "_Jacobian_dot_port");
    out_Jacobian_dot_co_.doc("Jacobian_dot Output Port for Ctrl " + this->name_);
    out_Jacobian_dot_co_.setDataSample(this->jacobian_dot_);
    out_Inertia_co_.setName("out_" + this->name_ + "_Inertia_port");
    out_Inertia_co_.doc("Inertia Output Port for Ctrl " + this->name_);
    out_Inertia_co_.setDataSample(this->inertia_);
    out_GC_co_.setName("out_" + this->name_ + "_GC_port");
    out_GC_co_.doc("GC Output Port for Ctrl " + this->name_);
    out_GC_co_.setDataSample(this->gc_);

    out_Inertia_c_co_.setName("out_" + this->name_ + "_Inertia_c_port");
    out_Inertia_c_co_.doc("Inertia_c Output Port for Ctrl " + this->name_);
    out_Inertia_c_co_.setDataSample(this->inertia_c_);
    out_P_co_.setName("out_" + this->name_ + "_P_port");
    out_P_co_.doc("P Output Port for Ctrl " + this->name_);
    out_P_co_.setDataSample(this->P_);
    out_P_dot_co_.setName("out_" + this->name_ + "_P_dot_port");
    out_P_dot_co_.doc("P_dot Output Port for Ctrl " + this->name_);
    out_P_dot_co_.setDataSample(this->P_dot_);

    out_robotstate_co_.setName("out_" + this->name_ + "_robotstatus_port");
    out_robotstate_co_.doc("robotstatus Output Port for Ctrl " + this->name_);
    out_robotstate_co_.setDataSample(this->robotstate_);

    if (this->is_vm_)
    {
        out_int_wrench_co_.setName("out_" + this->name_ + "_int_wrench_port");
        out_int_wrench_co_.doc("Internal Wrench Output Port for Ctrl " + this->name_);
        out_int_wrench_co_.setDataSample(this->int_wrench_);

        out_vm_fdb_cart_pos_.setName("out_" + this->name_ + "_vm_fdb_cart_pos_port");
        out_vm_fdb_cart_pos_.doc("VM Cart Pos Feedback Output Port for Ctrl " + this->name_);
        out_vm_fdb_cart_pos_.setDataSample(this->vm_fdb_cart_pos_);

        out_vm_fdb_cart_vel_.setName("out_" + this->name_ + "_vm_fdb_cart_vel_port");
        out_vm_fdb_cart_vel_.doc("VM Cart Vel Feedback Output Port for Ctrl " + this->name_);
        out_vm_fdb_cart_vel_.setDataSample(this->vm_fdb_cart_vel_);
    }

    tc->ports()->addPort(out_Jacobian_co_);
    tc->ports()->addPort(out_Jacobian_dot_co_);
    tc->ports()->addPort(out_Inertia_co_);
    tc->ports()->addPort(out_GC_co_);

    tc->ports()->addPort(out_Inertia_c_co_);
    tc->ports()->addPort(out_P_co_);
    tc->ports()->addPort(out_P_dot_co_);

    tc->ports()->addPort(out_robotstate_co_);

    if (this->is_vm_)
    {
        tc->ports()->addPort(out_int_wrench_co_);

        tc->ports()->addPort(out_vm_fdb_cart_pos_);
        tc->ports()->addPort(out_vm_fdb_cart_vel_);
    }
}

std::string ControlComponentContainer::getName()
{
    return this->name_;
}

void ControlComponentContainer::printDebug()
{
    RTT::log(RTT::Error) << "[PrintDebug] " << this->name_ << ": jacobian_ " << jacobian_.rows() << ", " << jacobian_.cols() << RTT::endlog();
}