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

#include "../../include/cosima-controller/task/control_objective_container.hpp"
#include "../../include/cosima-controller/task/vm_container.hpp"

using namespace cosima;
using namespace util;

ControlObjectiveContainer::ControlObjectiveContainer(const std::string &name)
{
    this->name_ = name;
    this->IamVM_ = false;
    this->selectionDirection_ = 0;
    this->currently_receives_gemini_ = COKind::None;
    this->current_active_cs_filter_idx_ = 0;
    this->my_kind_ = COKind::None;

    this->matDecomp = MatrixDecomposition();
    this->matDecomp.epsilon = 1.0e-06;
    this->matDecomp.epsilon = 0.01;
    this->matDecomp.borderA = 0.1;
    this->matDecomp.borderB = 0.01;
    this->matDecomp.changeRows = false;
    this->matDecomp.useAauto = true;
    this->matDecomp.useAuser = true;
    this->setMethod(2);
    start_time = -1;
}

void ControlObjectiveContainer::setIamVM(const bool iamvm)
{
    this->IamVM_ = iamvm;
}

void ControlObjectiveContainer::setAssociatedRobot(std::shared_ptr<ManipulatorDataSupplier> associated_robot)
{
    this->associated_robot_ = associated_robot;
}

void ControlObjectiveContainer::initializeDimensions(const unsigned int totalInternalTaskDof, const unsigned int totalInternalJointDof)
{
    // RTT::log(RTT::Error) << ">>>>>>>>>>>> " << this->getName() << ": jacobian_ " << totalInternalTaskDof << ", " << totalInternalJointDof << RTT::endlog();
    jacobian_ = Eigen::MatrixXd::Zero(totalInternalTaskDof, totalInternalJointDof);
    jacobian_dot_ = Eigen::MatrixXd::Zero(totalInternalTaskDof, totalInternalJointDof);
    inertia_ = Eigen::MatrixXd::Zero(totalInternalJointDof, totalInternalJointDof);
    gc_ = Eigen::VectorXd::Zero(totalInternalJointDof);
    robotstate_pos_ = Eigen::VectorXd::Zero(totalInternalJointDof);
    robotstate_vel_ = Eigen::VectorXd::Zero(totalInternalJointDof);
    robotstate_trq_ = Eigen::VectorXd::Zero(totalInternalJointDof);
}

void ControlObjectiveContainer::printDebug()
{
    RTT::log(RTT::Error) << "[PrintDebug] " << this->name_ << ":\njacobian_ " << jacobian_.rows() << ", " << jacobian_.cols() << "\n (out): " << jmgc_control_position_.j_i_ << ", " << jmgc_control_position_.j_j_ << ", " << jmgc_control_position_.j_p_ << ", " << jmgc_control_position_.j_q_ << RTT::endlog();
}

void ControlObjectiveContainer::drawJMGFromAssociatedRobot()
{
    if (opType == OPType::Chain || opType == OPType::Combined_Chain)
    {
        jacobian_.setIdentity();
        jacobian_dot_.setZero();
    }
    else
    {
        jacobian_ = this->associated_robot_->jacobian_;
        jacobian_dot_ = this->associated_robot_->jacobian_dot_;
    }
    inertia_ = this->associated_robot_->inertia_;
    gc_ = this->associated_robot_->gc_;
    robotstate_pos_ = this->associated_robot_->robotstatus_pos_;
    robotstate_vel_ = this->associated_robot_->robotstatus_vel_;
    robotstate_trq_ = this->associated_robot_->robotstatus_trq_;
}

bool ControlObjectiveContainer::containsFilterGemini(const std::string &cs_name, const std::string &filterName, const std::string &identifier)
{
    std::string mutableName = filterName;
    this->replace(mutableName, identifier, ":C:");
    if (this->map_cs_2_idx_.count(cs_name) <= 0)
    {
        return false;
    }
    unsigned int index_for_cs = this->map_cs_2_idx_[cs_name];
    Filter_Struct fs = this->vec_filter_[index_for_cs];
    if (fs.name.compare(mutableName) == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void ControlObjectiveContainer::applyFilter(const double activation)
{
    // Check currently active contact situation:
    Filter_Struct fs = this->vec_filter_[this->current_active_cs_filter_idx_]; // step A).
    if (this->IamVM_ && this->opType == OPType::Internal_Force)
    {
        this->jacobian_filtered_ = this->associated_robot_->jacobian_internal_;
        this->jacobian_dot_filtered_ = this->associated_robot_->jacobian_internal_dot_;
        this->gc_filtered_ = this->gc_;

        in_activation_var = activation * this->Ones_task_dof_;
        triggerGeminiCalculationOncePerRun(this->jacobian_filtered_, activation * Identity_filtered_dof_);
    }
    else
    {
        if (!fs.data.isIdentity())
        {
            Eigen::MatrixXd selection;
            // This mean that when a = 0 => Identity and a = 1 => S. E.g., :M: would use this.
            if (this->selectionDirection_ == 0)
            {
                selection = Identity_filtered_dof_ - activation * (Identity_filtered_dof_ - fs.data);
                // in_activation_var = this->Ones_task_dof_ - activation * this->Ones_task_dof_;
                in_activation_var = activation * this->Ones_task_dof_;

                // RTT::log(RTT::Error) << this->getName() << ": selectionDirection_ = " << selectionDirection_ << ", Identity_filtered_dof_ =\n"
                //                      << Identity_filtered_dof_ << "\nactivation = " << activation << ", fs.data =\n"
                //                      << fs.data << "\nselection =\n"
                //                      << selection << RTT::endlog();
            }
            // This mean that when a = 0 => Zero and a = 1 => S. E.g., :C: would use this.
            else if (this->selectionDirection_ == 1)
            {
                selection = activation * fs.data;
                in_activation_var = activation * this->Ones_task_dof_;
                // RTT::log(RTT::Error) << this->getName() << ": selectionDirection_ = " << selectionDirection_ << ", activation = " << activation << ", fs.data =\n"
                //                      << fs.data << "\nselection =\n"
                //                      << selection << RTT::endlog();
            }
            // in_activation_var = selection * this->Ones_task_dof_;

            // RTT::log(RTT::Error) << this->getName() << ": Ones_task_dof_ =\n"
            //                      << Ones_task_dof_ << "\nin_activation_var =\n"
            //                      << in_activation_var << RTT::endlog();

            jacobian_filtered_ = /* RR_world_fdir * */ (selection * (/* RR_world_fdir.inverse() * */ jacobian_));
            jacobian_dot_filtered_ = /* RR_world_fdir * */ (selection * (/* RR_world_fdir.inverse() * */ jacobian_dot_));

            // RTT::log(RTT::Error) << this->getName() << ": jacobian_filtered_ =\n"
            //                      << jacobian_filtered_ << "\njacobian_dot_filtered_ =\n"
            //                      << jacobian_dot_filtered_ << RTT::endlog();

            this->gc_filtered_ = this->gc_; // TODO perhaps we do not need GC at all in some filtered or constrained manner...
            triggerGeminiCalculationOncePerRun(jacobian_filtered_, selection);
        }
        else
        {
            // RTT::log(RTT::Error) << this->getName() << " current_active_cs_filter_idx_ = " << current_active_cs_filter_idx_ << ", fs =\n"
            //                      << fs.data << RTT::endlog();
            this->jacobian_filtered_ = jacobian_;
            this->jacobian_dot_filtered_ = jacobian_dot_;
            this->projection_.setIdentity();
            this->projection_dot_.setZero();
            this->projection_gemini_.setZero();
            this->inertia_filtered_ = this->inertia_;
            this->gc_filtered_ = this->gc_; // TODO perhaps we do not need GC at all in some filtered or constrained manner...
            if (this->IamVM_)
            {
                triggerGeminiCalculationOncePerRun(jacobian_filtered_, Identity_filtered_dof_);
            }
            // TODO DLW ??? in_activation_var = this->Ones_task_dof_;
        }
    }
}

bool ControlObjectiveContainer::replace(std::string &str, const std::string &from, const std::string &to)
{
    size_t start_pos = str.find(from);
    if (start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

void ControlObjectiveContainer::activateContactSituation(const std::string &csName)
{
    if (this->map_cs_2_idx_.find(csName) == this->map_cs_2_idx_.end())
    {
        RTT::log(RTT::Error) << this->getName() << ": Could not Switch to " << csName << ", because CS not found! Use Default!" << RTT::endlog();
        this->current_active_cs_filter_idx_ = 0;

        if (this->IamVM_)
        {
            // this->associated_robot_->fadeConstraintActivation_ = 0;
            this->associated_robot_->fadeConstraintActivation_ = 1;
        }
    }
    else
    {
        this->current_active_cs_filter_idx_ = this->map_cs_2_idx_[csName];

        if (this->IamVM_)
        {
            this->associated_robot_->fadeConstraintActivation_ = 1;
        }
    }
    this->currently_receives_gemini_ = this->vec_receives_gemini_[this->current_active_cs_filter_idx_];
    this->my_kind_ = this->vec_my_kind_[this->current_active_cs_filter_idx_];
    this->selectionDirection_ = this->vec_selection_direction_[this->current_active_cs_filter_idx_];

    RTT::log(RTT::Error) << this->getName() << ": this->selectionDirection_ = " << this->selectionDirection_ << " for index = " << this->current_active_cs_filter_idx_ << RTT::endlog();
    // Perhaps also store the filter globally somewhere, to avoid step A) during runtime?
}

void ControlObjectiveContainer::outputFilteredJMG(Eigen::MatrixXd &in_ctrl_J_var, Eigen::MatrixXd &in_ctrl_J_dot_var, Eigen::MatrixXd &in_ctrl_M_var, Eigen::VectorXd &in_ctrl_GC_var, Eigen::MatrixXd &in_ctrl_Mc_var, Eigen::MatrixXd &in_ctrl_P_var, Eigen::MatrixXd &in_ctrl_P_dot_var, Eigen::VectorXd &in_ctrl_robotstate_pos, Eigen::VectorXd &in_ctrl_robotstate_vel, Eigen::VectorXd &in_ctrl_robotstate_trq, Eigen::VectorXd &in_int_wrench_var, Eigen::VectorXd &in_out_vm_fdb_cart_pos_var, Eigen::VectorXd &in_out_vm_fdb_cart_vel_var)
{
    // RTT::log(RTT::Error) << "> " << this->name_ << ": SAVE internal j " << jacobian_filtered_.rows() << ", " << jacobian_filtered_.cols() << " in external place " << jmgc_control_position_.j_i_ << ", " << jmgc_control_position_.j_j_ << ", " << jmgc_control_position_.j_p_ << ", " << jmgc_control_position_.j_q_ << RTT::endlog();
    in_ctrl_J_var.block(jmgc_control_position_.j_i_, jmgc_control_position_.j_j_, jmgc_control_position_.j_p_, jmgc_control_position_.j_q_) = jacobian_filtered_;

    // RTT::log(RTT::Error) << "> " << this->name_ << ": SAVE internal j_dot " << jacobian_dot_filtered_.rows() << ", " << jacobian_dot_filtered_.cols() << " in external place " << jmgc_control_position_.j_i_ << ", " << jmgc_control_position_.j_j_ << ", " << jmgc_control_position_.j_p_ << ", " << jmgc_control_position_.j_q_ << RTT::endlog();
    in_ctrl_J_dot_var.block(jmgc_control_position_.j_i_, jmgc_control_position_.j_j_, jmgc_control_position_.j_p_, jmgc_control_position_.j_q_) = jacobian_dot_filtered_;

    // RTT::log(RTT::Error) << "> " << this->name_ << ": SAVE internal M " << inertia_.rows() << ", " << inertia_.cols() << " in external place " << jmgc_control_position_.m_i_ << ", " << jmgc_control_position_.m_j_ << ", " << jmgc_control_position_.m_p_ << ", " << jmgc_control_position_.m_q_ << RTT::endlog();
    in_ctrl_M_var.block(jmgc_control_position_.m_i_, jmgc_control_position_.m_j_, jmgc_control_position_.m_p_, jmgc_control_position_.m_q_) = inertia_;

    // RTT::log(RTT::Error) << "> " << this->name_ << ": SAVE internal GC " << gc_filtered_.rows() << " in external place " << jmgc_control_position_.gc_i_ << ", " << jmgc_control_position_.gc_p_ << RTT::endlog();
    in_ctrl_GC_var.segment(jmgc_control_position_.gc_i_, jmgc_control_position_.gc_p_) = gc_filtered_;

    // RTT::log(RTT::Error) << "> " << this->name_ << ": SAVE internal Mc " << inertia_filtered_.rows() << ", " << inertia_filtered_.cols() << " in external place " << jmgc_control_position_.m_i_ << ", " << jmgc_control_position_.m_j_ << ", " << jmgc_control_position_.m_p_ << ", " << jmgc_control_position_.m_q_ << RTT::endlog();
    in_ctrl_Mc_var.block(jmgc_control_position_.m_i_, jmgc_control_position_.m_j_, jmgc_control_position_.m_p_, jmgc_control_position_.m_q_) = inertia_filtered_;

    // RTT::log(RTT::Error) << "> " << this->name_ << ": SAVE internal P " << projection_.rows() << ", " << projection_.cols() << " in external place " << jmgc_control_position_.m_i_ << ", " << jmgc_control_position_.m_j_ << ", " << jmgc_control_position_.m_p_ << ", " << jmgc_control_position_.m_q_ << RTT::endlog();
    in_ctrl_P_var.block(jmgc_control_position_.m_i_, jmgc_control_position_.m_j_, jmgc_control_position_.m_p_, jmgc_control_position_.m_q_) = projection_;

    // RTT::log(RTT::Error) << "> " << this->name_ << ": SAVE internal P_dot " << projection_dot_.rows() << ", " << projection_dot_.cols() << " in external place " << jmgc_control_position_.m_i_ << ", " << jmgc_control_position_.m_j_ << ", " << jmgc_control_position_.m_p_ << ", " << jmgc_control_position_.m_q_ << RTT::endlog();
    in_ctrl_P_dot_var.block(jmgc_control_position_.m_i_, jmgc_control_position_.m_j_, jmgc_control_position_.m_p_, jmgc_control_position_.m_q_) = projection_dot_;
    // }

    in_ctrl_robotstate_pos.segment(jmgc_control_position_.gc_i_, jmgc_control_position_.gc_p_) = robotstate_pos_;
    in_ctrl_robotstate_vel.segment(jmgc_control_position_.gc_i_, jmgc_control_position_.gc_p_) = robotstate_vel_;
    in_ctrl_robotstate_trq.segment(jmgc_control_position_.gc_i_, jmgc_control_position_.gc_p_) = robotstate_trq_;

    if (this->IamVM_)
    {
        if (this->opType == OPType::Internal_Force)
        {
            in_int_wrench_var.segment(jmgc_control_position_.j_i_, jmgc_control_position_.j_p_) = this->associated_robot_->estimated_force_directionEE_;
        }
        in_out_vm_fdb_cart_pos_var = this->associated_robot_->cart_pos_;
        in_out_vm_fdb_cart_vel_var = this->associated_robot_->cart_vel_;
    }
}

void ControlObjectiveContainer::addContactSituation(const std::string &csName, const Filter_Struct &fs, const unsigned int selection_direction, std::shared_ptr<ControlObjectiveContainer> gemini_ptr, const COKind kind, const COKind gemini_kind)
{
    this->vec_filter_.push_back(fs);
    this->vec_gemini_.push_back(gemini_ptr);

    this->vec_my_kind_.push_back(kind);

    // this->vec_kind_.push_back(kind);
    this->vec_selection_direction_.push_back(selection_direction);
    this->map_cs_2_idx_[csName] = this->vec_filter_.size() - 1;
    if (gemini_ptr)
    {
        this->vec_receives_gemini_.push_back(gemini_kind);
    }
    else
    {
        this->vec_receives_gemini_.push_back(COKind::None);
    }
    // RTT::log(RTT::Error) << this->name_ << " > Mapping fs " << fs.name << " to  CS " << csName << " with type " << fs.type << ":\n" << fs.data << RTT::endlog();
}

void ControlObjectiveContainer::assignControlComponent(std::shared_ptr<ControlComponentContainer> ctrl)
{
    this->assignedControlComponent_ = ctrl;
}

std::shared_ptr<ControlComponentContainer> ControlObjectiveContainer::getAssignedControlComponent()
{
    return this->assignedControlComponent_;
}

void ControlObjectiveContainer::setMethod(unsigned int m)
{
    assert(m >= 0 && m <= 4);
    switch (m)
    {
    case 0:
        method_ = DLS;
        break;
    case 1:
        method_ = SVD;
        break;
    case 2:
        method_ = QRD;
        break;
    case 3:
        method_ = SVD;
        break;
    case 4:
        method_ = NONE;
        break;
    }
}

void ControlObjectiveContainer::retrievePFromGemini(Eigen::MatrixXd &P)
{
    P = this->projection_gemini_;
}

void ControlObjectiveContainer::calculateProjectionWithDecomposition(const Eigen::MatrixXd &j_as_base_for_P, Eigen::MatrixXd &projection)
{
    switch (method_)
    {
    case NONE:
        svd_solver_jac_c.compute(j_as_base_for_P, Eigen::ComputeFullU | Eigen::ComputeFullV);
        singular_values_jac_c = svd_solver_jac_c.singularValues();
        for (int i = 0; i < singular_values_jac_c.size(); i++)
        {
            if (singular_values_jac_c(i) < 1.e-06)
            {
                singular_values_jac_c(i) = 0;
            }
            else
            {
                singular_values_jac_c(i) = 1 / singular_values_jac_c(i);
            }
        }
        projection = this->Identity_filtered_dof_ - ((svd_solver_jac_c.matrixV().leftCols(singular_values_jac_c.size()) * singular_values_jac_c.asDiagonal() * svd_solver_jac_c.matrixU().leftCols(singular_values_jac_c.size()).transpose()) * j_as_base_for_P);
        break;
    case DLS:
        // matDecomp.computeDLSnullspace(j_as_base_for_P, regfactor, projDLS); //without activation...
        // projection = projDLS;
        break;
    case SVD:
        int out_rankSVD_var;
        matDecomp.computeSVDnullspace(j_as_base_for_P, in_activation_var, projection, out_rankSVD_var, Aauto); //without activation...
        // matDecomp.computeSVDnullspace(j_as_base_for_P, projSVD, out_rankSVD_var, T, S);
        // projection = projSVD;
        break;
    case QRD:
        matDecomp.computeQRDnullspace(j_as_base_for_P, in_activation_var, projection, out_rankQRD_var, Aauto); //with activation...
        // matDecomp.computeQRDnullspace(in_jcOrig_var, in_activation, projQRD_Orig, out_rankQRD_var, Aauto);
        // matDecomp.computeQRnullspace(j_as_base_for_P, in_activation, projQRD, out_rankQRD_var, RdiagonalSorted);
        break;
    case SVDQRD:
        //        matDecomp.computeSVDQRDnullspace(j_as_base_for_P, in_activation, projSVDQRD, out_rankSVD, out_rankQRD);
        //        projection = projSVDQRD;
        break;
    }
}

void ControlObjectiveContainer::triggerGeminiCalculationOncePerRun(const Eigen::MatrixXd &jacobian_filtered, const Eigen::MatrixXd &selection)
{
    Eigen::MatrixXd j_as_base_for_P = jacobian_filtered;

    // RTT::log(RTT::Error) << "DEBUG: my_type_: " << my_type_ << ", currently_receives_gemini_: " << currently_receives_gemini_ << ", IamVM_: " << IamVM_ << RTT::endlog();

    if (this->my_kind_ == COKind::SubspaceFirstOrder)
    {
        j_as_base_for_P = /* RR_world_fdir * */ (selection * (/* RR_world_fdir.inverse() * */ this->jacobian_));
        calculateProjectionWithDecomposition(j_as_base_for_P, this->projection_gemini_);
        this->projection_ = this->Identity_filtered_joint_dof_ - this->projection_gemini_;

        if (this->IamVM_) // && (this->currently_receives_gemini_ == COKind::InternalForce)
        {
            // This case can only occur if this is related to a VM, but to save computation power, I don't test that here!
            // => P_C = P^VM_TS
            this->projection_ = std::static_pointer_cast<VMContainer>(this->associated_robot_)->projection_internal_ * this->projection_;
        }
    }
    else if (this->my_kind_ == COKind::SubspaceSecondOrder)
    {
        if (this->currently_receives_gemini_ == COKind::SubspaceFirstOrder)
        {
            std::shared_ptr<ControlObjectiveContainer> tmp_ptr = this->vec_gemini_[this->current_active_cs_filter_idx_];
            if (tmp_ptr)
            {
                tmp_ptr->retrievePFromGemini(this->projection_);
            }
        }
        else
        {
            j_as_base_for_P = /* RR_world_fdir * */ ((Identity_filtered_dof_ - selection) * (/* RR_world_fdir.inverse() * */ this->jacobian_));
            calculateProjectionWithDecomposition(j_as_base_for_P, this->projection_);
        }

        if (this->IamVM_) // && (this->currently_receives_gemini_ == COKind::InternalForce)
        {
            // This case can only occur if this is related to a VM, but to save computation power, I don't test that here!
            // => P_C = P^VM_TS
            this->projection_ = std::static_pointer_cast<VMContainer>(this->associated_robot_)->projection_internal_ * this->projection_;
        }
    }
    else if (this->IamVM_ && this->opType == OPType::Internal_Force) // this->my_kind_ == COKind::InternalForce
    {
        // => P_IF = P^VM_IF
        this->projection_ = this->Identity_filtered_joint_dof_ - std::static_pointer_cast<VMContainer>(this->associated_robot_)->projection_internal_;
    }
    else if (this->my_kind_ == COKind::None)
    {
        if (this->IamVM_) // && (this->currently_receives_gemini_ == COKind::InternalForce)
        {
            // This case can only occur if this is related to a VM, but to save computation power, I don't test that here!
            // => P_C = P^VM_TS
            this->projection_gemini_.setZero(); // Not necessary, since I don't retrieve projections from None-Types!
            this->projection_ = std::static_pointer_cast<VMContainer>(this->associated_robot_)->projection_internal_;
        }
        else
        {
            this->projection_gemini_.setZero(); // Not necessary, since I don't retrieve projections from None-Types!
            this->projection_.setIdentity();
        }
    }

    //
    // if (this->my_kind_ == COKind::SubspaceFirstOrder)
    // {
    //     if (this->currently_receives_gemini_ == COKind::None)
    //     {
    //         RTT::log(RTT::Error) << "DEBUG: my_kind_: Unconstraint, currently_receives_gemini_: None, IamVM_: " << IamVM_ << RTT::endlog();
    //         // In this case there is no difference if this CO is related to a VM or not.

    //         // => P_m = Identity (depending on filtering)
    //         // this->projection_.setIdentity();

    //         // j_as_base_for_P = /* RR_world_fdir * */ ((Identity_filtered_dof_ - selection) * (/* RR_world_fdir.inverse() * */ this->jacobian_));
    //         j_as_base_for_P = /* RR_world_fdir * */ (selection * (/* RR_world_fdir.inverse() * */ this->jacobian_));
    //         calculateProjectionWithDecomposition(j_as_base_for_P, this->projection_);
    //         this->projection_ = this->Identity_filtered_joint_dof_ - this->projection_;
    //     }
    //     else if (this->IamVM_)
    //     {
    //         if (this->currently_receives_gemini_ == COKind::InternalForce)
    //         {
    //             RTT::log(RTT::Error) << "DEBUG: my_kind_: Unconstraint, currently_receives_gemini_: InternalForce, IamVM_(true): " << IamVM_ << RTT::endlog();
    //             // => P_M = P^VM_TS
    //             this->projection_ = std::static_pointer_cast<VMContainer>(this->associated_robot_)->projection_internal_;
    //         }
    //         else if (this->currently_receives_gemini_ == COKind::SubspaceFirstOrder)
    //         {
    //             RTT::log(RTT::Error) << "DEBUG: my_kind_: Unconstraint, currently_receives_gemini_: Constraint, IamVM_(true): " << IamVM_ << RTT::endlog();
    //             // => P_M = P^VM_TS * P^C_M <----------------------------------------- ?????????????? How to realize the chain here?
    //             // Actually this is kinda easy, because I just get the first part from the associated robot and the second one from the gemini..!
    //             std::shared_ptr<ControlObjectiveContainer> tmp_ptr = this->vec_gemini_[this->current_active_cs_filter_idx_];
    //             if (tmp_ptr)
    //             {
    //                 tmp_ptr->retrievePFromGemini(this->projection_);
    //             }
    //             this->projection_ = std::static_pointer_cast<VMContainer>(this->associated_robot_)->projection_internal_ * this->projection_;
    //         }
    //         // else { // This is not possible to happen! }
    //     }
    //     else if (this->currently_receives_gemini_ == COKind::SubspaceFirstOrder)
    //     {
    //         RTT::log(RTT::Error) << "DEBUG: my_kind_: Unconstraint, currently_receives_gemini_: Constraint, IamVM_(false): " << IamVM_ << RTT::endlog();
    //         // => P_M = P^C_M

    //         // retrieve it from the gemini which needs to be calculated first
    //         std::shared_ptr<ControlObjectiveContainer> tmp_ptr = this->vec_gemini_[this->current_active_cs_filter_idx_];
    //         if (tmp_ptr)
    //         {
    //             tmp_ptr->retrievePFromGemini(this->projection_);
    //         }
    //     }
    //     // else { // This is not possible to happen! }
    // }
    // else if (this->my_kind_ == COKind::SubspaceFirstOrder)
    // {
    //     if (this->currently_receives_gemini_ == COKind::None)
    //     {
    //         RTT::log(RTT::Error) << "DEBUG: my_kind_: Constraint, currently_receives_gemini_: None, IamVM_: " << IamVM_ << RTT::endlog();
    //         // => P_C = Identity - P_M

    //         // TODO  1- ??
    //         j_as_base_for_P = jacobian_filtered;
    //         calculateProjectionWithDecomposition(j_as_base_for_P, this->projection_);
    //     }
    //     else if (this->currently_receives_gemini_ == COKind::InternalForce)
    //     {
    //         RTT::log(RTT::Error) << "DEBUG: my_kind_: Constraint, currently_receives_gemini_: InternalForce, IamVM_: " << IamVM_ << RTT::endlog();
    //         // This case can only occur if this is related to a VM, but to save computation power, I don't test that here!

    //         // => P_C = P^VM_TS
    //         this->projection_ = std::static_pointer_cast<VMContainer>(this->associated_robot_)->projection_internal_;
    //     }
    //     // else { // This is not possible to happen! }
    // }
    // else if (this->my_kind_ == COKind::InternalForce)
    // {
    //     RTT::log(RTT::Error) << "DEBUG: my_kind_: InternalForce, currently_receives_gemini_: None?, IamVM_(true): " << IamVM_ << RTT::endlog();
    //     // Then it is only possible that it is also a VM, but to save computational power, we do not check this here!

    //     // => P_IF = P^VM_IF
    //     this->projection_ = this->Identity_filtered_joint_dof_ - std::static_pointer_cast<VMContainer>(this->associated_robot_)->projection_internal_;

    //     // TODO I don't know what to do here?!
    // }
    // // else { // This is not possible to happen! }

    // ################################## OLD ##################################

    // if (this->currently_receives_gemini_ == COKind::Constraint)
    // {
    //     // retrive it from the gemini which needs to be calculated first
    //     std::shared_ptr<ControlObjectiveContainer> tmp_ptr = this->vec_gemini_[this->current_active_cs_filter_idx_];
    //     if (tmp_ptr)
    //     {
    //         tmp_ptr->retrievePFromGemini(this->projection_);
    //     }
    //     // TODO do we need an else here?
    // }
    // else if (this->currently_receives_gemini_ == COKind::InternalForce && this->IamVM_)
    // {
    //     this->projection_ = std::static_pointer_cast<VMContainer>(this->associated_robot_)->projection_internal_;
    // }
    // else
    // {
    //     if (cokind == COKind::M)
    //     {
    //         // Here I have to invert the filter before using it on the jacobian
    //         j_as_base_for_P = /* RR_world_fdir * */ ((Identity_filtered_dof_ - selection) * (/* RR_world_fdir.inverse() * */ this->jacobian_));
    //         // TODO DLW, sollte nicht hier mein P = Ones sein?
    //     }
    //     // else if (cokind == COKind::C)
    //     // {
    //     //     // Here I have to proceed normally!
    //     // }

    //     calculateProjectionWithDecomposition(j_as_base_for_P, this->projection_);
    // }

    // NOTE! I am interested in the gemini of this->projection_ /////////////////
    if (this->projection_.isIdentity())
    {
        RTT::log(RTT::Error) << "Projection is identity??? j_as_base_for_P =\n"
                             << j_as_base_for_P << "\nin_activation_var =\n"
                             << in_activation_var << RTT::endlog();
    }

    // if (cokind == COKind::C && this->currently_receives_gemini_ == COKind::None)
    // {
    //     this->projection_ = this->Identity_filtered_joint_dof_ - this->projection_;
    // }
    // else if (cokind == COKind::IF && this->currently_receives_gemini_ == COKind::None)
    // {
    //     this->projection_ = this->Identity_filtered_joint_dof_ - this->projection_;
    // }

    // Calculate dP = ( P(t) - P(t-1) ) / dt
    double end_time = (1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks()));

    double dt = end_time - start_time;
    if (previousP_.isZero())
    {
        start_time = (1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks()));
        previousP_ = this->projection_;
    }
    else if (dt < 0.0001)
    {
    }
    else
    {
        // time-derivative of projection
        this->projection_dot_ = (this->projection_ - previousP_) / dt;
    }
    start_time = end_time;
    previousP_ = this->projection_;

    // if (this->projection_.isZero())
    // {
    //     RTT::log(RTT::Error) << "Projection 3 is zero??? :(" << RTT::endlog();
    // }

    // Compute constrained inertia (Eq. under Eq. 11 in Valerio's paper)
    this->inertia_filtered_ = this->projection_ * this->inertia_ + this->Identity_filtered_joint_dof_ - this->projection_;
}

unsigned int ControlObjectiveContainer::currentlyNeedsToRetrievePFromGemini()
{
    return this->currently_receives_gemini_;
}

void ControlObjectiveContainer::setCtrlOutputPositions(const unsigned int j_i, const unsigned int j_j, const unsigned int j_p, const unsigned int j_q, const unsigned int m_i, const unsigned int m_j, const unsigned int m_p, const unsigned int m_q, const unsigned int gc_i, const unsigned int gc_p)
{
    jmgc_control_position_.j_i_ = j_i;
    jmgc_control_position_.j_j_ = j_j;
    jmgc_control_position_.j_p_ = j_p;
    jmgc_control_position_.j_q_ = j_q;

    jmgc_control_position_.m_i_ = m_i;
    jmgc_control_position_.m_j_ = m_j;
    jmgc_control_position_.m_p_ = m_p;
    jmgc_control_position_.m_q_ = m_q;

    jmgc_control_position_.gc_i_ = gc_i;
    jmgc_control_position_.gc_p_ = gc_p;

    // set up internal output variables!
    jacobian_filtered_ = Eigen::MatrixXd::Zero(j_p, j_q);
    jacobian_dot_filtered_ = Eigen::MatrixXd::Zero(j_p, j_q);
    inertia_filtered_ = Eigen::MatrixXd::Zero(m_p, m_q);
    gc_filtered_ = Eigen::VectorXd::Zero(gc_p);

    Identity_filtered_dof_ = Eigen::MatrixXd::Identity(j_p, j_p);
    Identity_filtered_joint_dof_ = Eigen::MatrixXd::Identity(m_p, m_q);

    projection_ = Eigen::MatrixXd::Zero(m_p, m_q);
    previousP_ = Eigen::MatrixXd::Zero(m_p, m_q);
    projection_dot_ = Eigen::MatrixXd::Zero(m_p, m_q);
    projection_gemini_ = Eigen::MatrixXd::Zero(m_p, m_q);
    // projection_dot_gemini_ = Eigen::MatrixXd::Zero(m_p, m_q);

    svd_solver_jac_c = Eigen::JacobiSVD<Eigen::MatrixXd>(j_p, j_q);
    singular_values_jac_c.resize(j_p);
    Aauto = Eigen::VectorXd::Zero(j_p);
    out_rankQRD_var = -1;
    in_activation_var = Eigen::VectorXd::Zero(j_p);
    Ones_task_dof_ = Eigen::VectorXd::Ones(j_p);
}

void ControlObjectiveContainer::getDebug_CtrlOutputPositions(unsigned int j_i, unsigned int j_j, unsigned int j_p, unsigned int j_q, unsigned int m_i, unsigned int m_j, unsigned int m_p, unsigned int m_q, unsigned int gc_i, unsigned int gc_p)
{
    j_i = jmgc_control_position_.j_i_;
    j_j = jmgc_control_position_.j_j_;
    j_p = jmgc_control_position_.j_p_;
    j_q = jmgc_control_position_.j_q_;

    m_i = jmgc_control_position_.m_i_;
    m_j = jmgc_control_position_.m_j_;
    m_p = jmgc_control_position_.m_p_;
    m_q = jmgc_control_position_.m_q_;

    gc_i = jmgc_control_position_.gc_i_;
    gc_p = jmgc_control_position_.gc_p_;
}

JMGC_Control_Position_Struct ControlObjectiveContainer::getDebug_CtrlOutputPositions()
{
    return jmgc_control_position_;
}

void ControlObjectiveContainer::setOperationalFrame(const std::string &name, const std::string &type, const unsigned int taskdof, const unsigned int jointdof)
{
    this->opFrame.name = name;
    this->opFrame.type = type;
    if (this->opFrame.type.compare("Link") == 0)
    {
        opType = OPType::Link;
    }
    else if (this->opFrame.type.compare("Frame") == 0)
    {
        opType = OPType::Frame;
    }
    else if (this->opFrame.type.compare("Chain") == 0)
    {
        opType = OPType::Chain;
    }
    else if (this->opFrame.type.compare("Combined_Chain") == 0)
    {
        opType = OPType::Combined_Chain;
    }
    else if (this->opFrame.type.compare("Internal_Force") == 0)
    {
        opType = OPType::Internal_Force;
    }
    this->opFrame.taskdof = taskdof;
    this->opFrame.jointdof = jointdof;
}

Operational_Frame ControlObjectiveContainer::getOperationalFrame()
{
    return this->opFrame;
}

std::string ControlObjectiveContainer::getName()
{
    return this->name_;
}

std::shared_ptr<ManipulatorDataSupplier> ControlObjectiveContainer::getDebug_AssociatedRobot()
{
    return this->associated_robot_;
}