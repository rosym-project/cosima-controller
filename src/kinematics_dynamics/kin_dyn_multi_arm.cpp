/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2020 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>,
 *                       Niels Dehio
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

#include "../../include/cosima-controller/kinematics_dynamics/kin_dyn_multi_arm.hpp"

#include <rtt/Logger.hpp>
#define PRELOG(X) (RTT::log(RTT::X) << "[KinDynMultiArm" << "] ")

using namespace cosima;

KinDynMultiArm::KinDynMultiArm()
{
    this->vec_kin_dyn_solvers.clear();
}

const unsigned int KinDynMultiArm::getNumberOfRobots()
{
    return this->vec_kin_dyn_solvers.size();
}

const unsigned int KinDynMultiArm::getTotalNumberJointDofs()
{
    unsigned int _total_joint_dof = 0;
    for (unsigned int i = 0; i < this->vec_kin_dyn_solvers.size(); i++)
    {
        _total_joint_dof += this->vec_kin_dyn_solvers[i]->getDoF();
    }
    return _total_joint_dof;
}

const unsigned int KinDynMultiArm::getJointDofs(const unsigned int &index)
{
    return this->vec_kin_dyn_solvers[index]->getDoF();
}

void KinDynMultiArm::initializeRequiredVariables(
    Eigen::MatrixXd &out_inertia,
    Eigen::MatrixXd &out_inertiaInv,
    Eigen::VectorXd &out_gravity,
    Eigen::VectorXd &out_coriolis,
    Eigen::VectorXd &out_coriolisAndGravity,
    Eigen::VectorXd &out_cartPos,
    Eigen::VectorXd &out_cartVel,
    Eigen::VectorXd &out_cartAcc,
    Eigen::MatrixXd &out_jacobian,
    Eigen::MatrixXd &out_jacobianDot)
{
    unsigned int _total_joint_dof = this->getTotalNumberJointDofs();
    unsigned int _total_task_dof = 6 * this->vec_kin_dyn_solvers.size();
    unsigned int _total_task_quat_dof = 7 * this->vec_kin_dyn_solvers.size();

    out_inertia = Eigen::MatrixXd::Zero(_total_joint_dof, _total_joint_dof);
    out_inertiaInv = Eigen::MatrixXd::Zero(_total_joint_dof, _total_joint_dof);
    out_gravity = Eigen::VectorXd::Zero(_total_joint_dof);
    out_coriolis = Eigen::VectorXd::Zero(_total_joint_dof);
    out_coriolisAndGravity = Eigen::VectorXd::Zero(_total_joint_dof);
    out_cartPos = Eigen::VectorXd::Zero(_total_task_quat_dof);
    out_cartVel = Eigen::VectorXd::Zero(_total_task_dof);
    out_cartAcc = Eigen::VectorXd::Zero(_total_task_dof);
    out_jacobian = Eigen::MatrixXd::Zero(_total_task_dof, _total_joint_dof);
    out_jacobianDot = Eigen::MatrixXd::Zero(_total_task_dof, _total_joint_dof);
}

void KinDynMultiArm::computeAllAsStack(
    const sensor_msgs::JointState &in_robotstatus_as_stack,
    Eigen::MatrixXd &out_inertia,
    Eigen::MatrixXd &out_inertiaInv,
    Eigen::VectorXd &out_gravity,
    Eigen::VectorXd &out_coriolis,
    Eigen::VectorXd &out_coriolisAndGravity,
    Eigen::VectorXd &out_cartPos,
    Eigen::VectorXd &out_cartVel,
    Eigen::VectorXd &out_cartAcc,
    Eigen::MatrixXd &out_jacobian,
    Eigen::MatrixXd &out_jacobianDot)
{
    ////////////////////////////////////////////
    ////// RETRIEVE INFORMATION AND STACK //////
    ////////////////////////////////////////////
    unsigned int _accu_joint_num = 0;
    for (unsigned int i = 0; i < this->vec_kin_dyn_solvers.size(); i++)
    {
        unsigned int _task_space_cur = i * 6;
        unsigned int _task_space_quat_cur = i * 7;
        std::shared_ptr<sensor_msgs::JointState> _robotstatus_tmp = this->vec_robotstatus_tmp[i];
        std::shared_ptr<KinDynInterface> _solver = this->vec_kin_dyn_solvers[i];
        for (unsigned int j = 0; j < _solver->getDoF(); j++)
        {
            _robotstatus_tmp->position[j] = in_robotstatus_as_stack.position[j];
            _robotstatus_tmp->velocity[j] = in_robotstatus_as_stack.velocity[j];
            _robotstatus_tmp->effort[j] = in_robotstatus_as_stack.effort[j];
        }

        // TODO ask Jan to help optimize this!!!! ':D
        this->vec_kin_dyn_solvers[i]->computeStackPart(*_robotstatus_tmp.get(),
                                                       _accu_joint_num,
                                                       _task_space_cur,
                                                       _task_space_quat_cur,
                                                       out_inertia,
                                                       out_inertiaInv,
                                                       out_gravity,
                                                       out_coriolis,
                                                       out_coriolisAndGravity,
                                                       out_cartPos,
                                                       out_cartVel,
                                                       out_cartAcc,
                                                       out_jacobian,
                                                       out_jacobianDot);
        _accu_joint_num += _solver->getDoF();
    }
}

void KinDynMultiArm::computeSingle(
    const unsigned int &index,
    const sensor_msgs::JointState &in_robotstatus_as_single,
    Eigen::MatrixXd &out_inertia,
    Eigen::MatrixXd &out_inertiaInv,
    Eigen::VectorXd &out_gravity,
    Eigen::VectorXd &out_coriolis,
    Eigen::VectorXd &out_coriolisAndGravity,
    Eigen::VectorXd &out_cartPos,
    Eigen::VectorXd &out_cartVel,
    Eigen::VectorXd &out_cartAcc,
    Eigen::MatrixXd &out_jacobian,
    Eigen::MatrixXd &out_jacobianDot)
{
    return this->vec_kin_dyn_solvers[index]->compute(in_robotstatus_as_single,
                                                     out_inertia,
                                                     out_inertiaInv,
                                                     out_gravity,
                                                     out_coriolis,
                                                     out_coriolisAndGravity,
                                                     out_cartPos,
                                                     out_cartVel,
                                                     out_cartAcc,
                                                     out_jacobian,
                                                     out_jacobianDot);
}

bool KinDynMultiArm::addRobotChain(const std::string &solver_type, const std::string &modelname, const std::string &chain_root_link_name, const std::string &chain_tip_link_name)
{
    std::shared_ptr<KinDynInterface> _solver = NULL;

#ifndef DISABLE_KDL
    if (solver_type.compare("KDL") == 0)
    {
        _solver = std::shared_ptr<KinDynInterface>(new KinDynMultiArm_KDL(modelname));
        PRELOG(Error) << "Creating solver for solver_type == " << solver_type << RTT::endlog();
    }
#endif
#ifdef USE_LWR_CLOSED_CHAIN
    if (solver_type.compare("LWR_CLOSED_CHAIN") == 0)
    {
        _solver = std::shared_ptr<KinDynInterface>(new KinDynMultiArm_LWR_CLOSED_CHAIN(modelname));
    }
#endif
#ifdef USE_BULLET
    if (solver_type.compare("BULLET") == 0)
    {
        _solver = std::shared_ptr<KinDynInterface>(new KinDynMultiArm_BULLET(modelname));
    }
#endif
#ifdef USE_RBDL
    if (solver_type.compare("RBDL") == 0)
    {
        _solver = std::shared_ptr<KinDynInterface>(new KinDynMultiArm_RBDL(modelname));
    }
#endif

    if (!_solver)
    {
        PRELOG(Error) << "Solver could not be created! Perhaps the solver_type = " << solver_type << " is not found!" << RTT::endlog();
        return false;
    }

    if (!_solver->setChain(chain_root_link_name, chain_tip_link_name))
    {
        PRELOG(Error) << "Chain could not be set for: " << chain_root_link_name << " and " << chain_tip_link_name << RTT::endlog();
        return false;
    }
    std::shared_ptr<sensor_msgs::JointState> _robotstatus_tmp = std::shared_ptr<sensor_msgs::JointState>(new sensor_msgs::JointState());
    _robotstatus_tmp->position.resize(_solver->getDoF());
    _robotstatus_tmp->velocity.resize(_solver->getDoF());
    _robotstatus_tmp->effort.resize(_solver->getDoF());
    for (unsigned int j = 0; j < _solver->getDoF(); j++)
    {
        _robotstatus_tmp->position[j] = 0.0;
        _robotstatus_tmp->velocity[j] = 0.0;
        _robotstatus_tmp->effort[j] = 0.0;
    }
    this->vec_robotstatus_tmp.push_back(_robotstatus_tmp);
    this->vec_kin_dyn_solvers.push_back(_solver);
    return true;
}

bool KinDynMultiArm::addRobotChainWithWorldOffset(const std::string &solver_type, const std::string &modelname, const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const Eigen::VectorXd &worldOffsetTranslation, const Eigen::VectorXd &worldOffsetRotation)
{
    std::shared_ptr<KinDynInterface> _solver = NULL;

#ifndef DISABLE_KDL
    if (solver_type.compare("KDL") == 0)
    {
        _solver = std::shared_ptr<KinDynInterface>(new KinDynMultiArm_KDL(modelname));
    }
#endif
#ifdef USE_LWR_CLOSED_CHAIN
    if (solver_type.compare("LWR_CLOSED_CHAIN") == 0)
    {
        _solver = std::shared_ptr<KinDynInterface>(new KinDynMultiArm_LWR_CLOSED_CHAIN(modelname));
    }
#endif
#ifdef USE_BULLET
    if (solver_type.compare("BULLET") == 0)
    {
        _solver = std::shared_ptr<KinDynInterface>(new KinDynMultiArm_BULLET(modelname));
    }
#endif
#ifdef USE_RBDL
    if (solver_type.compare("RBDL") == 0)
    {
        _solver = std::shared_ptr<KinDynInterface>(new KinDynMultiArm_RBDL(modelname));
    }
#endif
    // Otherwise return with error
    return false;

    if ((!_solver) || (!_solver->setChainWithWorldOffset(chain_root_link_name, chain_tip_link_name, worldOffsetTranslation, worldOffsetRotation)))
    {
        return false;
    }
    std::shared_ptr<sensor_msgs::JointState> _robotstatus_tmp = std::shared_ptr<sensor_msgs::JointState>(new sensor_msgs::JointState());
    _robotstatus_tmp->position.resize(_solver->getDoF());
    _robotstatus_tmp->velocity.resize(_solver->getDoF());
    _robotstatus_tmp->effort.resize(_solver->getDoF());
    for (unsigned int j = 0; j < _solver->getDoF(); j++)
    {
        _robotstatus_tmp->position[j] = 0.0;
        _robotstatus_tmp->velocity[j] = 0.0;
        _robotstatus_tmp->effort[j] = 0.0;
    }
    this->vec_robotstatus_tmp.push_back(_robotstatus_tmp);
    this->vec_kin_dyn_solvers.push_back(_solver);
    return true;
}

bool KinDynMultiArm::addRobotChainWithWorldOffset(const std::string &solver_type, const std::string &modelname, const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const geometry_msgs::Pose &worldOffset)
{
    std::shared_ptr<KinDynInterface> _solver = NULL;

#ifndef DISABLE_KDL
    if (solver_type.compare("KDL") == 0)
    {
        _solver = std::shared_ptr<KinDynInterface>(new KinDynMultiArm_KDL(modelname));
    }
#endif
#ifdef USE_LWR_CLOSED_CHAIN
    if (solver_type.compare("LWR_CLOSED_CHAIN") == 0)
    {
        _solver = std::shared_ptr<KinDynInterface>(new KinDynMultiArm_LWR_CLOSED_CHAIN(modelname));
    }
#endif
#ifdef USE_BULLET
    if (solver_type.compare("BULLET") == 0)
    {
        _solver = std::shared_ptr<KinDynInterface>(new KinDynMultiArm_BULLET(modelname));
    }
#endif
#ifdef USE_RBDL
    if (solver_type.compare("RBDL") == 0)
    {
        _solver = std::shared_ptr<KinDynInterface>(new KinDynMultiArm_RBDL(modelname));
    }
#endif
    // Otherwise return with error
    return false;

    if ((!_solver) || (!_solver->setChainWithWorldOffset(chain_root_link_name, chain_tip_link_name, worldOffset)))
    {
        return false;
    }
    std::shared_ptr<sensor_msgs::JointState> _robotstatus_tmp = std::shared_ptr<sensor_msgs::JointState>(new sensor_msgs::JointState());
    _robotstatus_tmp->position.resize(_solver->getDoF());
    _robotstatus_tmp->velocity.resize(_solver->getDoF());
    _robotstatus_tmp->effort.resize(_solver->getDoF());
    for (unsigned int j = 0; j < _solver->getDoF(); j++)
    {
        _robotstatus_tmp->position[j] = 0.0;
        _robotstatus_tmp->velocity[j] = 0.0;
        _robotstatus_tmp->effort[j] = 0.0;
    }
    this->vec_robotstatus_tmp.push_back(_robotstatus_tmp);
    this->vec_kin_dyn_solvers.push_back(_solver);
    return true;
}
