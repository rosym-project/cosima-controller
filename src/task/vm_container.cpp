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

#include "../../include/cosima-controller/task/vm_container.hpp"

#include "../../include/cosima-controller/task/vm_geometry_bar.hpp"
#include "../../include/cosima-controller/task/vm_geometry_box.hpp"

#include <fstream>
#include <iostream>

using namespace cosima;
using namespace util;

VMContainer::VMContainer(const std::string &name, const GeometryType gt)
{
    this->name_ = name;
    this->matDecomp = MatrixDecomposition();
    this->matDecomp.epsilon = 1.0e-06;
    this->matDecomp.epsilon = 0.01;
    this->matDecomp.borderA = 0.1;
    this->matDecomp.borderB = 0.01;
    this->matDecomp.changeRows = false;
    this->matDecomp.useAauto = true;
    this->matDecomp.useAuser = true;

    // time_storage_c1.reserve(100000);
    // time_storage_c2.reserve(100000);
    // time_storage_c3.reserve(100000);
    // time_storage_c4.reserve(100000);

    if (gt == GeometryType::BAR)
    {
        geometryObject = std::make_shared<VMGeometryBar>();
    }
    else
    {
        geometryObject = std::make_shared<VMGeometryBox>();
    }
}

std::pair<unsigned int, unsigned int> VMContainer::getTaskAndJointDof()
{
    return std::make_pair(jacobian_.rows(), jacobian_.cols());
}

std::string VMContainer::getRobotName()
{
    return this->name_;
}

void VMContainer::addRealRobot(std::shared_ptr<RobotContainer> rrobot, const unsigned int j_i, const unsigned int j_j, const unsigned int j_p, const unsigned int j_q, const unsigned int m_i, const unsigned int m_j, const unsigned int m_p, const unsigned int m_q, const unsigned int gc_i, const unsigned int gc_p, /* const unsigned int cp_i, const unsigned int cp_p, const unsigned int cv_i, const unsigned int cv_p, */ const unsigned int cart_pos_i)
{
    JMGC_Position_Struct p;
    p.j_i_ = j_i;
    p.j_j_ = j_j;
    p.j_p_ = j_p;
    p.j_q_ = j_q;

    p.m_i_ = m_i;
    p.m_j_ = m_j;
    p.m_p_ = m_p;
    p.m_q_ = m_q;

    p.gc_i_ = gc_i;
    p.gc_p_ = gc_p;

    // p.cp_i_ = cp_i;
    // p.cp_p_ = cp_p;
    // p.cv_i_ = cv_i;
    // p.cv_p_ = cv_p;

    p.cart_pos_i = cart_pos_i;

    jmgc_positions_.push_back(p);
    involvedRealRobots_.push_back(rrobot);
}

void VMContainer::writeDebugTiming()
{
    // std::ofstream myfile;
    // myfile.open ("time_storage_VM_c1.csv");
    // for (unsigned int i = 0; i < time_storage_c1.size(); i++)
    // {
    //     if (i > 0)
    //     {
    //         myfile << ",";
    //     }
    //     myfile << time_storage_c1[i];
    // }
    // myfile.close();

    // myfile.open ("time_storage_VM_c2.csv");
    // for (unsigned int i = 0; i < time_storage_c2.size(); i++)
    // {
    //     if (i > 0)
    //     {
    //         myfile << ",";
    //     }
    //     myfile << time_storage_c2[i];
    // }
    // myfile.close();

    // myfile.open ("time_storage_VM_c3.csv");
    // for (unsigned int i = 0; i < time_storage_c3.size(); i++)
    // {
    //     if (i > 0)
    //     {
    //         myfile << ",";
    //     }
    //     myfile << time_storage_c3[i];
    // }
    // myfile.close();

    // myfile.open ("time_storage_VM_c4.csv");
    // for (unsigned int i = 0; i < time_storage_c4.size(); i++)
    // {
    //     if (i > 0)
    //     {
    //         myfile << ",";
    //     }
    //     myfile << time_storage_c4[i];
    // }
    // myfile.close();
}

void VMContainer::updateJMGForRobot(const unsigned int robot_id, const Eigen::MatrixXd &in_robot_J_var, const Eigen::MatrixXd &in_robot_J_dot_var, const Eigen::MatrixXd &in_robot_M_var, const Eigen::VectorXd &in_robot_GC_var, const Eigen::VectorXd &in_robotstatus_pos, const Eigen::VectorXd &in_robotstatus_vel, const Eigen::VectorXd &in_robotstatus_trq, const Eigen::VectorXd &in_cartPos_var, const Eigen::VectorXd &in_cartVel_var)
{
    JMGC_Position_Struct p = jmgc_positions_[robot_id];

    jacobian.block(p.j_i_, p.j_j_, p.j_p_, p.j_q_) = in_robot_J_var;
    jacobian_dot.block(p.j_i_, p.j_j_, p.j_p_, p.j_q_) = in_robot_J_dot_var;
    inertia_.block(p.m_i_, p.m_j_, p.m_p_, p.m_q_) = in_robot_M_var;
    gc_.segment(p.gc_i_, p.gc_p_) = in_robot_GC_var;

    robotstatus_pos_.segment(p.gc_i_, p.gc_p_) = in_robotstatus_pos;
    robotstatus_vel_.segment(p.gc_i_, p.gc_p_) = in_robotstatus_vel;
    robotstatus_trq_.segment(p.gc_i_, p.gc_p_) = in_robotstatus_trq;

    cart_pos.segment<7>(p.cart_pos_i) = in_cartPos_var;
    cart_vel.segment<6>(p.j_i_) = in_cartVel_var;
}

void VMContainer::initializeDimensions(const unsigned int totalInternalTaskDof, const unsigned int totalInternalJointDof)
{
    jacobian = Eigen::MatrixXd::Zero(totalInternalTaskDof, totalInternalJointDof);
    jacobian_dot = Eigen::MatrixXd::Zero(totalInternalTaskDof, totalInternalJointDof);
    jacobian_ = Eigen::MatrixXd::Zero(totalInternalTaskDof, totalInternalJointDof);
    jacobian_dot_ = Eigen::MatrixXd::Zero(totalInternalTaskDof, totalInternalJointDof);
    jacobian_internal_ = Eigen::MatrixXd::Zero(12, totalInternalJointDof);
    jacobian_internal_dot_ = Eigen::MatrixXd::Zero(12, totalInternalJointDof);
    inertia_ = Eigen::MatrixXd::Zero(totalInternalJointDof, totalInternalJointDof);
    gc_ = Eigen::VectorXd::Zero(totalInternalJointDof);

    projection_internal_ = Eigen::MatrixXd::Zero(totalInternalJointDof, totalInternalJointDof);
    // projection_internal_dot_ = Eigen::MatrixXd::Zero(totalInternalJointDof, totalInternalJointDof);

    cart_pos = Eigen::VectorXd::Zero(7 * 2);
    cart_vel = Eigen::VectorXd::Zero(6 * 2);
    // These are for the VM and need to be published!
    cart_pos_ = Eigen::VectorXd::Zero(7);
    cart_vel_ = Eigen::VectorXd::Zero(6);

    robotstatus_pos_ = Eigen::VectorXd::Zero(totalInternalJointDof);
    robotstatus_vel_ = Eigen::VectorXd::Zero(totalInternalJointDof);
    robotstatus_trq_ = Eigen::VectorXd::Zero(totalInternalJointDof);

    // TODO DLW !!!
    fadeConstraintActivation_ = 1;

    identityCstrDim_ = Eigen::MatrixXd::Identity(totalInternalTaskDof, totalInternalTaskDof);
    PCstrGraspmap_ = Eigen::MatrixXd::Zero(totalInternalTaskDof, totalInternalTaskDof);
    identityTotalJointDof = Eigen::MatrixXd::Identity(totalInternalJointDof, totalInternalJointDof);

    svd_solver_jac_c = Eigen::JacobiSVD<Eigen::MatrixXd>(12, totalInternalJointDof);
    singular_values_jac_c.resize(12);
}

void VMContainer::initializeAssociatedRobotRelations()
{
    graspmap_ = Eigen::MatrixXd::Zero(6, 6 * involvedRealRobots_.size());
    graspmapPinvT_ = Eigen::MatrixXd::Zero(6, 6 * involvedRealRobots_.size());
    incontactstate_ = Eigen::VectorXd::Ones(involvedRealRobots_.size());
    estimated_force_directionEE_ = Eigen::VectorXd::Zero(6 * involvedRealRobots_.size());
    manipulatorTranslation_.clear();
    manipulatorRotation33_.clear();
    for (unsigned int manipulator = 0; manipulator < involvedRealRobots_.size(); manipulator++)
    {
        manipulatorTranslation_.push_back(Eigen::Vector3d::Zero());
        manipulatorRotation33_.push_back(Eigen::Matrix3d::Zero());
    }
    Aauto = Eigen::VectorXd::Zero(6 * involvedRealRobots_.size());
    in_activation_var = Eigen::VectorXd::Ones(6 * involvedRealRobots_.size());

    geometryObject->initialize(involvedRealRobots_.size());
}

void VMContainer::computeDirectionEE(Eigen::VectorXd &out_directionEE)
{
    out_directionEE.setZero();
    Eigen::Vector3d averagEndeffectorPosition = Eigen::Vector3d::Zero();
    unsigned int _numManipulators = involvedRealRobots_.size();
    for (unsigned int manipulator = 0; manipulator < _numManipulators; manipulator++)
    {
        averagEndeffectorPosition += manipulatorTranslation_.at(manipulator);
    }
    averagEndeffectorPosition = averagEndeffectorPosition / _numManipulators;

    for (unsigned int manipulator = 0; manipulator < _numManipulators; manipulator++)
    {
        out_directionEE.segment<3>(6 * manipulator) = averagEndeffectorPosition - manipulatorTranslation_.at(manipulator);
        out_directionEE.segment<3>(6 * manipulator) = out_directionEE.segment<3>(6 * manipulator) / out_directionEE.segment<3>(6 * manipulator).norm();
    }
}

void VMContainer::computeInternalForceProjection(
    const Eigen::MatrixXd &in_jacobian_constraint,
    const Eigen::MatrixXd &in_jacobian_constraint_dot,
    Eigen::MatrixXd &out_projection)
{
    // TODO DLW check the importance of the activation in this context!
    int out_rank_var = -1;
    // RTT::log(RTT::Error) << "]]]]]]]]]]]]]]]> in_jacobian_constraint =\n"
    //                      << in_jacobian_constraint << ", in_activation_var =\n"
    //                      << in_activation_var << ", out_projection =\n"
    //                      << out_projection << ", out_rank_var = " << out_rank_var << ", Aauto =\n"
    //                      << Aauto << RTT::endlog();
    matDecomp.computeSVDnullspace(in_jacobian_constraint, in_activation_var, out_projection, out_rank_var, Aauto); //without activation...

    // matDecomp.computeQRDnullspace(in_jacobian_constraint, in_activation_var, out_projection, out_rank_var, Aauto);

    // svd_solver_jac_c.compute(in_jacobian_constraint, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // singular_values_jac_c = svd_solver_jac_c.singularValues();
    // for (int i = 0; i < singular_values_jac_c.size(); i++)
    // {
    //     if (singular_values_jac_c(i) < 1.e-06)
    //     {
    //         singular_values_jac_c(i) = 0;
    //     }
    //     else
    //     {
    //         singular_values_jac_c(i) = 1 / singular_values_jac_c(i);
    //     }
    // }
    // out_projection = this->identityTotalJointDof - ((svd_solver_jac_c.matrixV().leftCols(singular_values_jac_c.size()) * singular_values_jac_c.asDiagonal() * svd_solver_jac_c.matrixU().leftCols(singular_values_jac_c.size()).transpose()) * in_jacobian_constraint);
}

void VMContainer::compute()
{
    Eigen::VectorXd activation = fadeConstraintActivation_ * incontactstate_;

    // RTT::os::TimeService::nsecs start_c1 = RTT::os::TimeService::Instance()->getNSecs();
    // this->computeBoxPosVelEstimation(cart_pos, cart_vel, activation, this->cart_pos_, this->cart_vel_);
    geometryObject->computeBoxPosVelEstimation(involvedRealRobots_.size(), cart_pos, cart_vel, activation, this->graspmapPinvT_, this->PCstrGraspmap_,
                                               this->manipulatorTranslation_, this->manipulatorRotation33_, this->cart_pos_, this->cart_vel_);

    // RTT::os::TimeService::nsecs end_c1 = RTT::os::TimeService::Instance()->getNSecs(start_c1);
    // if (time_storage_c1.size() < time_storage_c1.capacity())
    // {
    //     time_storage_c1.push_back(1e-6 * end_c1);
    // }

    // RTT::os::TimeService::nsecs start_c2 = RTT::os::TimeService::Instance()->getNSecs();
    this->computeJacobianCstrAndBox(robotstatus_pos_, robotstatus_vel_, robotstatus_trq_, jacobian, jacobian_dot, activation, jacobian_, jacobian_dot_, jacobian_internal_, jacobian_internal_dot_);
    // RTT::os::TimeService::nsecs end_c2 = RTT::os::TimeService::Instance()->getNSecs(start_c2);
    // if (time_storage_c2.size() < time_storage_c2.capacity())
    // {
    //     time_storage_c2.push_back(1e-6 * end_c2);
    // }

    // RTT::os::TimeService::nsecs start_c3 = RTT::os::TimeService::Instance()->getNSecs();
    this->computeDirectionEE(estimated_force_directionEE_);
    // RTT::os::TimeService::nsecs end_c3 = RTT::os::TimeService::Instance()->getNSecs(start_c3);
    // if (time_storage_c3.size() < time_storage_c3.capacity())
    // {
    //     time_storage_c3.push_back(1e-6 * end_c3);
    // }

    // RTT::os::TimeService::nsecs start_c4 = RTT::os::TimeService::Instance()->getNSecs();
    this->computeInternalForceProjection(jacobian_internal_, jacobian_internal_dot_, projection_internal_);
    // RTT::os::TimeService::nsecs end_c4 = RTT::os::TimeService::Instance()->getNSecs(start_c4);
    // if (time_storage_c4.size() < time_storage_c4.capacity())
    // {
    //     time_storage_c4.push_back(1e-6 * end_c4);
    // }

    // After this, the stuff is ready to be read from the control objectives!
}

//////////////////////////////////////////
/**
 * Assumption: when bar is grasped, there is not slipping possible!
 */
bool VMContainer::computeBoxPosVelEstimation(
    Eigen::VectorXd &in_cartPos,
    Eigen::VectorXd &in_cartVel,
    Eigen::VectorXd &in_incontactstate,
    Eigen::VectorXd &out_cartPosTaskBox,
    Eigen::VectorXd &out_cartVelTaskBox)
{
    // WARNING: At the moment we only support 2 manipulators
    if (involvedRealRobots_.size() != 2)
    {
        // TODO DLW
        RTT::log(RTT::Error) << "At the moment we only support 2 manipulators! Skipping. Please handle this properly!" << RTT::endlog();
        return false;
    }

    bool _in_contact_state = true;
    // 1) recompute global position and orientation of manipulators and store them.
    for (unsigned int manipulator = 0; manipulator < involvedRealRobots_.size(); manipulator++)
    {
        manipulatorTranslation_.at(manipulator) = in_cartPos.segment<3>(manipulator * 7);
        // if (manipulatorTranslation_.at(manipulator).hasNaN())
        // {
        //     RTT::log(RTT::Error) << "manipulatorTranslation_.at(manipulator) NAN!=\n"
        //                          << manipulatorTranslation_.at(manipulator) << RTT::endlog();
        // }

        Eigen::Vector4d manipulatorRotationQuaternion = in_cartPos.segment<4>(manipulator * 7 + 3);
        KDL::Rotation manipulatorRotationKDL = KDL::Rotation::Quaternion(manipulatorRotationQuaternion(1), manipulatorRotationQuaternion(2), manipulatorRotationQuaternion(3), manipulatorRotationQuaternion(0));
        this->convertKDLRotation2EigenMatrix(manipulatorRotationKDL, manipulatorRotation33_.at(manipulator));
        if (in_incontactstate(manipulator) == 0)
        {
            _in_contact_state = false;
        }
    }

    if (!_in_contact_state)
    {
        // TODO DLW
        // RTT::log(RTT::Error) << "Not all manipulators in contact yet!" << RTT::endlog();
        return false;
    }

    // // Based on the slipping norm, decide if the manipulator really is in contact...
    // slippingNorm = xdot_c.segment<3>(manipulator * 6).norm() + xdot_c.segment<3>(manipulator * 6 + 3).norm();

    // 2) Calculate the midpoint of the two manipulators
    objectTranslation_ = 0.5 * (manipulatorTranslation_.at(0) + manipulatorTranslation_.at(1));

    // Eigen::Quaternionf rot;
    // rot.setFromTwoVectors(manipulatorTranslation_.at(0), manipulatorTranslation_.at(1));
    // boxRotation33 = rot.toRotationMatrix(); //manipulatorRotation33_.at(tightManipulatorID) * relativeFinger2ObjectRotation33.at(tightManipulatorID);

    // double angleBar = manipulatorTranslation_.at(0).dot(manipulatorTranslation_.at(1)) / (manipulatorTranslation_.at(0).norm() * manipulatorTranslation_.at(1).norm());
    // Eigen::AngleAxisd axAngleBar = Eigen::AngleAxisd(angleBar, manipulatorTranslation_.at(0).cross(manipulatorTranslation_.at(1)));
    // // Eigen::Quaternionf quatfTmp = Eigen::Quaternionf(axAngle.toRotationMatrix());
    // // boxRotation33 = quatfTmp.toRotationMatrix();
    // boxRotation33 = axAngleBar.toRotationMatrix();

    Eigen::Vector3d diffBoxT = manipulatorTranslation_.at(0) - manipulatorTranslation_.at(1);
    Eigen::Vector3d unitY = Eigen::Vector3d::Zero();
    unitY(0) = 0;
    unitY(1) = 1;
    unitY(2) = 0;

    Eigen::Vector3d crossT = diffBoxT.cross(unitY);
    Eigen::Vector3d crossTNormalized = crossT * (1 / crossT.norm());

    double dotAngle = diffBoxT.dot(unitY);
    double dotAngleSqrt = dotAngle / sqrt(diffBoxT(0) * diffBoxT(0) + diffBoxT(1) * diffBoxT(1) + diffBoxT(2) * diffBoxT(2));
    double dotAngleSqrtAcos = -acos(dotAngleSqrt);

    // RTT::log(RTT::Error) << "dotAngle = " << dotAngle << RTT::endlog();
    // RTT::log(RTT::Error) << "dotAngleSqrt = " << dotAngleSqrt << RTT::endlog();
    // RTT::log(RTT::Error) << "dotAngleSqrtAcos = " << dotAngleSqrtAcos << RTT::endlog();

    Eigen::AngleAxisd axAngleBar = Eigen::AngleAxisd(dotAngleSqrtAcos, crossTNormalized);
    // RTT::log(RTT::Error) << "axAngleBar =\n"
    //                      << axAngleBar << RTT::endlog();

    Eigen::Matrix<double, 3, 3> boxRotation33 = axAngleBar.toRotationMatrix();

    // if (boxRotation33.hasNaN())
    // {
    //     RTT::log(RTT::Error) << "boxRotation33 NAN!=\n"
    //                          << boxRotation33 << RTT::endlog();
    // }

    KDL::Rotation boxRotationKDL;
    this->convertEigenMatrix2KDLRotation(boxRotation33, boxRotationKDL);
    double x, y, z, w;
    boxRotationKDL.GetQuaternion(x, y, z, w);
    // boxRotationQuaternion(0) = w;
    out_cartPosTaskBox(3) = w;
    out_cartPosTaskBox(4) = x;
    out_cartPosTaskBox(5) = y;
    out_cartPosTaskBox(6) = z;
    // TODO DLW calculate the Rotation for the endeffs based on the position of the arms (or on the estimated bar, but arm is better because not really estimated)!
    // That should work since the orientation is not used for the bar position and thus they do not influence each other@

    // Update relative transformations between manipulator and object
    // for (int manipulator = 0; manipulator < numManipulators; manipulator++)
    // {
    // if (manipulator != tightManipulatorID) // Alle anderen auf denen die vorherigen Berechnungen nicht beruhen.
    // {
    // relativeFinger2ObjectTranslation_.at(manipulator) = objectTranslation_ - manipulatorTranslation_.at(manipulator);
    // relativeFinger2ObjectRotation33.at(manipulator) = manipulatorRotation33_.at(manipulator).inverse() * boxRotation33;
    // }
    // }
    // TODO not sure if this is still right...?
    // Here I try to calculate the velocity based on the manipulator 0 and its cartesian velocity which is related to the virtual object.
    // out_cartVelTaskBox = graspmapPinvT.block<6, 6>(0, 0 /* tightManipulatorID */ * 6) * in_cartVel.segment<6>(0 /* tightManipulatorID */ * 6);
    // Instead why not use all the manipulators velocities? I really do not know, perhaps we experience interferences this way? Let's find out, shall we?!
    out_cartVelTaskBox = graspmapPinvT_ * in_cartVel;
    // if (out_cartVelTaskBox.hasNaN())
    // {
    //     RTT::log(RTT::Error) << "(graspmapPinvT_ * in_cartVel) NAN!=\n"
    //                          << out_cartVelTaskBox << RTT::endlog();
    // }
    // or just set it zero...
    // boxVelocity.setZero();
    out_cartPosTaskBox.head<3>() = objectTranslation_;
    // if (objectTranslation_.hasNaN())
    // {
    //     RTT::log(RTT::Error) << "objectTranslation_ NAN!=\n"
    //                          << objectTranslation_ << RTT::endlog();
    // }
    // out_cartPosTaskBox.tail<4>() = boxRotationQuaternion;

    if (out_cartPosTaskBox.hasNaN())
    {
        // RTT::log(RTT::Error) << "out_cartPosTaskBox NAN!=\n"
        //                      << out_cartPosTaskBox << RTT::endlog();
        out_cartPosTaskBox.setZero();
    }

    if (out_cartVelTaskBox.hasNaN())
    {
        // RTT::log(RTT::Error) << "out_cartVelTaskBox NAN!=\n"
        //                      << out_cartVelTaskBox << RTT::endlog();
        out_cartVelTaskBox.setZero();
    }

    return true;
}

void VMContainer::computeJacobianCstrAndBox(
    const Eigen::VectorXd &in_robotstatus_pos,
    const Eigen::VectorXd &in_robotstatus_vel,
    const Eigen::VectorXd &in_robotstatus_trq,
    const Eigen::MatrixXd &in_jacobian,
    const Eigen::MatrixXd &in_jacobianDot,
    Eigen::VectorXd &in_incontactstate,
    Eigen::MatrixXd &out_jacobianTaskObject,
    Eigen::MatrixXd &out_jacobianDotTaskObject,
    Eigen::MatrixXd &out_jacobianInternalObject,
    Eigen::MatrixXd &out_jacobianDotInternalObject)
{
    // 1) Create the grasp map composed of the manipulator and object properties.
    for (unsigned int manipulator = 0; manipulator < involvedRealRobots_.size(); manipulator++)
    {
        // create partial grasp matrix for manipulators.
        this->getSkewSymmetricMatrix(manipulatorTranslation_.at(manipulator) - objectTranslation_, skewmat_);
        Eigen::Matrix<double, 6, 6> graspmatrixManipulator = Eigen::Matrix<double, 6, 6>::Identity();
        // graspmatrixManipulator.setIdentity();
        graspmatrixManipulator.bottomLeftCorner<3, 3>() = skewmat_;

        // Create grasp map based on the manipulators that are in contact state based on the input port and not the slip-norm. Why?
        graspmap_.block<6, 6>(0, manipulator * 6) = graspmatrixManipulator * in_incontactstate(manipulator);
    }

    // Handle the case when the grasp map is zero.
    if (graspmap_.isZero())
    {
        graspmapPinvT_.setZero();
    }
    else
    {
        // pseudo inv.
        graspmapPinvT_ = (graspmap_ * graspmap_.transpose()).inverse() * graspmap_;
    }

    // // Handle Activation
    double cstrActivation = 1; //in_constraintActivation_var;
    // if (fadeConstraintActivation_ != 0)
    // {
    //     double timeDelta = (getOrocosTime() - startCstrActTime) / fadeDuration;
    //     cstrActivation = sigmoidHalf(timeDelta, 15);
    //     // use this to adjust the direction of activation and deactivation.
    //     if (fadeConstraintActivation_ < 0)
    //     {
    //         cstrActivation = 1 - cstrActivation;
    //     }
    //     if (timeDelta >= 1)
    //     {
    //         in_constraintActivation_var = cstrActivation;
    //         fadeConstraintActivation_ = 0;
    //         RTT::log(RTT::Error) << "[" << this->getName() << "] constraint activation fade finished!" << RTT::endlog();
    //     }
    // }

    // 2) Additional internal filtering of the internal force space!
    // Eigen::MatrixXd jSave = in_jacobian;
    // Eigen::MatrixXd jdSave = in_jacobianDot;
    // This is how I fully disable the VM to use the following joints.
    // By doing this, the internal force space gets bigger.
    // But in this case I hope that the strong grip on the bar handles the interal forces for me, as well as the nullspace controller!

    // TODO DLW this is only in the case of the bar!
    // jSave.row(3).setZero();
    // jSave.row(4).setZero();
    // jSave.row(5).setZero();
    // jdSave.row(3).setZero();
    // jdSave.row(4).setZero();
    // jdSave.row(5).setZero();
    out_jacobianTaskObject = graspmapPinvT_ * in_jacobian;
    out_jacobianDotTaskObject = graspmapPinvT_ * in_jacobianDot;

    // ######################################################################################################################
    // WARNING: INFORMATION: In the fully scaled case, we have three spaces we need to care about with a virtual manipulator:
    // I) The most important space is normally the internal force space, since we want to maintain the connection that forms this very VM at all costs.
    //    However, if we have a physical connection (e.g., via grasping (form-closure) established, we can normally ignore this space.
    // II) The second most important space is the constraint space that is established through a contact, through which forces can be exerted.
    // III) Last but not least, the normal un-constraint space needs to be handled for motion generation.
    // ######################################################################################################################

    // 3) Compute the null space projection matrix of the grasp map to handle the internal forces!
    Eigen::MatrixXd PTaskGraspmap = graspmap_.transpose() * graspmapPinvT_;
    PCstrGraspmap_ = identityCstrDim_ - PTaskGraspmap;

    // 4) I) Handle the internal space creation
    out_jacobianInternalObject = PCstrGraspmap_ * in_jacobian;
    out_jacobianDotInternalObject = PCstrGraspmap_ * in_jacobianDot;

    // out_jacobianInternalObject.row(3).setZero();
    // out_jacobianInternalObject.row(4).setZero();
    // out_jacobianInternalObject.row(5).setZero();
    // out_jacobianInternalObject.row(9).setZero();
    // out_jacobianInternalObject.row(10).setZero();
    // out_jacobianInternalObject.row(11).setZero();

    // out_jacobianDotInternalObject.row(3).setZero();
    // out_jacobianDotInternalObject.row(4).setZero();
    // out_jacobianDotInternalObject.row(5).setZero();
    // out_jacobianDotInternalObject.row(9).setZero();
    // out_jacobianDotInternalObject.row(10).setZero();
    // out_jacobianDotInternalObject.row(11).setZero();

    // 5) II) Handle the constraint contact situation
    // This is done in the respective control objective!

    // out_jacobianCstrObject = out_jacobianTaskObject;
    // out_jacobianCstrObject.row(0).setZero();
    // out_jacobianCstrObject.row(1).setZero();
    // out_jacobianCstrObject.row(2) = out_jacobianCstrObject.row(2) * cstrActivation;
    // out_jacobianCstrObject.row(3).setZero();
    // out_jacobianCstrObject.row(4).setZero();
    // out_jacobianCstrObject.row(5).setZero();

    // out_jacobianDotCstrObject = out_jacobianDotTaskObject;
    // out_jacobianDotCstrObject.row(0).setZero();
    // out_jacobianDotCstrObject.row(1).setZero();
    // out_jacobianDotCstrObject.row(2) = out_jacobianDotCstrObject.row(2) * cstrActivation;
    // out_jacobianDotCstrObject.row(3).setZero();
    // out_jacobianDotCstrObject.row(4).setZero();
    // out_jacobianDotCstrObject.row(5).setZero();

    // // 6) III) Handle the un-constraint case
    // out_jacobianTaskObject.row(2) = out_jacobianTaskObject.row(2) * (1 - cstrActivation);
    // out_jacobianDotTaskObject.row(2) = out_jacobianDotTaskObject.row(2) * (1 - cstrActivation);

    // added by Josh! to enable or disable parts of the Jacobians based on the in_incontactstate var.
    // for (int i = 0; i < numManipulators; i++)
    // {
    //     out_jacobianCstr.block(i * 6, i * 7, 6, 7) = out_jacobianCstr.block(i * 6, i * 7, 6, 7) * in_incontactstate(i);
    //     out_jacobianDotCstr.block(i * 6, i * 7, 6, 7) = out_jacobianDotCstr.block(i * 6, i * 7, 6, 7) * in_incontactstate(i);
    //     //out_jacobianTaskBox.block(0,i*7,6,7) = out_jacobianTaskBox.block(0,i*7,6,7)* in_incontactstate(i);
    //     //out_jacobianDotTaskBox.block(0,i*7,6,7) = out_jacobianDotTaskBox.block(0,i*7,6,7)* in_incontactstate(i);
    // }
}

double VMContainer::getOrocosTime()
{
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

void VMContainer::convertKDLRotation2EigenMatrix(KDL::Rotation &kdl, Eigen::Matrix<double, 3, 3> &eigen)
{
    eigen(0, 0) = kdl.data[0];
    eigen(0, 1) = kdl.data[1];
    eigen(0, 2) = kdl.data[2];
    eigen(1, 0) = kdl.data[3];
    eigen(1, 1) = kdl.data[4];
    eigen(1, 2) = kdl.data[5];
    eigen(2, 0) = kdl.data[6];
    eigen(2, 1) = kdl.data[7];
    eigen(2, 2) = kdl.data[8];
}

void VMContainer::convertEigenMatrix2KDLRotation(Eigen::Matrix<double, 3, 3> &eigen, KDL::Rotation &kdl)
{
    kdl.data[0] = eigen(0, 0);
    kdl.data[1] = eigen(0, 1);
    kdl.data[2] = eigen(0, 2);
    kdl.data[3] = eigen(1, 0);
    kdl.data[4] = eigen(1, 1);
    kdl.data[5] = eigen(1, 2);
    kdl.data[6] = eigen(2, 0);
    kdl.data[7] = eigen(2, 1);
    kdl.data[8] = eigen(2, 2);
}

void VMContainer::getSkewSymmetricMatrix(Eigen::Vector3d const &vec, Eigen::Matrix3d &mat)
{
    mat.setZero();
    mat(0, 0) = 0;
    mat(0, 1) = -vec(2);
    mat(0, 2) = vec(1);
    mat(1, 0) = vec(2);
    mat(1, 1) = 0;
    mat(1, 2) = -vec(0);
    mat(2, 0) = -vec(1);
    mat(2, 1) = vec(0);
    mat(2, 2) = 0;
}

//////////////////////////////////////////

void VMContainer::getDebug_ReadJacobianFromGlobalPort(const unsigned int robot_idx, unsigned int &j_i, unsigned int &j_j, unsigned int &j_p, unsigned int &j_q)
{
    JMGC_Position_Struct p = jmgc_positions_[robot_idx];
    j_i = p.j_i_;
    j_j = p.j_j_;
    j_p = p.j_p_;
    j_q = p.j_q_;
}

void VMContainer::getDebug_ReadInertiaFromGlobalPort(const unsigned int robot_idx, unsigned int &m_i, unsigned int &m_j, unsigned int &m_p, unsigned int &m_q)
{
    JMGC_Position_Struct p = jmgc_positions_[robot_idx];
    m_i = p.m_i_;
    m_j = p.m_j_;
    m_p = p.m_p_;
    m_q = p.m_q_;
}

void VMContainer::getDebug_ReadGCFromGlobalPort(const unsigned int robot_idx, unsigned int &gc_i, unsigned int &gc_p)
{
    JMGC_Position_Struct p = jmgc_positions_[robot_idx];
    gc_i = p.gc_i_;
    gc_p = p.gc_p_;
}