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
#include "robot_container.hpp"

#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

#include "../util/MatrixDecomposition.hpp"

#include "vm_geometry.hpp"

struct JMGC_Position_Struct
{
  // std::string robot_name_;
  unsigned int j_i_, j_j_, j_p_, j_q_;
  unsigned int m_i_, m_j_, m_p_, m_q_;
  unsigned int gc_i_, gc_p_;
  // unsigned int cp_i_, cp_p_;
  // unsigned int cv_i_, cv_p_;
  unsigned int cart_pos_i;
};

enum GeometryType
{
  BAR,
  BOX
};

class VMContainer : public ManipulatorDataSupplier
{
public:
  VMContainer(const std::string &name, const GeometryType gt = GeometryType::BAR);
  // ~VMContainer();

  void compute();

  void initializeAssociatedRobotRelations();

  void updateJMGForRobot(const unsigned int robot_id, const Eigen::MatrixXd &in_robot_J_var, const Eigen::MatrixXd &in_robot_J_dot_var, const Eigen::MatrixXd &in_robot_M_var, const Eigen::VectorXd &in_robot_GC_var, const Eigen::VectorXd &in_robotstatus_pos, const Eigen::VectorXd &in_robotstatus_vel, const Eigen::VectorXd &in_robotstatus_trq, const Eigen::VectorXd &in_cartPos_var, const Eigen::VectorXd &in_cartVel_var);
  void addRealRobot(std::shared_ptr<RobotContainer> rrobot, const unsigned int j_i, const unsigned int j_j, const unsigned int j_p, const unsigned int j_q, const unsigned int m_i, const unsigned int m_j, const unsigned int m_p, const unsigned int m_q, const unsigned int gc_i, const unsigned int gc_p, /* const unsigned int cp_i, const unsigned int cp_p, const unsigned int cv_i, const unsigned int cv_p, */ const unsigned int cart_pos_i);
  std::vector<std::shared_ptr<RobotContainer>> involvedRealRobots_;
  void initializeDimensions(const unsigned int totalInternalTaskDof, const unsigned int totalInternalJointDof);
  std::pair<unsigned int, unsigned int> getTaskAndJointDof();

  void getDebug_ReadJacobianFromGlobalPort(const unsigned int robot_idx, unsigned int &j_i, unsigned int &j_j, unsigned int &j_p, unsigned int &j_q);
  void getDebug_ReadInertiaFromGlobalPort(const unsigned int robot_idx, unsigned int &m_i, unsigned int &m_j, unsigned int &m_p, unsigned int &m_q);
  void getDebug_ReadGCFromGlobalPort(const unsigned int robot_idx, unsigned int &gc_i, unsigned int &gc_p);

  // Overridden from ManipulatorDataSupplier
  std::string getRobotName();

  //////////////////////////////////////////

  bool computeBoxPosVelEstimation(
      Eigen::VectorXd &in_cartPos,
      Eigen::VectorXd &in_cartVel,
      Eigen::VectorXd &in_incontactstate,
      Eigen::VectorXd &out_cartPosTaskBox,
      Eigen::VectorXd &out_cartVelTaskBox);

  void computeJacobianCstrAndBox(
      const Eigen::VectorXd &in_robotstatus_pos,
      const Eigen::VectorXd &in_robotstatus_vel,
      const Eigen::VectorXd &in_robotstatus_trq,
      const Eigen::MatrixXd &in_jacobian,
      const Eigen::MatrixXd &in_jacobianDot,
      Eigen::VectorXd &in_incontactstate,
      Eigen::MatrixXd &out_jacobianTaskObject,
      Eigen::MatrixXd &out_jacobianDotTaskObject,
      Eigen::MatrixXd &out_jacobianInternalObject,
      Eigen::MatrixXd &out_jacobianDotInternalObject);

  void computeInternalForceProjection(
      const Eigen::MatrixXd &in_jacobian_constraint,
      const Eigen::MatrixXd &in_jacobian_constraint_dot,
      Eigen::MatrixXd &out_projection);

  // Internal force projections which are independent of the control objectives
  Eigen::MatrixXd projection_internal_;
  // Eigen::MatrixXd projection_internal_dot_;

  void writeDebugTiming();

private:
  std::shared_ptr<VMGeometry> geometryObject;

  std::string name_;
  std::vector<JMGC_Position_Struct> jmgc_positions_;

  Eigen::MatrixXd jacobian, jacobian_dot;
  Eigen::VectorXd cart_pos, cart_vel;

  ///////////////////////////////////////////
  std::vector<Eigen::Vector3d> manipulatorTranslation_;
  std::vector<Eigen::Matrix<double, 3, 3>> manipulatorRotation33_;
  Eigen::Vector3d objectTranslation_;
  Eigen::Matrix3d skewmat_;
  Eigen::MatrixXd graspmap_, graspmapPinvT_;
  // double fadeConstraintActivation_;
  Eigen::MatrixXd identityCstrDim_;
  Eigen::MatrixXd identityTotalJointDof;
  Eigen::VectorXd incontactstate_;

  /**
   * Converts KDL rotation to Eigen rotation matrix.
   */
  void convertKDLRotation2EigenMatrix(KDL::Rotation &kdl, Eigen::Matrix<double, 3, 3> &eigen);

  /**
   * Converts Eigen rotation matrix to KDL rotation.
   */
  void convertEigenMatrix2KDLRotation(Eigen::Matrix<double, 3, 3> &eigen, KDL::Rotation &kdl);

  /**
   * Computes the skew matrix for rotation conversion.
   * 
   * Used inside computeJacobianCstrAndBox.
   */
  void getSkewSymmetricMatrix(Eigen::Vector3d const &vec, Eigen::Matrix3d &mat);

  /**
   * Computes the endeffector force direction, pointing to the weighted average position of all endeffectors 
   */
  void computeDirectionEE(Eigen::VectorXd &out_directionEE);

  double getOrocosTime();

  cosima::util::MatrixDecomposition matDecomp;
  Eigen::VectorXd Aauto;
  Eigen::VectorXd in_activation_var;
  Eigen::MatrixXd PCstrGraspmap_;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver_jac_c;
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singular_values_jac_c;

  // std::vector<double> time_storage_c1;
  // std::vector<double> time_storage_c2;
  // std::vector<double> time_storage_c3;
  // std::vector<double> time_storage_c4;
};
