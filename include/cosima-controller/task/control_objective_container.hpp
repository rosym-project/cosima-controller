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
#include <algorithm>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <map>

// #include <rtt/TaskContext.hpp>

#include "manipulator_data_supplier.hpp"
#include "robot_container.hpp"
#include "control_component_container.hpp"

#include "../util/Pseudoinversion.hpp"
#include "../util/MatrixDecomposition.hpp"

// #include <kdl/frames.hpp>
// #include <kdl/frames_io.hpp>
// #include <rst-rt/geometry/Translation.hpp>
// #include <rst-rt/geometry/Rotation.hpp>
// #include <rst-rt/dynamics/Wrench.hpp>
// #include <rst-rt/robot/JointState.hpp>

// struct JMGC_Position_Struct
// {
//   // std::string robot_name_;
//   unsigned int j_i_, j_j_, j_p_, j_q_;
//   unsigned int m_i_, m_j_, m_p_, m_q_;
//   unsigned int gc_i_, gc_p_;
// };

struct JMGC_Control_Position_Struct
{
  unsigned int j_i_, j_j_, j_p_, j_q_;
  unsigned int m_i_, m_j_, m_p_, m_q_;
  unsigned int gc_i_, gc_p_;
};

struct Operational_Frame
{
  std::string name;
  std::string type;
  unsigned int taskdof;
  unsigned int jointdof;
};

struct Filter_Struct
{
  std::string name;
  std::string type;
  Eigen::MatrixXd data;
};

enum METHOD
{
  DLS,
  SVD,
  QRD,
  SVDQRD,
  NONE
};

enum OPType
{
  Link,
  Frame,
  Chain,
  Combined_Chain,
  Internal_Force
};

// enum COKind
// {
//   None,
//   Unconstraint,
//   Constraint,
//   InternalForce
// };

enum COKind
{
  None,
  SubspaceFirstOrder,
  SubspaceSecondOrder,
  InternalForce
};

class ControlObjectiveContainer
{
public:
  ControlObjectiveContainer(const std::string &name);
  // ~ControlObjectiveContainer();

  // void addJMG(const unsigned int robot_id, const Eigen::MatrixXd &in_robot_J_var, const Eigen::MatrixXd &in_robot_J_dot_var, const Eigen::MatrixXd &in_robot_M_var, const Eigen::VectorXd &in_robot_GC_var);
  void drawJMGFromAssociatedRobot();

  void activateContactSituation(const std::string &csName);
  void applyFilter(const double activation);

  void addContactSituation(const std::string &csName, const Filter_Struct &fs, const unsigned int selection_direction, std::shared_ptr<ControlObjectiveContainer> gemini_ptr, const COKind kind, const COKind gemini_kind);
  // void addContactSituation(const std::string &csName, const Filter_Struct &fs, const bool provides_gemini);
  // void addContactSituationWithProjectionGemini(const std::string &csName, const Filter_Struct &fs, std::shared_ptr<ControlObjectiveContainer> pGemini);

  void outputFilteredJMG(Eigen::MatrixXd &in_ctrl_J_var, Eigen::MatrixXd &in_ctrl_J_dot_var, Eigen::MatrixXd &in_ctrl_M_var, Eigen::VectorXd &in_ctrl_GC_var, Eigen::MatrixXd &in_ctrl_Mc_var, Eigen::MatrixXd &in_ctrl_P_var, Eigen::MatrixXd &in_ctrl_P_dot_var, Eigen::VectorXd &in_ctrl_robotstate_pos, Eigen::VectorXd &in_ctrl_robotstate_vel, Eigen::VectorXd &in_ctrl_robotstate_trq, Eigen::VectorXd &in_int_wrench_var, Eigen::VectorXd &in_out_vm_fdb_cart_pos_var, Eigen::VectorXd &in_out_vm_fdb_cart_vel_var);

  // void addRealRobot(std::shared_ptr<RobotContainer> rrobot, const unsigned int j_i, const unsigned int j_j, const unsigned int j_p, const unsigned int j_q, const unsigned int m_i, const unsigned int m_j, const unsigned int m_p, const unsigned int m_q, const unsigned int gc_i, const unsigned int gc_p);
  // std::vector<std::shared_ptr<RobotContainer>> involvedRealRobots_;

  void setCtrlOutputPositions(const unsigned int j_i, const unsigned int j_j, const unsigned int j_p, const unsigned int j_q, const unsigned int m_i, const unsigned int m_j, const unsigned int m_p, const unsigned int m_q, const unsigned int gc_i, const unsigned int gc_p);

  void assignControlComponent(std::shared_ptr<ControlComponentContainer> ctrl);

  std::shared_ptr<ControlComponentContainer> getAssignedControlComponent();

  void setOperationalFrame(const std::string &name, const std::string &type, const unsigned int taskdof, const unsigned int jointdof);

  Operational_Frame getOperationalFrame();

  void retrievePFromGemini(Eigen::MatrixXd &P);

  void initializeDimensions(const unsigned int totalInternalTaskDof, const unsigned int totalInternalJointDof);

  std::string getName();

  void printDebug();

  void setAssociatedRobot(std::shared_ptr<ManipulatorDataSupplier> associated_robot);

  std::shared_ptr<ManipulatorDataSupplier> getDebug_AssociatedRobot();
  void getDebug_CtrlOutputPositions(unsigned int j_i, unsigned int j_j, unsigned int j_p, unsigned int j_q, unsigned int m_i, unsigned int m_j, unsigned int m_p, unsigned int m_q, unsigned int gc_i, unsigned int gc_p);
  JMGC_Control_Position_Struct getDebug_CtrlOutputPositions();

  // void getGeminiNullspaceProjection(Eigen::MatrixXd &pGemini, Eigen::MatrixXd &pdotGemini);
  // bool isUsingGeminiOfFilterInCS(const std::string &cs, const std::string &filterMname);
  bool containsFilterGemini(const std::string &cs_name, const std::string &filterName, const std::string &identifier);

  // https://stackoverflow.com/questions/3418231/replace-part-of-a-string-with-another-string
  bool replace(std::string &str, const std::string &from, const std::string &to);

  void setIamVM(const bool iamvm);

  void setMethod(unsigned int m);

  unsigned int currentlyNeedsToRetrievePFromGemini();

private:
  void triggerGeminiCalculationOncePerRun(const Eigen::MatrixXd &jacobian_filtered, const Eigen::MatrixXd &selection);
  void calculateProjectionWithDecomposition(const Eigen::MatrixXd &j_as_base_for_P, Eigen::MatrixXd &projection);

  std::string name_;

  std::map<std::string, Eigen::MatrixXd> filters_; // for each cs
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd jacobian_dot_;
  Eigen::MatrixXd inertia_;
  Eigen::VectorXd gc_;
  Eigen::VectorXd robotstate_pos_;
  Eigen::VectorXd robotstate_vel_;
  Eigen::VectorXd robotstate_trq_;

  Eigen::MatrixXd jacobian_filtered_;
  Eigen::MatrixXd jacobian_dot_filtered_;
  Eigen::MatrixXd inertia_filtered_;
  Eigen::VectorXd gc_filtered_;
  Eigen::MatrixXd Identity_filtered_dof_;
  Eigen::MatrixXd Identity_filtered_joint_dof_;
  // Generate
  Eigen::MatrixXd projection_, projection_dot_, previousP_;
  Eigen::MatrixXd projection_gemini_;
  //, projection_dot_gemini_; // Gemini teilweise entfernt!

  std::vector<std::shared_ptr<ControlObjectiveContainer>> vec_gemini_;
  std::vector<COKind> vec_receives_gemini_;
  std::vector<COKind> vec_my_kind_;
  // std::vector<COKind> vec_kind_;
  COKind currently_receives_gemini_;
  COKind my_kind_;

  // std::vector<JMGC_Position_Struct> jmgc_positions_;
  // std::vector<JMGC_Control_Position_Struct> jmgc_control_positions_;
  JMGC_Control_Position_Struct jmgc_control_position_;

  std::shared_ptr<ControlComponentContainer> assignedControlComponent_;

  Operational_Frame opFrame;
  OPType opType;

  std::vector<Filter_Struct> vec_filter_;
  unsigned int current_active_cs_filter_idx_;
  std::map<std::string, unsigned int> map_cs_2_idx_;

  bool IamVM_;

  // Per CS
  // 0 mean that when a = 0 => Identity and a = 1 => S. E.g., :M: would use this.
  // 1 mean that when a = 0 => Zero and a = 1 => S. E.g., :C: would use this.
  std::vector<unsigned int> vec_selection_direction_;
  unsigned int selectionDirection_;

  METHOD method_;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver_jac_c;
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singular_values_jac_c;
  cosima::util::MatrixDecomposition matDecomp;
  Eigen::VectorXd Aauto;
  int out_rankQRD_var;
  Eigen::VectorXd in_activation_var;
  Eigen::VectorXd Ones_task_dof_;
  double start_time;

  std::shared_ptr<ManipulatorDataSupplier> associated_robot_;
};
