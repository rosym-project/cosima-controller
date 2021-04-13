/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2018 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
 *                    based on Niels Dehio's,
 *                             Sina Mirrazavi's,
 *                             Joshua Smith's,
 *                             Hsiu-Chin Lin's implementations
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

// ROS TYPE includes
#include <sensor_msgs/JointState.h>

// KDL includes
#include <kdl/tree.hpp>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>

#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>

// header for introspection
#include "../introspection/rtt-introspection-base.hpp"

namespace cosima
{

  namespace controller
  {
    class RTTCartPIDController : public cogimon::RTTIntrospectionBase
    {
    public:
      RTTCartPIDController(std::string const &name);

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

      bool loadYAMLConfig(const std::string &myname, const std::string &file);

      void compute(
          Eigen::VectorXd &in_desiredTaskSpacePosition,
          Eigen::VectorXd &in_desiredTaskSpaceVelocity,
          Eigen::VectorXd &in_desiredTaskSpaceAcceleration,
          Eigen::VectorXd &in_currentTaskSpacePosition,
          Eigen::VectorXd &in_currentTaskSpaceVelocity,
          sensor_msgs::JointState &in_robotstatus,
          Eigen::MatrixXd &in_jacobian,
          Eigen::MatrixXd &in_jacobianDot,
          Eigen::VectorXd &in_h,
          Eigen::MatrixXd &in_inertiaInv,
          Eigen::MatrixXd &in_projection,
          Eigen::MatrixXd &in_projectionDot,
          Eigen::VectorXd &out_torques,
          Eigen::VectorXd &out_force);
      bool setDOFsize(unsigned int DOFsize);

      // Compensation
      void useTSgravitycompensation(bool useTSgravitycompensation);
      void addJSgravitycompensation(bool addJSgravitycompensation);

      // Gains
      bool setGains(double kp, double kd);
      bool setGainsOrientation(double kp, double kd);
      bool setGainsBatch(const Eigen::VectorXd &kps, const Eigen::VectorXd &kds);
      bool setGainsOrientationBatch(const Eigen::VectorXd &kps, const Eigen::VectorXd &kds);
      bool setGainsForEE(double kp, double kd, unsigned int index);
      bool setGainsOrientationForEE(double kp, double kd, unsigned int index);
      bool setGainsSingleForEE(double kp, double kd, unsigned int index, unsigned int dir);
      bool setGainsOrientationSingleForEE(double kp, double kd, unsigned int index, unsigned int dir);

      void computeTranslationError(
          Eigen::Vector3d const &cart_desiredPosition,
          Eigen::Vector3d const &cart_currentPosition,
          Eigen::Vector3d const &desiredVelocity,
          Eigen::Vector3d const &cart_cart_currentVelocity,
          Eigen::Vector3d &cart_errorPosition,
          Eigen::Vector3d &cart_errorVelocity);
      void computeOrientationError(
          Eigen::Vector4d const &quaternion_desired,
          Eigen::Vector4d const &quaternion_current,
          Eigen::Vector3d const &axisangle_desiredVelocity,
          Eigen::Vector3d const &axisangle_currentVelocity,
          Eigen::Vector3d &axisangle_errorPosition,
          Eigen::Vector3d &axisangle_errorVelocity);

      void preparePorts();
      void displayStatus();
      void checkConnections();

      void setNoCommandReceivedBehavior(std::string const &type);

      // void calculateNullspace();

      /////////////////////////////////

      void addRobot(unsigned int task_dof, unsigned int joint_dof);
      void calculateTotalDofs();

    private:
      // void setNS(const Eigen::VectorXd &nsp);

      // Declare input ports and their datatypes
      RTT::InputPort<trajectory_msgs::MultiDOFJointTrajectoryPoint> in_desiredTaskSpace_port;

      // RTT::InputPort<geometry_msgs::Pose> in_desiredTaskSpacePosition_port;
      // RTT::InputPort<geometry_msgs::Twist> in_desiredTaskSpaceVelocity_port;
      // RTT::InputPort<geometry_msgs::Accel> in_desiredTaskSpaceAcceleration_port;
      // RTT::InputPort<Eigen::VectorXd> in_desiredTaskSpacePosition_port;
      // RTT::InputPort<Eigen::VectorXd> in_desiredTaskSpaceVelocity_port;
      // RTT::InputPort<Eigen::VectorXd> in_desiredTaskSpaceAcceleration_port;

      RTT::InputPort<Eigen::VectorXd> in_currentTaskSpacePosition_port;
      RTT::InputPort<Eigen::VectorXd> in_currentTaskSpaceVelocity_port;

      RTT::InputPort<sensor_msgs::JointState> in_robotstatus_port;

      RTT::InputPort<Eigen::MatrixXd> in_jacobian_port;
      RTT::InputPort<Eigen::MatrixXd> in_jacobianDot_port;
      RTT::InputPort<Eigen::VectorXd> in_coriolisAndGravity_port;
      RTT::InputPort<Eigen::MatrixXd> in_inertia_port;
      RTT::InputPort<Eigen::MatrixXd> in_projection_port;
      RTT::InputPort<Eigen::MatrixXd> in_projectionDot_port;

      // Declare output ports and their datatypes
      RTT::OutputPort<Eigen::VectorXd> out_torques_port;
      RTT::OutputPort<Eigen::VectorXd> out_force_port;
      RTT::OutputPort<Eigen::VectorXd> out_error_position_port;
      RTT::OutputPort<Eigen::VectorXd> out_error_velocity_port;

      // Data flow:
      RTT::FlowStatus in_desiredTaskSpacePosition_flow;
      RTT::FlowStatus in_desiredTaskSpaceVelocity_flow;
      RTT::FlowStatus in_desiredTaskSpaceAcceleration_flow;
      RTT::FlowStatus in_currentTaskSpacePosition_flow;
      RTT::FlowStatus in_currentTaskSpaceVelocity_flow;
      RTT::FlowStatus in_robotstatus_flow;
      RTT::FlowStatus in_jacobian_flow;
      RTT::FlowStatus in_jacobianDot_flow;
      RTT::FlowStatus in_coriolisAndGravity_flow;
      RTT::FlowStatus in_inertia_flow;
      RTT::FlowStatus in_projection_flow;
      RTT::FlowStatus in_projectionDot_flow;

      // geometry_msgs::Pose in_desiredTaskSpacePosition_var;
      // geometry_msgs::Twist in_desiredTaskSpaceVelocity_var;
      // geometry_msgs::Accel in_desiredTaskSpaceAcceleration_var;

      trajectory_msgs::MultiDOFJointTrajectoryPoint in_desiredTaskSpace_var;

      // Eigen::VectorXd in_desiredTaskSpacePosition_var;
      // Eigen::VectorXd in_desiredTaskSpaceVelocity_var;
      // Eigen::VectorXd in_desiredTaskSpaceAcceleration_var;
      Eigen::VectorXd in_desiredTaskSpacePosition_var_eig;
      Eigen::VectorXd in_desiredTaskSpaceVelocity_var_eig;
      Eigen::VectorXd in_desiredTaskSpaceAcceleration_var_eig;
      //
      Eigen::VectorXd in_currentTaskSpacePosition_var;
      Eigen::VectorXd in_currentTaskSpaceVelocity_var;
      sensor_msgs::JointState in_robotstatus_var;
      Eigen::MatrixXd in_jacobian_var;
      Eigen::MatrixXd in_jacobianDot_var;
      Eigen::VectorXd in_coriolisAndGravity_var;
      Eigen::MatrixXd in_inertia_var;
      Eigen::MatrixXd in_projection_var;
      Eigen::MatrixXd in_projectionDot_var;
      Eigen::VectorXd out_torques_var;
      Eigen::VectorXd out_force_var;

      Eigen::Vector3d errorTranslationPosition, errorTranslationVelocity;
      Eigen::Vector3d errorOrientationPosition, errorOrientationVelocity;
      Eigen::VectorXd errorPosition, errorVelocity;
      Eigen::Vector3d desiredPosition, currentPosition, desiredVelocity, currentVelocity;
      Eigen::Vector4d desiredQuaternionPosition, currentQuaternionPosition;
      bool use_TSgravitycompensation;
      bool add_JSgravitycompensation;
      bool portsArePrepared;
      bool impedanceCTRL;
      Eigen::MatrixXd inertiaInvP;

      // int nullspaceMethod;
      // rstrt::kinematics::JointAngles nullspace_configuration_var;
      // Eigen::MatrixXd nullspace;

      bool hold_current_position, hold_current_position_last;

      int noCommandReceivedBehaviorType;

      bool lockOrientation;

      std::vector<Eigen::MatrixXd> lambda_;
      std::vector<Eigen::JacobiSVD<Eigen::MatrixXd>> svd_solver_lambda_c_;
      std::vector<Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType> singular_values_lambda_c_;

      Eigen::MatrixXd lambda_global_;
      Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver_lambda_c_global_;
      Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singular_values_lambda_c_global_;

      // Adaption for generic control
      // unsigned int numEndEffectors, numObjects;
      unsigned int WorkspaceDimension, WorkspaceQuaternionDimension;
      unsigned int TaskSpaceDimension, TaskSpaceQuaternionDimension;
      std::vector<Eigen::VectorXd> gainTranslationP_, gainTranslationD_, gainOrientationP_, gainOrientationD_;
      Eigen::VectorXd in_desiredTaskSpacePosition_proxy;

      //////////////////////////////////////
      std::vector<std::pair<unsigned int, unsigned int>> vec_dimensions_;
      unsigned int total_dof_size_;
    };

  } // namespace controller

} // namespace cosima