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
#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

// ROS TYPE includes
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

// EIGEN includes
#include <Eigen/Geometry>
#include <Eigen/Dense>

// Generic Kin Dyn Solver Interface includes
#include "kin_dyn_multi_arm.hpp"

namespace cosima
{

  class RTTKinDynMultiArm : public RTT::TaskContext
  {
  public:
    RTTKinDynMultiArm(std::string const &name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    // void addCubeObject(float mass, float inertia);
    // void addObjectChain(float mass, Eigen::Vector3f cog, float Ixx, float Iyy, float Izz, float Ixy, float Ixz, float Iyz);
    void preparePorts();

    // Needs to be called during configuration phase (Non-RT)
    bool addChain(const std::string &solver_type, const std::string &modelname, const std::string &chain_root_link_name, const std::string &chain_tip_link_name);
    bool addChainWithWorldOffset_to(const std::string &solver_type, const std::string &modelname, const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const Eigen::VectorXd &worldOffsetTranslation, const Eigen::VectorXd &worldOffsetRotation);
    bool addChainWithWorldOffset(const std::string &solver_type, const std::string &modelname, const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const geometry_msgs::Pose &worldOffset);

  private:
    KinDynMultiArm solver_manager;
    unsigned int num_robots;

    // Declare input ports and their datatypes

    // include gravity port to receive it from the defined chains
    std::vector<std::unique_ptr<RTT::InputPort<sensor_msgs::JointState>>> in_robotstatus_ports;
    std::vector<sensor_msgs::JointState> in_robotstatus_vars;
    std::vector<RTT::FlowStatus> in_robotstatus_flows;
    std::vector<std::unique_ptr<RTT::InputPort<Eigen::VectorXd>>> in_external_gravity_ports;
    std::vector<Eigen::VectorXd> in_external_gravity_vars;
    std::vector<RTT::FlowStatus> in_external_gravity_flows;
    std::vector<std::unique_ptr<RTT::InputPort<Eigen::MatrixXd>>> in_inertia_ports;
    std::vector<Eigen::MatrixXd> in_inertia_vars;
    std::vector<RTT::FlowStatus> in_inertia_flows;

    // Declare output ports and their datatypes
    RTT::OutputPort<sensor_msgs::JointState> out_robotstatus_port;
    RTT::OutputPort<Eigen::MatrixXd> out_inertia_port;
    RTT::OutputPort<Eigen::MatrixXd> out_inertiaInv_port;
    RTT::OutputPort<Eigen::VectorXd> out_gravity_port;
    RTT::OutputPort<Eigen::VectorXd> out_coriolis_port;
    RTT::OutputPort<Eigen::VectorXd> out_coriolisAndGravity_port;
    RTT::OutputPort<Eigen::VectorXd> out_cartPos_port;
    RTT::OutputPort<Eigen::VectorXd> out_cartVel_port;
    RTT::OutputPort<Eigen::VectorXd> out_cartAcc_port;
    RTT::OutputPort<Eigen::MatrixXd> out_jacobian_port;
    RTT::OutputPort<Eigen::MatrixXd> out_jacobianDot_port;

    // Data flow:

    // variables
    sensor_msgs::JointState in_robotstatus_var_stacked;
    Eigen::VectorXd in_external_gravity_var_stacked;
    Eigen::MatrixXd in_inertia_var_stacked;
    sensor_msgs::JointState out_robotstatus_var;
    Eigen::MatrixXd out_inertia_var;
    Eigen::MatrixXd out_inertiaInv_var;
    Eigen::VectorXd out_gravity_var;
    Eigen::VectorXd out_coriolis_var;
    Eigen::VectorXd out_coriolisAndGravity_var;
    Eigen::VectorXd out_cartPos_var;
    Eigen::VectorXd out_cartVel_var;
    Eigen::VectorXd out_cartAcc_var;
    Eigen::MatrixXd out_jacobian_var;
    Eigen::MatrixXd out_jacobianDot_var;

    // unsigned int DOFsize, DOFsizeActive;
    bool portsArePrepared;
    // unsigned int numRobotArms, numObjects;
    // std::vector<Eigen::Matrix3f> objectInertia;
    // std::vector<float> objectMass;
    // Eigen::MatrixXf identity66, zero66;

    // bool include_gravity;
    // std::string DynamicsType;

    std::vector<std::unique_ptr<RTT::OutputPort<geometry_msgs::Pose>>> out_cartPos_ports;
    std::vector<std::unique_ptr<RTT::OutputPort<geometry_msgs::Twist>>> out_cartVel_ports;

    std::vector<geometry_msgs::Pose> out_cartPos_vars;
    std::vector<geometry_msgs::Twist> out_cartVel_vars;

  };

} // namespace cosima
