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

// STL includes
#include <string>

// ROS TYPE includes
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

// EIGEN includes
#include <Eigen/Geometry>
#include <Eigen/Dense>

// Generic Kin Dyn Solver Interface includes
#include "kin_dyn_if.hpp"

// Specific Kin Dyn Solver Interface includes
#ifndef DISABLE_KDL
#include "kin_dyn_kdl.hpp"
#endif

namespace cosima
{
  class KinDynMultiArm
  {
  public:
    KinDynMultiArm();

    // Needs to be called during configuration phase (Non-RT)
    bool addRobotChain(const std::string &solver_type, const std::string &modelname, const std::string &chain_root_link_name, const std::string &chain_tip_link_name);
    bool addRobotChainWithWorldOffset(const std::string &solver_type, const std::string &modelname, const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const Eigen::VectorXd &worldOffsetTranslation, const Eigen::VectorXd &worldOffsetRotation);
    bool addRobotChainWithWorldOffset(const std::string &solver_type, const std::string &modelname, const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const geometry_msgs::Pose &worldOffset);

    void computeAllAsStack(
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
        Eigen::MatrixXd &out_jacobianDot,
        const Eigen::VectorXd &ext_coriolisAndGravity,
        const Eigen::MatrixXd &ext_inertia,
        bool override = false);

    void computeSingle(
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
        Eigen::MatrixXd &out_jacobianDot);

    void initializeRequiredVariables(
        Eigen::MatrixXd &out_inertia,
        Eigen::MatrixXd &out_inertiaInv,
        Eigen::VectorXd &out_gravity,
        Eigen::VectorXd &out_coriolis,
        Eigen::VectorXd &out_coriolisAndGravity,
        Eigen::VectorXd &out_cartPos,
        Eigen::VectorXd &out_cartVel,
        Eigen::VectorXd &out_cartAcc,
        Eigen::MatrixXd &out_jacobian,
        Eigen::MatrixXd &out_jacobianDot);

    const unsigned int getNumberOfRobots();

    const unsigned int getTotalNumberJointDofs();

    const unsigned int getJointDofs(const unsigned int &index);

  private:
    std::vector<std::shared_ptr<KinDynInterface>> vec_kin_dyn_solvers;

    std::vector<std::shared_ptr<sensor_msgs::JointState>> vec_robotstatus_tmp;
  };

} // namespace cosima