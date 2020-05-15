/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2020 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
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
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Twist.h>

// ROS KDL PARSER includes
#include <kdl_parser/kdl_parser.hpp>

// KDL includes
#include <kdl/tree.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

// BOOST includes
#include <boost/scoped_ptr.hpp>

// EIGEN includes
#include <Eigen/Geometry>
#include <Eigen/Dense>

// LWR MODEL (header) inclues
#include "models/kukaLWRModel.h"

// Generic Kind Dyn Solver Interface includes
#include "kin_dyn_if.hpp"

namespace cosima
{

  class KinDynMultiArm_KDL : public KinDynInterface
  {
  public:
    KinDynMultiArm_KDL(const std::string &modelname);

    // Needs to be called during configuration phase (Non-RT)
    bool setChain(const std::string &chain_root_link_name, const std::string &chain_tip_link_name);
    bool setChainWithWorldOffset(const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const Eigen::VectorXd &worldOffsetTranslation, const Eigen::VectorXd &worldOffsetRotation);
    bool setChainWithWorldOffset(const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const geometry_msgs::Pose &worldOffset);

    // Should be called once everything is set up (RT)
    void compute(
        const sensor_msgs::JointState &in_robotstatus,
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

    void computeStackPart(
        const sensor_msgs::JointState &in_robotstatus,
        const unsigned int &index_js,
        const unsigned int &index_ts,
        const unsigned int &index_ts_quat,
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

    unsigned int getDoF();

  private:
    bool loadModel(const std::string &modelname);
    void initializeVariables();
    void solve(const sensor_msgs::JointState &in_robotstatus);
    bool exists_test(const std::string &name);
    void getSkewSymmetricMatrix(const Eigen::Vector3f &vec, Eigen::Matrix3f &mat);

    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;

    KDL::Vector gravity_vectorKDL;

    boost::shared_ptr<KDL::ChainDynParam> id_dyn_solver;
    boost::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
    boost::shared_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jac_dot_solver;
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_cart_pos_solver;
    boost::shared_ptr<KDL::ChainFkSolverVel_recursive> jnt_to_cart_vel_solver;

    KDL::JntArrayVel q_qd;
    KDL::JntArray g;
    KDL::JntSpaceInertiaMatrix M;
    KDL::JntArray c;
    KDL::Jacobian J;
    KDL::Jacobian Jd;
    KDL::Frame cartPosFrame;
    KDL::FrameVel cartVelFrame;

    // TODO I don't know how to handle this. This is important expert information right here
    // and ONLY for the LWR 4(+) robot!
    kukaLWRModel kukaClosedForm;
    double tempInertia[7][7];
    double tempGravity[7];
    double tempCorilois[7][7];

    // Helper
    Eigen::Vector4d quat_helper;
  };

} // namespace cosima