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

// ROS TYPE includes
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

// EIGEN includes
#include <Eigen/Geometry>
#include <Eigen/Dense>

namespace cosima {

class KinDynInterface
{
public:
    KinDynInterface(const std::string &modelname)
    {
        model_name = modelname;
        // loadModel(model_name);
    }
    // Needs to be called during configuration phase (Non-RT)
    virtual bool setChain(const std::string &chain_root_link_name, const std::string &chain_tip_link_name) = 0;
    virtual bool setChainWithWorldOffset(const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const Eigen::VectorXd &worldOffsetTranslation, const Eigen::VectorXd &worldOffsetRotation) = 0;
    virtual bool setChainWithWorldOffset(const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const geometry_msgs::Pose &worldOffset, const geometry_msgs::Pose &compliance_frame) = 0;

    // Should be called once everything is set up (RT)
    virtual void compute(
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
        Eigen::MatrixXd &out_jacobianDot,
        const Eigen::VectorXd &in_ext_torque,
        Eigen::VectorXd &out_ext_wrench) = 0;

    virtual void computeStackPart(
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
        Eigen::MatrixXd &out_jacobianDot,
        const Eigen::VectorXd &in_ext_torque,
        Eigen::VectorXd &out_ext_wrench,
        const Eigen::VectorXd &ext_coriolisAndGravity,
        const Eigen::MatrixXd &ext_inertia,
        bool override = false) = 0;

    virtual unsigned int getDoF() = 0;

    virtual void setComplianceFrame(const geometry_msgs::Pose &offset) = 0;

protected:
    virtual bool loadModel(const std::string &modelname) = 0;
    std::string model_name;
};

}