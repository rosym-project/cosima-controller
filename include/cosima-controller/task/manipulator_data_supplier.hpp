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

#include <string>
#include <Eigen/Dense>

class ManipulatorDataSupplier
{
public:
    virtual ~ManipulatorDataSupplier() {}
    virtual std::string getRobotName() = 0;

    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd jacobian_dot_;
    Eigen::MatrixXd jacobian_internal_;
    Eigen::MatrixXd jacobian_internal_dot_;
    Eigen::MatrixXd inertia_;
    Eigen::VectorXd gc_;
    Eigen::VectorXd robotstatus_pos_;
    Eigen::VectorXd robotstatus_vel_;
    Eigen::VectorXd robotstatus_trq_;
    Eigen::VectorXd cart_pos_;
    Eigen::VectorXd cart_vel_;
    // Only if this is a VM
    Eigen::VectorXd estimated_force_directionEE_;

    double fadeConstraintActivation_;
};
