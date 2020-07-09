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

#include <Eigen/Dense>

class VMGeometry
{
public:
    // virtual ~VMGeometry() {}
    virtual bool computeBoxPosVelEstimation(
        const unsigned int numberRobots,
        const Eigen::VectorXd &in_cartPos,
        const Eigen::VectorXd &in_cartVel,
        const Eigen::VectorXd &in_incontactstate,
        const Eigen::MatrixXd &graspmapPinvT,
        const Eigen::MatrixXd &PCstrGraspmap,
        std::vector<Eigen::Vector3d> &manipulatorTranslation,
        std::vector<Eigen::Matrix<double, 3, 3>> &manipulatorRotation33,
        Eigen::VectorXd &out_cartPosTaskBox,
        Eigen::VectorXd &out_cartVelTaskBox){

    };

    virtual void initialize(const unsigned int numberRobots){

    };

    void convertEigenMatrix2KDLRotation(const Eigen::Matrix<double, 3, 3> &eigen, KDL::Rotation &kdl)
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
    };

    void convertKDLRotation2EigenMatrix(const KDL::Rotation &kdl, Eigen::Matrix<double, 3, 3> &eigen)
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
    };
};
