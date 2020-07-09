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

#include "../../include/cosima-controller/task/vm_geometry_box.hpp"

#include <fstream>
#include <iostream>

VMGeometryBox::VMGeometryBox()
{
}

void VMGeometryBox::initialize(const unsigned int numberRobots)
{
    relativeFinger2ObjectTranslation.clear();
    relativeFinger2ObjectRotation33.clear();
    for (unsigned int manipulator = 0; manipulator < numberRobots; manipulator++)
    { //TODO extend for multiple objects
        relativeFinger2ObjectTranslation.push_back(Eigen::Vector3d::Zero());
        relativeFinger2ObjectRotation33.push_back(Eigen::Matrix<double, 3, 3>::Identity());
    }
}

bool VMGeometryBox::computeBoxPosVelEstimation(
    const unsigned int numberRobots,
    const Eigen::VectorXd &in_cartPos,
    const Eigen::VectorXd &in_cartVel,
    const Eigen::VectorXd &in_incontactstate,
    const Eigen::MatrixXd &graspmapPinvT,
    const Eigen::MatrixXd &PCstrGraspmap,
    std::vector<Eigen::Vector3d> &manipulatorTranslation,
    std::vector<Eigen::Matrix<double, 3, 3>> &manipulatorRotation33,
    Eigen::VectorXd &out_cartPosTaskBox,
    Eigen::VectorXd &out_cartVelTaskBox)
{
    // WARNING: At the moment we only support 2 manipulators
    if (numberRobots != 2)
    {
        // TODO DLW
        RTT::log(RTT::Error) << "At the moment we only support 2 manipulators! Skipping. Please handle this properly!" << RTT::endlog();
        return false;
    }

    bool _in_contact_state = true;
    // 1) recompute global position and orientation of manipulators and store them.
    for (unsigned int manipulator = 0; manipulator < numberRobots; manipulator++)
    {
        manipulatorTranslation.at(manipulator) = in_cartPos.segment<3>(manipulator * 7);
        Eigen::Vector4d manipulatorRotationQuaternion = in_cartPos.segment<4>(manipulator * 7 + 3);
        KDL::Rotation manipulatorRotationKDL = KDL::Rotation::Quaternion(manipulatorRotationQuaternion(1), manipulatorRotationQuaternion(2), manipulatorRotationQuaternion(3), manipulatorRotationQuaternion(0));
        this->convertKDLRotation2EigenMatrix(manipulatorRotationKDL, manipulatorRotation33.at(manipulator));
        if (in_incontactstate(manipulator) == 0)
        {
            _in_contact_state = false;
        }
    }

    if (!_in_contact_state)
    {
        return false;
    }

    int tightManipulatorID = -1;
    // Eigen::VectorXd xdot_c = PCstrGraspmap * in_cartVel;
    // double leastSlippingNorm = std::numeric_limits<double>::max();
    // double slippingNorm = std::numeric_limits<double>::max();
    // for (unsigned int manipulator = 0; manipulator < numberRobots; manipulator++)
    // {
    //     if (in_incontactstate(manipulator) == 1)
    //     {
    //         // TODO eventually impose weighting between position and orientation.
    //         // Based on the slipping norm, decide if the manipulator really is in contact...
    //         slippingNorm = xdot_c.segment<3>(manipulator * 6).norm() + xdot_c.segment<3>(manipulator * 6 + 3).norm();
    //         if (slippingNorm < leastSlippingNorm)
    //         {
    //             leastSlippingNorm = slippingNorm;
    //             tightManipulatorID = manipulator;
    //         }
    //     }
    // }
    tightManipulatorID = 0; // TODO shortcut to check if everything is alright!

    Eigen::Matrix<double, 3, 3> boxRotation33 = Eigen::Matrix<double, 3, 3>::Identity();
    if (tightManipulatorID != -1) // Otherwise, no manipulator is in contact.
    {
        // out_cartPosTaskBox.segment<3>(0) = relativeFinger2ObjectTranslation.at(tightManipulatorID) + manipulatorTranslation.at(tightManipulatorID);
        out_cartPosTaskBox.segment<3>(0) = (manipulatorTranslation.at(1) - manipulatorTranslation.at(tightManipulatorID)) * 0.5 + manipulatorTranslation.at(tightManipulatorID);

        boxRotation33 = manipulatorRotation33.at(tightManipulatorID) * relativeFinger2ObjectRotation33.at(tightManipulatorID);
        KDL::Rotation boxRotationKDL;
        this->convertEigenMatrix2KDLRotation(boxRotation33, boxRotationKDL);
        double x, y, z, w;
        boxRotationKDL.GetQuaternion(x, y, z, w);
        out_cartPosTaskBox(3) = w;
        out_cartPosTaskBox(4) = x;
        out_cartPosTaskBox(5) = y;
        out_cartPosTaskBox(6) = z;
    }

    // Update relative transformations between manipulator and object
    for (int manipulator = 0; manipulator < numberRobots; manipulator++)
    {
        if (manipulator != tightManipulatorID)
        {
            //if (in_incontactstate(manipulator) != 1){ // TODO use this if condition when averaging box pose
            relativeFinger2ObjectTranslation.at(manipulator) = out_cartPosTaskBox.segment<3>(0) - manipulatorTranslation.at(manipulator);
            relativeFinger2ObjectRotation33.at(manipulator) = manipulatorRotation33.at(manipulator).inverse() * boxRotation33;
        }
    }
    if (tightManipulatorID != -1)
    {
        // TODO implement version based on tightManipulatorID which is better than this.
        out_cartVelTaskBox = graspmapPinvT.block<6, 6>(0, tightManipulatorID * 6) * in_cartVel.segment<6>(tightManipulatorID * 6);
    }
    else
    {
        out_cartVelTaskBox.setZero();
    }

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