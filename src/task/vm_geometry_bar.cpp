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

#include "../../include/cosima-controller/task/vm_geometry_bar.hpp"

#include <fstream>
#include <iostream>

VMGeometryBar::VMGeometryBar()
{
}

void VMGeometryBar::initialize(const unsigned int numberRobots)
{
}

bool VMGeometryBar::computeBoxPosVelEstimation(
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
        // if (manipulatorTranslation.at(manipulator).hasNaN())
        // {
        //     RTT::log(RTT::Error) << "manipulatorTranslation.at(manipulator) NAN!=\n"
        //                          << manipulatorTranslation.at(manipulator) << RTT::endlog();
        // }

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
        // TODO DLW
        // RTT::log(RTT::Error) << "Not all manipulators in contact yet!" << RTT::endlog();
        return false;
    }

    // // Based on the slipping norm, decide if the manipulator really is in contact...
    // slippingNorm = xdot_c.segment<3>(manipulator * 6).norm() + xdot_c.segment<3>(manipulator * 6 + 3).norm();

    // 2) Calculate the midpoint of the two manipulators
    Eigen::Vector3d objectTranslation = 0.5 * (manipulatorTranslation.at(0) + manipulatorTranslation.at(1));

    // Eigen::Quaternionf rot;
    // rot.setFromTwoVectors(manipulatorTranslation.at(0), manipulatorTranslation.at(1));
    // boxRotation33 = rot.toRotationMatrix(); //manipulatorRotation33.at(tightManipulatorID) * relativeFinger2ObjectRotation33.at(tightManipulatorID);

    // double angleBar = manipulatorTranslation.at(0).dot(manipulatorTranslation.at(1)) / (manipulatorTranslation.at(0).norm() * manipulatorTranslation.at(1).norm());
    // Eigen::AngleAxisd axAngleBar = Eigen::AngleAxisd(angleBar, manipulatorTranslation.at(0).cross(manipulatorTranslation.at(1)));
    // // Eigen::Quaternionf quatfTmp = Eigen::Quaternionf(axAngle.toRotationMatrix());
    // // boxRotation33 = quatfTmp.toRotationMatrix();
    // boxRotation33 = axAngleBar.toRotationMatrix();

    Eigen::Vector3d diffBoxT = manipulatorTranslation.at(0) - manipulatorTranslation.at(1);
    Eigen::Vector3d unitY = Eigen::Vector3d::Zero();
    unitY(0) = 0;
    unitY(1) = 1;
    unitY(2) = 0;

    Eigen::Vector3d crossT = diffBoxT.cross(unitY);
    Eigen::Vector3d crossTNormalized = crossT * (1 / crossT.norm());

    double dotAngle = diffBoxT.dot(unitY);
    double dotAngleSqrt = dotAngle / sqrt(diffBoxT(0) * diffBoxT(0) + diffBoxT(1) * diffBoxT(1) + diffBoxT(2) * diffBoxT(2));
    double dotAngleSqrtAcos = -acos(dotAngleSqrt);

    // RTT::log(RTT::Error) << "dotAngle = " << dotAngle << RTT::endlog();
    // RTT::log(RTT::Error) << "dotAngleSqrt = " << dotAngleSqrt << RTT::endlog();
    // RTT::log(RTT::Error) << "dotAngleSqrtAcos = " << dotAngleSqrtAcos << RTT::endlog();

    Eigen::AngleAxisd axAngleBar = Eigen::AngleAxisd(dotAngleSqrtAcos, crossTNormalized);
    // RTT::log(RTT::Error) << "axAngleBar =\n"
    //                      << axAngleBar << RTT::endlog();

    Eigen::Matrix<double, 3, 3> boxRotation33 = axAngleBar.toRotationMatrix();

    // if (boxRotation33.hasNaN())
    // {
    //     RTT::log(RTT::Error) << "boxRotation33 NAN!=\n"
    //                          << boxRotation33 << RTT::endlog();
    // }

    KDL::Rotation boxRotationKDL;
    this->convertEigenMatrix2KDLRotation(boxRotation33, boxRotationKDL);
    double x, y, z, w;
    boxRotationKDL.GetQuaternion(x, y, z, w);
    // boxRotationQuaternion(0) = w;
    out_cartPosTaskBox(3) = w;
    out_cartPosTaskBox(4) = x;
    out_cartPosTaskBox(5) = y;
    out_cartPosTaskBox(6) = z;
    // TODO DLW calculate the Rotation for the endeffs based on the position of the arms (or on the estimated bar, but arm is better because not really estimated)!
    // That should work since the orientation is not used for the bar position and thus they do not influence each other@

    // Update relative transformations between manipulator and object
    // for (int manipulator = 0; manipulator < numManipulators; manipulator++)
    // {
    // if (manipulator != tightManipulatorID) // Alle anderen auf denen die vorherigen Berechnungen nicht beruhen.
    // {
    // relativeFinger2ObjectTranslation_.at(manipulator) = objectTranslation - manipulatorTranslation.at(manipulator);
    // relativeFinger2ObjectRotation33.at(manipulator) = manipulatorRotation33.at(manipulator).inverse() * boxRotation33;
    // }
    // }
    // TODO not sure if this is still right...?
    // Here I try to calculate the velocity based on the manipulator 0 and its cartesian velocity which is related to the virtual object.
    // out_cartVelTaskBox = graspmapPinvT.block<6, 6>(0, 0 /* tightManipulatorID */ * 6) * in_cartVel.segment<6>(0 /* tightManipulatorID */ * 6);
    // Instead why not use all the manipulators velocities? I really do not know, perhaps we experience interferences this way? Let's find out, shall we?!
    out_cartVelTaskBox = graspmapPinvT * in_cartVel;
    // if (out_cartVelTaskBox.hasNaN())
    // {
    //     RTT::log(RTT::Error) << "(graspmapPinvT_ * in_cartVel) NAN!=\n"
    //                          << out_cartVelTaskBox << RTT::endlog();
    // }
    // or just set it zero...
    // boxVelocity.setZero();
    out_cartPosTaskBox.head<3>() = objectTranslation;
    // if (objectTranslation.hasNaN())
    // {
    //     RTT::log(RTT::Error) << "objectTranslation NAN!=\n"
    //                          << objectTranslation << RTT::endlog();
    // }
    // out_cartPosTaskBox.tail<4>() = boxRotationQuaternion;

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