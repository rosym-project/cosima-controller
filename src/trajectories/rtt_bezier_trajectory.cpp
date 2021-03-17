/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2018 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
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

#include "../../include/cosima-controller/trajectories/rtt_bezier_trajectory.hpp"

#include <rtt/Component.hpp> // needed for the macro at the end of this file

using namespace cosima;

BezierTrajectory::BezierTrajectory(std::string const &name) : RTT::TaskContext(name),
                                                              start_time(0.0),
                                                              firstReceivedFeedback(false),
                                                              converged(false),
                                                              TaskSpaceDimension(6),
                                                              timeItShouldTake(10),
                                                              t(0.0)
{
    addOperation("preparePorts", &BezierTrajectory::preparePorts, this).doc("prepare ports");

    addOperation("setConverged", &BezierTrajectory::setConverged, this, RTT::ClientThread);

    addOperation("testBezierCurve", &BezierTrajectory::testBezierCurve, this, RTT::ClientThread);

    addOperation("setEndPoint", &BezierTrajectory::setEndPoint, this, RTT::ClientThread);
    addOperation("setPostStartPoint", &BezierTrajectory::setPostStartPoint, this, RTT::ClientThread);
    addOperation("setPreEndPoint", &BezierTrajectory::setPreEndPoint, this, RTT::ClientThread);
    addOperation("setBezierPointsRSTRT", &BezierTrajectory::setBezierPointsRSTRT, this, RTT::ClientThread);
    addOperation("setBezierPointsEigen", &BezierTrajectory::setBezierPointsEigen, this, RTT::ClientThread);

    addProperty("converged", converged);

    addProperty("timeItShouldTake", timeItShouldTake);

    lastValueO = Eigen::Quaternionf(0, 0, 1, 0);

    startPoint = Eigen::Vector3d::Zero();
    postStartPoint = Eigen::Vector3d::Zero();
    preEndPoint = Eigen::Vector3d::Zero();
    endPoint = Eigen::Vector3d::Zero();
}

void BezierTrajectory::setPostStartPoint(float x, float y, float z)
{
    postStartPoint(0) = x;
    postStartPoint(1) = y;
    postStartPoint(2) = z;
}
void BezierTrajectory::setPreEndPoint(float x, float y, float z)
{
    preEndPoint(0) = x;
    preEndPoint(1) = y;
    preEndPoint(2) = z;
}
void BezierTrajectory::setEndPoint(float x, float y, float z)
{
    endPoint(0) = x;
    endPoint(1) = y;
    endPoint(2) = z;
}

void BezierTrajectory::setBezierPointsEigen(const Eigen::VectorXd &postStart, const Eigen::VectorXd &preEnd, const Eigen::VectorXd &end)
{
    postStartPoint = postStart.segment<3>(0);
    preEndPoint = preEnd.segment<3>(0);
    endPoint = end.segment<3>(0);
}

void BezierTrajectory::calculateBezierCurve(float t, Eigen::Vector3d &out)
{
    out(0) = pow(1 - t, 3) * startPoint(0) + 3 * t * pow(1 - t, 2) * postStartPoint(0) + 3 * pow(t, 2) * (1 - t) * preEndPoint(0) + pow(t, 3) * endPoint(0);
    out(1) = pow(1 - t, 3) * startPoint(1) + 3 * t * pow(1 - t, 2) * postStartPoint(1) + 3 * pow(t, 2) * (1 - t) * preEndPoint(1) + pow(t, 3) * endPoint(1);
    out(2) = pow(1 - t, 3) * startPoint(2) + 3 * t * pow(1 - t, 2) * postStartPoint(2) + 3 * pow(t, 2) * (1 - t) * preEndPoint(2) + pow(t, 3) * endPoint(2);
}

void BezierTrajectory::testBezierCurve(float t)
{
    RTT::log(RTT::Error) << "[BezierTrajectory](" << this->getName()
                         << "\nx = " << (pow(1 - t, 3) * startPoint(0) + 3 * t * pow(1 - t, 2) * postStartPoint(0) + 3 * pow(t, 2) * (1 - t) * preEndPoint(0) + pow(t, 3) * endPoint(0)) << "\n"
                         << "y = " << (pow(1 - t, 3) * startPoint(1) + 3 * t * pow(1 - t, 2) * postStartPoint(1) + 3 * pow(t, 2) * (1 - t) * preEndPoint(1) + pow(t, 3) * endPoint(1)) << "\n"
                         << "z = " << (pow(1 - t, 3) * startPoint(2) + 3 * t * pow(1 - t, 2) * postStartPoint(2) + 3 * pow(t, 2) * (1 - t) * preEndPoint(2) + pow(t, 3) * endPoint(2)) << RTT::endlog();
}

void BezierTrajectory::setConverged(bool conv)
{
    converged = conv;
}

bool BezierTrajectory::configureHook()
{
    preparePorts();
    return true;
}

bool BezierTrajectory::startHook()
{
    out = rstrt::geometry::Translation();
    firstReceivedFeedback = false;
    converged = false;
    this->start_time = getSimulationTime();
    return true;
}

void BezierTrajectory::updateHook()
{
    if (!firstReceivedFeedback)
    {
        in_currentTaskSpacePosition_flow = in_currentTaskSpacePosition_port.read(in_currentTaskSpacePosition_var);
        if (in_currentTaskSpacePosition_flow != RTT::NewData)
        {
            return;
        }
        RTT::log(RTT::Error) << "[BezierTrajectory](" << this->getName() << ") First time feedback received" << RTT::endlog();

        // Save initial orientation
        lastValueO.w() = in_currentTaskSpacePosition_var.orientation.w;
        lastValueO.x() = in_currentTaskSpacePosition_var.orientation.x;
        lastValueO.y() = in_currentTaskSpacePosition_var.orientation.y;
        lastValueO.z() = in_currentTaskSpacePosition_var.orientation.z;

        // Save initial translation as start position for the curve
        startPoint(0) = in_currentTaskSpacePosition_var.position.x;
        startPoint(1) = in_currentTaskSpacePosition_var.position.y;
        startPoint(2) = in_currentTaskSpacePosition_var.position.z;

        // Resetting time
        this->start_time = getSimulationTime();

        // Initialize variables
        converged = false;
        firstReceivedFeedback = true;
    }

    double timeElapsed = getSimulationTime() - this->start_time;
    double tDelta = (1 / timeItShouldTake) * this->getPeriod(); // Could potentially lead to a problem in non RT systems.

    t += tDelta;
    if (t > 1.0)
    {
        t = 1.0;
    }
    calculateBezierCurve(t, out);

    out_desiredTaskSpaceCommand_var(0) = out(0);
    out_desiredTaskSpaceCommand_var(1) = out(1);
    out_desiredTaskSpaceCommand_var(2) = out(2);

    out_desiredTaskSpaceVelocity_var.setZero();

    out_desiredTaskSpaceAcceleration_var.setZero(); // TODO DLW ?!

    out_desiredTaskSpaceCommand_var(3) = lastValueO.w();
    out_desiredTaskSpaceCommand_var(4) = lastValueO.x();
    out_desiredTaskSpaceCommand_var(5) = lastValueO.y();
    out_desiredTaskSpaceCommand_var(6) = lastValueO.z();

    out_desiredTaskSpaceCommand_port.write(out_desiredTaskSpaceCommand_var);
    out_desiredTaskSpaceVelocity_port.write(out_desiredTaskSpaceVelocity_var);
    out_desiredTaskSpaceAcceleration_port.write(out_desiredTaskSpaceAcceleration_var);
}

void BezierTrajectory::stopHook()
{
    firstReceivedFeedback = false;
    t = 0;
}

void BezierTrajectory::cleanupHook()
{
}

void BezierTrajectory::preparePorts()
{
    if (ports()->getPort("out_desiredTaskSpaceCommand_port"))
    {
        ports()->removePort("out_desiredTaskSpaceCommand_port");
    }

    // trajectory_msgs::MultiDOFJointTrajectoryPoint point;
    // point.transforms = std::vector<geometry_msgs::Transform>(1);
    // point.velocities = std::vector<geometry_msgs::Twist>(1);
    // point.accelerations = std::vector<geometry_msgs::Twist>(1);

    // point.transforms[0].translation.x = x;
    // point.transforms[0].translation.y = y;
    // point.transforms[0].translation.z = z;

    // point.transforms[0].rotation.x = qx;
    // point.transforms[0].rotation.y = qy;
    // point.transforms[0].rotation.z = qz;
    // point.transforms[0].rotation.w = qw;


    out_desiredTaskSpaceCommand_var = trajectory_msgs::MultiDOFJointTrajectory();
    out_desiredTaskSpaceCommand_port.setName("out_desiredTaskSpaceCommand_port");
    out_desiredTaskSpaceCommand_port.doc("Output port for sending the desired taskspace position");
    out_desiredTaskSpaceCommand_port.setDataSample(out_desiredTaskSpaceCommand_var);
    ports()->addPort(out_desiredTaskSpaceCommand_port);

    in_currentTaskSpacePosition_var = geometry_msgs::Pose();
    in_currentTaskSpacePosition_port.setName("in_currentTaskSpacePosition_port");
    in_currentTaskSpacePosition_port.doc("Input port for receiving the ground-truth cartesian position");
    in_currentTaskSpacePosition_flow = RTT::NoData;
    ports()->addPort(in_currentTaskSpacePosition_port);
}

double BezierTrajectory::getSimulationTime()
{
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(
                      RTT::os::TimeService::Instance()->getTicks());
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::BezierTrajectory)
