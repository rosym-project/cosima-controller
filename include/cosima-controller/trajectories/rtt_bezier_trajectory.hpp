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

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/os/Timer.hpp>
#include <string>

#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

namespace cosima
{

class BezierTrajectory : public RTT::TaskContext
{
public:
  BezierTrajectory(std::string const &name);

  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void cleanupHook();

  // void setStartPoint(float x, float y, float z);
  void setEndPoint(float x, float y, float z);
  void setPostStartPoint(float x, float y, float z);
  void setPreEndPoint(float x, float y, float z);

  void setBezierPointsEigen(const Eigen::VectorXd &postStart, const Eigen::VectorXd &preEnd, const Eigen::VectorXd &end);

  Eigen::Vector3d startPoint;
  Eigen::Vector3d postStartPoint;
  Eigen::Vector3d preEndPoint;
  Eigen::Vector3d endPoint;

  double timeItShouldTake;
  double t;
  Eigen::Vector3d out;

  void testBezierCurve(float t);

  //####################################

  void resetStartTime(float r);
  void preparePorts();
  double getSimulationTime();
  void setTimescale(float timescale);

private:
  void calculateBezierCurve(float t, Eigen::Vector3d &out);

  void setConverged(bool conv);

  // Input port for the current pose
  RTT::InputPort<geometry_msgs::Pose> in_currentTaskSpacePosition_port;
  RTT::FlowStatus in_currentTaskSpacePosition_flow;
  geometry_msgs::Pose in_currentTaskSpacePosition_var;

  // Output port for the command
  RTT::OutputPort<trajectory_msgs::MultiDOFJointTrajectory> out_desiredTaskSpaceCommand_port;
  trajectory_msgs::MultiDOFJointTrajectory out_desiredTaskSpaceCommand_var;

  unsigned int TaskSpaceDimension;
  double start_time;

  bool firstReceivedFeedback;
  bool converged;

  Eigen::Quaterniond lastValueO;
};

} // namespace cosima