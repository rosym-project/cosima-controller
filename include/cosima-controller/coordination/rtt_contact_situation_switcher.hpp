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

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <Eigen/Dense>
#include <iostream>

// ROS includes
#include <ros/ros.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_roscomm/rostopic.h>
#include <rtt_roscomm/rosservice.h>

// #include "monitor_wrench.hpp"

// ROS TYPE includes
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <cosima_msgs/Assemble.h>
#include <cosima_msgs/Move.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <std_msgs/Bool.h>

namespace cosima
{

  namespace coordination
  {

    class ContactSituationSwitcher : public RTT::TaskContext
    {
    public:
      ContactSituationSwitcher(std::string const &name);

      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();

      // void addWrenchMonitor(std::string const &name);
      // void setWrenchMonitorBounds(std::string const &name, unsigned int const &dimension, double lower, double upper);

    private:
      double getOrocosTime();

      bool assemble_srv(cosima_msgs::AssembleRequest& req,cosima_msgs::AssembleResponse& resp);
      bool move_srv(cosima_msgs::MoveRequest& req,cosima_msgs::MoveResponse& resp);

      // InputPort: current cart pose
      RTT::InputPort<geometry_msgs::Pose> in_pose_port;
      RTT::FlowStatus in_pose_flow;
      geometry_msgs::Pose in_pose_var;

      // InputPort: current cart wrench
      RTT::InputPort<geometry_msgs::Wrench> in_wrench_port;
      RTT::FlowStatus in_wrench_flow;
      geometry_msgs::Wrench in_wrench_var;

      // OutputPort: converged
      RTT::OutputPort<std_msgs::Bool> out_converged_port;
      std_msgs::Bool out_converged_var;

      // OutputPort: command gripper
      RTT::OutputPort<std_msgs::Bool> out_gripper_port;
      std_msgs::Bool out_gripper_var;

      // OutputPort: command pose of arm
      RTT::OutputPort<trajectory_msgs::MultiDOFJointTrajectoryPoint> out_desiredTaskSpace_port;
      trajectory_msgs::MultiDOFJointTrajectoryPoint out_desiredTaskSpace_var;

      // std::map<std::string, std::shared_ptr<MonitorWrench> > map_monitors_wrench;

      // Temp pose variable
      Eigen::Vector3d _pose_var_trans;
      //
      Eigen::Vector3d out_trans_, des_trans_, cur_trans_;

      Eigen::Quaterniond _pose_var_orn;
      //
      Eigen::Quaterniond out_orn_, des_orn_, cur_orn_;

      

      // // InputPort to receive the pass-through pose commands (use for high frequency)
      // RTT::InputPort<geometry_msgs::Pose> in_pose_pt_port;
      // RTT::FlowStatus in_pose_pt_flow;
      // geometry_msgs::Pose in_pose_pt_var;

      // // InputPort to receive the current pose matrix
      // // RTT::InputPort<Eigen::MatrixXd> in_current_pose_port;
      // // RTT::FlowStatus in_current_pose_flow;
      // // Eigen::MatrixXd in_current_pose_var;
      // RTT::InputPort<geometry_msgs::Pose> in_current_pose_port;
      // RTT::FlowStatus in_current_pose_flow;
      // geometry_msgs::Pose in_current_pose_var;

      // // OutputPort for the interpolated pose matrix commands
      // RTT::OutputPort<Eigen::MatrixXd> out_pose_matrix_port;
      // Eigen::MatrixXd out_pose_matrix_var;

      // // OutputPort for the interpolated pose commands
      // RTT::OutputPort<geometry_msgs::Pose> out_pose_port;
      // geometry_msgs::Pose out_pose_var;

      RTT::TaskContext* skill_stack_ptr;

      unsigned int skill_type; /* IDLE 0, MOVE 1, ASSEMBLE 2 */
      bool new_move_command, new_assemble_command;

      double converged_th;
      double move_speed_trans;

      double slerp_time_, move_speed_orn;

    };

  } // namespace coordination

} // namespace cosima