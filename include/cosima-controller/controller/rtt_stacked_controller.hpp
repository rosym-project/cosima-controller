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

namespace cosima
{

  namespace controller
  {

    struct StackedRobotInformation
    {
      unsigned int robot_index;
      unsigned int jDoF_index;
      unsigned int tDoF_index;
      unsigned int jDoF;
      unsigned int tDoF;
    };

    class RTTStackedCtrl : public RTT::TaskContext
    {
    public:
      RTTStackedCtrl(std::string const &name) : RTT::TaskContext(name)
      {
        this->addOperation("addRobot", &RTTStackedCtrl::addRobot, this)
            .doc("Add a robot in terms of the sequential (controlled) DoF")
            .arg("dof", "degrees of freedom");

        this->vec_stacked_robot_info.clear();
        this->total_jDoF = 0;
        this->total_tDoF_quat = 0;
        this->total_tDoF_euler = 0;
      }

      // This controller can handle stacked robots.
      bool addRobot(const unsigned int &joint_dof)
      {
        if (joint_dof <= 0)
        {
          RTT::log(RTT::Error) << "[" << this->getName() << "] "
                               << "In order to add a robot, it needs to have at least joint DoF(" << joint_dof << ") > 0" << RTT::endlog();
          return false;
        }

        StackedRobotInformation sri;
        sri.robot_index = this->vec_stacked_robot_info.size();
        sri.jDoF = joint_dof;
        sri.tDoF = 7; // 3 pos + 4 (quat) orn
        this->vec_stacked_robot_info.push_back(sri);
        return true;
      };

      void calculateStackedVariables()
      {
        unsigned int tmp_joint = 0;
        unsigned int tmp_task_quat = 0;
        unsigned int tmp_task_euler = 0;
        for (unsigned int i = 0; i < this->vec_stacked_robot_info.size(); i++)
        {
          this->vec_stacked_robot_info[i].jDoF_index = tmp_joint;
          this->vec_stacked_robot_info[i].tDoF_index = tmp_task_quat;

          tmp_joint += this->vec_stacked_robot_info[i].jDoF;
          tmp_task_quat += this->vec_stacked_robot_info[i].tDoF;
          tmp_task_euler += this->vec_stacked_robot_info[i].tDoF - 1;

          RTT::log(RTT::Error) << "[" << this->getName() << "] "
                               << "Added Robot "
                               << this->vec_stacked_robot_info[i].robot_index
                               << " at jDoF_index: "
                               << this->vec_stacked_robot_info[i].jDoF_index
                               << ", tDoF_index: "
                               << this->vec_stacked_robot_info[i].tDoF_index
                               << " with jDoF: " << this->vec_stacked_robot_info[i].jDoF
                               << ", tDoF: "
                               << this->vec_stacked_robot_info[i].tDoF
                               << RTT::endlog();
        }

        this->total_jDoF = tmp_joint;
        this->total_tDoF_quat = tmp_task_quat;
        this->total_tDoF_euler = tmp_task_euler;
      };

    protected:
      // Store the DoF information for the added robots
      std::vector<StackedRobotInformation> vec_stacked_robot_info;
      unsigned int total_jDoF;
      unsigned int total_tDoF_quat;
      unsigned int total_tDoF_euler;
    };

  } // namespace controller

} // namespace cosima