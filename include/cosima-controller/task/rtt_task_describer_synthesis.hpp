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

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>
#include <Eigen/Dense>
#include <rtt/Component.hpp>

#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

#include <sensor_msgs/JointState.h>

#include "../util/LowPassFilter.hpp"
#include <memory>

// ROS KDL PARSER includes
#include <kdl_parser/kdl_parser.hpp>

// KDL includes
#include <kdl/tree.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

#include <yaml-cpp/yaml.h>

#include "robot_container.hpp"
#include "control_objective_container.hpp"
#include "control_component_container.hpp"
#include "vm_container.hpp"

// header for introspection
#include "../introspection/rtt-introspection-base.hpp"

namespace cosima
{

  namespace task
  {

    class TaskDescriberSynthesis : public cogimon::RTTIntrospectionBase
    {
    public:
      TaskDescriberSynthesis(std::string const &name);

      ///////////////////////////////////////////
      // Internal mirrors of the default Orocos
      // life cycle from the introspection base.
      ///////////////////////////////////////////
      bool configureHookInternal();
      bool startHookInternal();
      void updateHookInternal();
      void stopHookInternal();
      void cleanupHookInternal();
      ///////////////////////////////////////////

      void setJointDOFsize(unsigned int JointDOFsize);
      void preparePorts();
      void displayCurrentState();

      bool loadYAMLConfig(const std::string &file);

      void writeMappingConfigurationToFile(const std::string file);

      void activateContactSituation(const std::string &csName);

      void activateContactSituationResetActivation(const std::string &csName, const double time_secs);

      bool switchCSinSecs(const std::string &cs_name, const double time_secs);

    private:
      struct JacobianConfig
      {
        unsigned int J_ID;
        unsigned int J_Selection_ID;
        unsigned int J_Base_ID;
        unsigned int J_Out_Port_ID;
      };

      // RTT::InputPort<Eigen::VectorXd> cartBaseEEF // ?
      std::vector<std::shared_ptr<RobotContainer>> vec_robots_;
      std::vector<std::shared_ptr<VMContainer>> vec_vms_;

      std::vector<std::shared_ptr<ControlComponentContainer>> vec_ccs_;

      std::vector<std::shared_ptr<ControlObjectiveContainer>> vec_cos_;

      bool exists_test(const std::string &name);

      RTT::OutputPort<double> out_debug_pointintime_port;

      RTT::FlowStatus J_Flow;
      RTT::InputPort<Eigen::MatrixXd> in_J_Input_port;
      Eigen::MatrixXd in_J_var;
      // J dot
      RTT::FlowStatus J_Dot_Flow;
      RTT::InputPort<Eigen::MatrixXd> in_J_Dot_Input_port;
      Eigen::MatrixXd in_J_Dot_var;
      // M
      RTT::FlowStatus M_Flow;
      RTT::InputPort<Eigen::MatrixXd> in_M_Input_port;
      Eigen::MatrixXd in_M_var;
      // GC
      RTT::FlowStatus GC_Flow;
      RTT::InputPort<Eigen::VectorXd> in_GC_Input_port;
      Eigen::VectorXd in_GC_var;
      // robot status
      RTT::FlowStatus robotstatus_Flow;
      RTT::InputPort<sensor_msgs::JointState> in_robotstatus_Input_port;
      sensor_msgs::JointState in_robotstatus_var;
      // cart position of endeffectors
      RTT::FlowStatus CartPos_Flow;
      RTT::InputPort<Eigen::VectorXd> in_CartPos_Input_port;
      Eigen::VectorXd in_CartPos_var;
      // cart velocity of endeffectors
      RTT::FlowStatus CartVel_Flow;
      RTT::InputPort<Eigen::VectorXd> in_CartVel_Input_port;
      Eigen::VectorXd in_CartVel_var;

      bool portsArePrepared;

      double debug_pointintime;

      double activation_inc_steps_;
      void increaseActivation();

      bool new_cs_requested_;
      std::string new_cs_name_requested_;
      double new_cs_timed_requested_;
      //
      bool new_cs_reached_;
      bool isNewCSReached();

      // std::vector<double> time_storage;
      // std::vector<double> time_storage_robot_containers;
      // std::vector<double> time_storage_vm;
      // std::vector<double> time_storage_co_time;
      // std::vector<double> time_storage_co_gemini;
      // std::vector<double> time_storage_send;

      Eigen::VectorXd in_robotstatus_pos_;
      Eigen::VectorXd in_robotstatus_vel_;
      Eigen::VectorXd in_robotstatus_trq_;

      RTT::OperationCaller<void(std::string,double)> tc_prio_activateContactSituationResetActivation;
    };

  } // namespace task

} // namespace cosima