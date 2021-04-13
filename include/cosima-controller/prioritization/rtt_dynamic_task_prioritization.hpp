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
#include <iterator>
#include <map>

#include <boost/lexical_cast.hpp>

#include <sensor_msgs/JointState.h>

#include "rtt_robot_contact_situations.hpp"

#include <yaml-cpp/yaml.h>

// header for introspection
#include "../introspection/rtt-introspection-base.hpp"

namespace cosima
{

  namespace prioritization
  {

    class DynamicTaskPrioritization : public cogimon::RTTIntrospectionBase
    {
    public:
      DynamicTaskPrioritization(std::string const &name);

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

      void setDOFsize(unsigned int DOFsize);
      void preparePorts();
      void displayCurrentState();

      bool loadYAMLConfig(const std::string &file);
      // void debugTestRandom();

      void setNewTargetCS(const std::string &cs_name);
      void setStartCS(const std::string &cs_name);
      void activateContactSituationResetActivation(const std::string &cs_name, const double time_secs);

      void debug_print_state(bool debug);

      void alternateManualCalculationsDebug(Eigen::VectorXd &in_robotstatus_pos,
                                            Eigen::VectorXd &in_robotstatus_vel,
                                            Eigen::VectorXd &in_robotstatus_trq,
                                            const Eigen::MatrixXd &in_jToBeCstr,
                                            const Eigen::MatrixXd &in_jToBeMoved,
                                            // Eigen::MatrixXd &in_projection,
                                            Eigen::MatrixXd &in_inertia,
                                            // Eigen::MatrixXd &in_inertia_c,
                                            Eigen::VectorXd &in_h,
                                            // Eigen::MatrixXd &in_Pdot,
                                            Eigen::VectorXd &in_torquesTask,
                                            Eigen::VectorXd &in_torquesCstr,
                                            Eigen::VectorXd &out_torques);

    private:
      // Declare input ports and their datatypes
      std::vector<boost::shared_ptr<RTT::InputPort<Eigen::VectorXd>>> in_torques_ports;
      std::vector<boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>>> in_jacobian_ports;
      // std::vector<boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>>> in_weighting_ports;
      std::vector<boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>>> in_inertia_ports;
      // VM
      std::vector<boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>>> in_projection_internal_force_ports;
      std::vector<boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>>> in_projection_dot_internal_force_ports;

      RTT::InputPort<Eigen::VectorXd> in_coriolisAndGravity_port;

      RTT::InputPort<sensor_msgs::JointState> in_robotstatus_port;
      RTT::FlowStatus in_robotstatus_flow;
      sensor_msgs::JointState in_robotstatus_var;
      Eigen::VectorXd in_robotstatus_pos_;
      Eigen::VectorXd in_robotstatus_vel_;
      Eigen::VectorXd in_robotstatus_trq_;

      // Declare output ports and their datatypes
      RTT::OutputPort<Eigen::VectorXd> out_torques_port;
      std::vector<boost::shared_ptr<RTT::OutputPort<Eigen::VectorXd>>> out_torquesProj_ports;

      RTT::OutputPort<Eigen::MatrixXd> out_current_alphas_port;
      RTT::OutputPort<Eigen::VectorXd> out_current_projected_trq_cmds_port;
      RTT::OutputPort<Eigen::MatrixXd> out_current_projection_matrices_port;

      RTT::OutputPort<double> out_debug_pointintime_port;
      RTT::OutputPort<std::string> out_new_cs_name_requested_port;
      RTT::OutputPort<bool> out_new_cs_requested_port;
      RTT::OutputPort<bool> out_new_cs_reached_port;

      // Data flow:
      std::vector<RTT::FlowStatus> in_torques_flow;
      std::vector<RTT::FlowStatus> in_jacobian_flow;
      // std::vector<RTT::FlowStatus> in_weighting_flow;
      std::vector<RTT::FlowStatus> in_inertia_flow;
      // VM
      RTT::FlowStatus in_projection_internal_force_flow;
      RTT::FlowStatus in_projection_dot_internal_force_flow;

      RTT::FlowStatus in_coriolisAndGravity_flow;

      // variables
      std::vector<Eigen::VectorXd> in_torques_var;
      std::vector<Eigen::MatrixXd> in_jacobian_var;
      // std::vector<Eigen::MatrixXd> in_weighting_var;
      std::vector<Eigen::MatrixXd> in_inertia_var;
      // VM
      std::vector<Eigen::MatrixXd> in_projection_internal_force_var;
      std::vector<Eigen::MatrixXd> in_projection_dot_internal_force_var;

      Eigen::VectorXd in_coriolisAndGravity_var;
      Eigen::VectorXd out_torques_var;
      std::vector<Eigen::VectorXd> out_torquesProj_var;
      std::vector<unsigned int> Tasksize;
      std::vector<Eigen::MatrixXd> nullspaces, projections;
      Eigen::MatrixXd identityDOFsizeDOFsize;

      // unsigned int numInputPorts;
      unsigned int DOFsize;
      bool portsArePrepared;

      double debug_pointintime;
      // double debug_pointintime_old;

      std::map<std::string, unsigned int> map_csName_to_csId;

      std::vector<RobotContactSituations> rcss;
      // TODO make this local?
      std::vector<std::string> rcss_index_name;

      // DEBUG
      std::vector<double> time_storage;

      double start_time;
      Eigen::MatrixXd previousP;
      bool debug_activeee;
      bool debug_switch_to_manual;
      Eigen::VectorXd out_torquesss;
      Eigen::VectorXd debug_tau_task;
      Eigen::VectorXd debug_tau_cstr;

      void calculateDebug(const Eigen::MatrixXd &in_inertia, const Eigen::VectorXd &in_h, const Eigen::MatrixXd &in_jacobian, const Eigen::VectorXd &jointT, const Eigen::VectorXd &taskT, Eigen::VectorXd &out);

      bool debug_vm_on;

      double activation_inc_steps_;
      void increaseActivation();

      bool new_cs_requested_;

      std::string new_cs_name_requested_;

      double new_cs_timed_requested_;
      //
      bool new_cs_reached_;
      bool isNewCSReached();

      bool substract_gravity;
    };

  } // namespace prioritization

} // namespace cosima