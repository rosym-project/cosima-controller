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

#include "../../include/cosima-controller/prioritization/rtt_dynamic_task_prioritization.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#include "../../include/cosima-controller/util/MatrixDecomposition.hpp"

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

using namespace cosima;
using namespace prioritization;
using namespace util;

DynamicTaskPrioritization::DynamicTaskPrioritization(std::string const &name) : cogimon::RTTIntrospectionBase(name), portsArePrepared(false), substract_gravity(false)
{
    //prepare operations
    // addOperation("setDOFsize", &DynamicTaskPrioritization::setDOFsize, this).doc("set DOF size");
    // addOperation("preparePorts", &DynamicTaskPrioritization::preparePorts, this).doc("prepare ports");
    addOperation("displayCurrentState", &DynamicTaskPrioritization::displayCurrentState, this).doc("displayCurrentState");
    addOperation("loadYAMLConfig", &DynamicTaskPrioritization::loadYAMLConfig, this).doc("Load YAML (.yaml) Config File").arg("file", "Complete path to .yaml file.");

    // addOperation("debugTestRandom", &DynamicTaskPrioritization::debugTestRandom, this);

    addOperation("setNewTargetCS", &DynamicTaskPrioritization::setNewTargetCS, this);
    addOperation("setStartCS", &DynamicTaskPrioritization::setStartCS, this);
    addOperation("activateContactSituationResetActivation", &DynamicTaskPrioritization::activateContactSituationResetActivation, this);

    addProperty("debug_pointintime", debug_pointintime);
    addOperation("debug_print_state", &DynamicTaskPrioritization::debug_print_state, this);

    addOperation("isNewCSReached", &DynamicTaskPrioritization::isNewCSReached, this);

    addProperty("substract_gravity", this->substract_gravity);

    debug_pointintime = 0;

    debug_activeee = false;
    addProperty("debug_activeee", debug_activeee);
    debug_switch_to_manual = false;
    addProperty("debug_switch_to_manual", debug_switch_to_manual);

    time_storage.reserve(100000);

    out_torquesss = Eigen::VectorXd::Zero(7);
    addProperty("out_torquesss", out_torquesss);
    addProperty("out_torques_var", out_torques_var);

    // TODO remove
    start_time = -1;
    previousP = Eigen::MatrixXd::Zero(7, 7);
    debug_tau_task = Eigen::VectorXd::Zero(7);
    debug_tau_cstr = Eigen::VectorXd::Zero(7);
    addProperty("debug_tau_task", debug_tau_task);
    addProperty("debug_tau_cstr", debug_tau_cstr);

    this->debug_vm_on = true;
    addProperty("debug_vm_on", debug_vm_on);

    this->activation_inc_steps_ = 0;

    this->new_cs_requested_ = false;
    this->new_cs_name_requested_ = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
    this->new_cs_timed_requested_ = 0.0;
    this->new_cs_reached_ = true; // ?

    addProperty("new_cs_name_requested", new_cs_name_requested_);
    addProperty("new_cs_requested", new_cs_requested_);
    addProperty("new_cs_reached", new_cs_reached_);

    out_debug_pointintime_port.setName("out_debug_pointintime_port");
    out_debug_pointintime_port.doc("Output port for sending the current point in time for the transition");
    out_debug_pointintime_port.setDataSample(debug_pointintime);
    ports()->addPort(out_debug_pointintime_port);

    out_new_cs_name_requested_port.setName("out_new_cs_name_requested_port");
    out_new_cs_name_requested_port.doc("Output port for sending the current point in time for the transition");
    out_new_cs_name_requested_port.setDataSample(new_cs_name_requested_);
    ports()->addPort(out_new_cs_name_requested_port);

    out_new_cs_requested_port.setName("out_new_cs_requested_port");
    out_new_cs_requested_port.doc("Output port for sending the current point in time for the transition");
    out_new_cs_requested_port.setDataSample(new_cs_requested_);
    ports()->addPort(out_new_cs_requested_port);

    out_new_cs_reached_port.setName("out_new_cs_reached_port");
    out_new_cs_reached_port.doc("Output port for sending the current point in time for the transition");
    out_new_cs_reached_port.setDataSample(new_cs_reached_);
    ports()->addPort(out_new_cs_reached_port);
}

void DynamicTaskPrioritization::debug_print_state(bool debug)
{
    for (unsigned int i = 0; i < rcss.size(); i++)
    {
        rcss[i].debug_print_state(debug);
    }
}

// Need to be called after setDOFsize is called????????????
bool DynamicTaskPrioritization::loadYAMLConfig(const std::string &file)
{
    std::map<std::string, unsigned int> map_co_2_dofs;
    std::map<std::string, unsigned int> map_co_2_jointDofs;
    std::map<std::string, std::string> map_co_2_type;

    YAML::Node config = YAML::LoadFile(file);

    // RTT::log(RTT::Error) << "config.type = " << config.Type() << RTT::endlog();
    YAML::Node rootControlObjectivesConfig = config["RootControlObjectives"];
    if (!rootControlObjectivesConfig.IsDefined() || rootControlObjectivesConfig.IsNull())
    {
        RTT::log(RTT::Error) << "RootControlObjectives is not defined properly!" << RTT::endlog();
        return false;
    }
    // 1) Iterate through all robots and get all the control objectives
    for (std::size_t i = 0; i < rootControlObjectivesConfig.size(); i++)
    {
        YAML::Node entry = rootControlObjectivesConfig[i];
        YAML::Node robot = entry["Robot"];

        std::string robotName = robot.as<std::string>();
        RTT::log(RTT::Error) << "> " << robotName << RTT::endlog();

        YAML::Node cs = entry["ContactSituations"];
        RTT::log(RTT::Error) << " Amount of CS " << cs.size() << RTT::endlog();

        YAML::Node cos = entry["ControlObjectives"];
        RTT::log(RTT::Error) << " Amount of CO " << cos.size() << RTT::endlog();

        unsigned int jointDOF = entry["JointDof"].as<unsigned int>();
        RTT::log(RTT::Error) << " Joint Dof controlled by robot " << jointDOF << RTT::endlog();

        YAML::Node amVM = entry["VM"];
        bool amIaVM = true;
        if (!amVM.IsDefined() || amVM.IsNull())
        {
            amIaVM = false;
        }

        // Create RCS for each robot
        RobotContactSituations rcs(cs.size(), jointDOF, cos.size(), amIaVM);
        rcs.setRobotName(robotName);
        RTT::log(RTT::Error) << "RCS: " << robotName << " initialized!" << RTT::endlog();

        // if (amVM.IsDefined() && !amVM.IsNull()) {
        //     for (std::size_t d = 0; d < amVM.size(); d++) {
        //         std::string vmRname = amVM[d].as<std::string>();
        //         rsc.addVM
        //     }
        // }

        for (std::size_t j = 0; j < cos.size(); j++)
        {
            std::string co_name = cos[j]["Name"].as<std::string>();
            unsigned int taskdof = cos[j]["OperationalFrame"]["Dof"].as<unsigned int>();
            std::string type = cos[j]["OperationalFrame"]["Type"].as<std::string>();
            RTT::log(RTT::Error) << " - " << co_name << ", DoF: " << taskdof << ", type: " << type << RTT::endlog();
            map_co_2_dofs[co_name] = taskdof;
            map_co_2_type[co_name] = type;
            map_co_2_jointDofs[co_name] = jointDOF;
            rcs.addControlObjectiveTask(co_name, j, taskdof);
            RTT::log(RTT::Error) << "RCS: " << robotName << " addControlObjectiveTask(" << co_name << ", " << j << ", " << taskdof << ")" << RTT::endlog();
        }

        // Parse all contact situations
        for (std::size_t h = 0; h < cs.size(); h++)
        {
            std::string cs_name = cs[h]["Name"].as<std::string>();

            Eigen::MatrixXd prio;
            prio.resize(cos.size(), cos.size());
            std::vector<std::string> header;
            YAML::Node prioMatrix = cs[h]["PrioritizationMatrix"];
            for (std::size_t v = 0; v < prioMatrix.size(); v++)
            {
                YAML::Node innerCells = prioMatrix[v];
                for (std::size_t w = 0; w < innerCells.size(); w++)
                {
                    if (v == 0)
                    {
                        // we have the header
                        header.push_back(innerCells[w].as<std::string>());
                    }
                    else
                    {
                        // we have the floating numbers
                        prio(v - 1, w) = innerCells[w].as<double>();
                    }
                }
            }
            rcs.addCSAlpha(cs_name, prio, header);

            // This just seems to be for the debug output log!
            std::string removeMe1 = "";
            for (unsigned int hh = 0; hh < header.size(); hh++)
            {
                removeMe1 += header[hh] + " ";
            }
            RTT::log(RTT::Error) << "RCS: " << robotName << " addCSAlpha(" << cs_name << ",\n"
                                 << prio << ",\n"
                                 << removeMe1 << ")" << RTT::endlog();
        }

        out_torquesProj_var.push_back(Eigen::VectorXd::Zero(jointDOF));
        RTT::log(RTT::Error) << "For " << rcs.getRobotName() << " add dims: " << (jointDOF) << RTT::endlog();

        rcss.push_back(rcs);
        rcss_index_name.push_back(robotName);
        // map_robot_2_processingIndex[robotName] = i; // This stores the robot Id in rcss with regards to the name.
    }

    RTT::log(RTT::Error) << "---------" << RTT::endlog();

    // Create or use already existent ports
    YAML::Node portMapsConfig = config["PortMaps"];
    if (!portMapsConfig.IsDefined() || portMapsConfig.IsNull())
    {
        RTT::log(RTT::Error) << "PortMaps is not defined properly!" << RTT::endlog();
        return false;
    }
    // Find in PortMaps -> Component: Prioritization
    YAML::Node prioComponent = portMapsConfig["Prioritization"];
    if (!prioComponent.IsDefined() || prioComponent.IsNull() || !prioComponent.IsSequence())
    {
        RTT::log(RTT::Error) << "Prioritization is not defined properly in PortMaps!" << RTT::endlog();
        return false;
    }
    for (std::size_t i = 0; i < prioComponent.size(); i++)
    {
        YAML::Node port = prioComponent[i];
        YAML::Node port_name = port["Name"];
        if (!port_name.IsDefined() || port_name.IsNull())
        {
            RTT::log(RTT::Error) << "Name is not defined in port " << i << "!" << RTT::endlog();
            return false;
        }
        std::string port_name_val = port_name.as<std::string>();
        RTT::log(RTT::Error) << "< Port Base Name: " << port_name_val << RTT::endlog();

        // Check if port already exists
        boost::shared_ptr<RTT::InputPort<Eigen::VectorXd>> tmpPort_Tau = NULL;
        for (unsigned int p = 0; p < in_torques_ports.size(); p++)
        {
            if (in_torques_ports[p]->getName().compare("in_" + port_name_val + "_torques") == 0)
            {
                tmpPort_Tau = in_torques_ports[p];
                break;
            }
        }
        if (!tmpPort_Tau)
        {
            boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>> tmpPort = boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>>(new RTT::InputPort<Eigen::MatrixXd>());
            tmpPort->setName("in_" + port_name_val + "_J");
            tmpPort->doc("Input port for reading Jacobian values");
            ports()->addPort(*tmpPort);
            in_jacobian_ports.push_back(tmpPort);
            in_jacobian_flow.push_back(RTT::NoData);

            tmpPort_Tau = boost::shared_ptr<RTT::InputPort<Eigen::VectorXd>>(new RTT::InputPort<Eigen::VectorXd>());
            tmpPort_Tau->setName("in_" + port_name_val + "_torques");
            tmpPort_Tau->doc("Input port for reading Torque command values");
            ports()->addPort(*tmpPort_Tau);
            in_torques_ports.push_back(tmpPort_Tau);
            in_torques_flow.push_back(RTT::NoData);

            boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>> tmpPort_M = boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>>(new RTT::InputPort<Eigen::MatrixXd>());
            tmpPort_M->setName("in_" + port_name_val + "_inertia");
            tmpPort_M->doc("Input port for reading Inertia values");
            ports()->addPort(*tmpPort_M);
            in_inertia_ports.push_back(tmpPort_M);
            in_inertia_flow.push_back(RTT::NoData);

            // Check all ControlObjectives
            YAML::Node port_co_split_seq = port["ControlObjectives"];
            if (!port_co_split_seq.IsDefined() || port_co_split_seq.IsNull() || !port_co_split_seq.IsSequence())
            {
                RTT::log(RTT::Error) << "ControlObjectives are not defined in port " << i << "!" << RTT::endlog();
                return false;
            }

            unsigned int summedDof = 0;
            unsigned int jointDof = 0;
            for (std::size_t j = 0; j < port_co_split_seq.size(); j++)
            {
                std::string co_name = port_co_split_seq[j].as<std::string>();
                // if (map_co_2_type[co_name].compare("Chain") == 0) {
                //     // Ignore since the jacobian will always be 1.
                // }

                // Find in map and sum dofs
                summedDof += map_co_2_dofs[co_name];
                // Assume that each co that is associated with this port has the same number of joint dofs.
                jointDof += map_co_2_jointDofs[co_name]; // TODO
            }
            RTT::log(RTT::Error) << " Found sumed up DoFs " << summedDof << ", and joint DoFs " << jointDof << RTT::endlog();
            // Create jacobian vairable with sumed up task dofs.
            in_jacobian_var.push_back(Eigen::MatrixXd(summedDof, jointDof));
            RTT::log(RTT::Error) << "in_jacobian_var: push_back MatrixXd(" << summedDof << ", " << jointDof << ")" << RTT::endlog();
            // in_torques_var.push_back(Eigen::VectorXd::Zero(port_co_split_seq.size() * jointDof));
            // RTT::log(RTT::Error) << "in_torques_var: push_back JointTorques(" << port_co_split_seq.size() * jointDof << ")" << RTT::endlog();
            in_torques_var.push_back(Eigen::VectorXd::Zero(jointDof));
            RTT::log(RTT::Error) << "in_torques_var: push_back JointTorques(" << jointDof << ")" << RTT::endlog();

            in_inertia_var.push_back(Eigen::MatrixXd(jointDof, jointDof));
            RTT::log(RTT::Error) << "in_inertia_var: push_back MatrixXd(" << jointDof << ", " << jointDof << ")" << RTT::endlog();
        }
    }

    // For each robot
    for (std::size_t i = 0; i < rootControlObjectivesConfig.size(); i++)
    {
        YAML::Node entry = rootControlObjectivesConfig[i];
        YAML::Node robot = entry["Robot"];

        std::string robotName = robot.as<std::string>();

        // Create ports for Projection input of internal force TODO DLW
        YAML::Node amVM = entry["VM"];
        if (!amVM.IsNull() && amVM.IsDefined())
        {

            boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>> tmpPortVM = boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>>(new RTT::InputPort<Eigen::MatrixXd>());
            tmpPortVM->setName("in_" + robotName + "_P_intF");
            tmpPortVM->doc("Input port for reading the Internal Force Projection");
            ports()->addPort(*tmpPortVM);
            in_projection_internal_force_ports.push_back(tmpPortVM);
            in_projection_internal_force_flow = RTT::NoData;
            // in_projection_internal_force_var.push_back(Eigen::MatrixXd::Zero());

            boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>> tmpPortVMext = boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>>(new RTT::InputPort<Eigen::MatrixXd>());
            tmpPortVMext->setName("in_" + robotName + "_P_dot_intF");
            tmpPortVMext->doc("Input port for reading the Internal Force Projection Dot");
            ports()->addPort(*tmpPortVMext);
            in_projection_dot_internal_force_ports.push_back(tmpPortVMext);
            in_projection_dot_internal_force_flow = RTT::NoData;

            rcss[i].setPortIdforVMSpecificPorts(in_projection_dot_internal_force_ports.size() - 1);
        }

        if (rcss[i].amIaVM())
        {
            // YAML::Node amVM = entry["VM"];
            for (std::size_t d = 0; d < amVM.size(); d++)
            {
                std::string vmRname = amVM[d].as<std::string>();
                rcss[i].addRobot2CombinedRobots(vmRname);
                RTT::log(RTT::Error) << "RCS " << rcss[i].getRobotName() << " addRobot2CombinedRobots(" << vmRname << ")" << RTT::endlog();
            }
        }

        YAML::Node cos = entry["ControlObjectives"];
        // For each control objective of a specific robot
        for (std::size_t j = 0; j < cos.size(); j++)
        {
            std::string co_name = cos[j]["Name"].as<std::string>();
            // For each port
            for (std::size_t k = 0; k < prioComponent.size(); k++)
            {
                YAML::Node port = prioComponent[k];
                YAML::Node port_name = port["Name"];
                // Iterate over all control objectives and compare them
                YAML::Node port_co_split_seq = port["ControlObjectives"];
                unsigned int sumDofsTask = 0;
                unsigned int sumDofsJoints = 0;
                for (std::size_t l = 0; l < port_co_split_seq.size(); l++)
                {
                    std::string co_name_port = port_co_split_seq[l].as<std::string>();

                    if (co_name.compare(co_name_port) == 0)
                    {
                        // Add to map
                        unsigned int ret = rcss[i].getControlObjectiveTaskIndex(co_name);
                        if (ret == -1)
                        {
                            RTT::log(RTT::Error) << "Could not find " << co_name << " in " << rcss_index_name[i] << RTT::endlog();
                            return false;
                        }
                        rcss[i].addPortMapping(k, ret, sumDofsTask, sumDofsJoints, map_co_2_dofs[co_name], l);
                        RTT::log(RTT::Error) << "RCS " << rcss[i].getRobotName() << " addPortMapping(" << k << ", " << ret << ", " << sumDofsTask << ", " << sumDofsJoints << ", " << map_co_2_dofs[co_name] << ", "
                                             << l << ")" << RTT::endlog();
                    }

                    sumDofsTask += map_co_2_dofs[co_name];
                    sumDofsJoints += map_co_2_jointDofs[co_name];
                }
            }
        }
    }
    // YAML::Node filtersConfig = config["Filters"];
    // if (!filtersConfig.IsDefined() || filtersConfig.IsNull())
    // {
    //     RTT::log(RTT::Error) << "Filters is not defined properly!" << RTT::endlog();
    //     return false;
    // }

    std::map<std::string, unsigned int> map_real_robots_2_processingIndex;
    std::map<std::string, unsigned int> map_real_robots_2_startingIndex;
    // Define out_torques_var dimension based on jointDofs of real robots!
    unsigned int totalJointDofs = 0;
    unsigned int startingIndex = 0;
    // unsigned int index = 0;
    for (unsigned int i = 0; i < rcss.size(); i++)
    {
        if (!rcss[i].amIaVM())
        {
            map_real_robots_2_processingIndex[rcss[i].getRobotName()] = i;
            map_real_robots_2_startingIndex[rcss[i].getRobotName()] = startingIndex;
            // rcss[i].setStartingIndex(startingIndex);
            totalJointDofs += out_torquesProj_var[i].rows();
            // map_real_robots_2_processingIndex[rcss[i].getRobotName()] = index;
            // index++;
        }
        startingIndex += out_torquesProj_var[i].rows();
    }
    out_torques_var = Eigen::VectorXd::Zero(totalJointDofs);
    in_coriolisAndGravity_var = Eigen::VectorXd::Zero(totalJointDofs);
    in_robotstatus_var = sensor_msgs::JointState();
    in_robotstatus_pos_ = Eigen::VectorXd::Zero(totalJointDofs);
    in_robotstatus_vel_ = Eigen::VectorXd::Zero(totalJointDofs);
    in_robotstatus_trq_ = Eigen::VectorXd::Zero(totalJointDofs);
    for (unsigned int index_init_sensor_msg = 0; index_init_sensor_msg < totalJointDofs; index_init_sensor_msg++)
    {
        in_robotstatus_var.position.push_back(0.0);
        in_robotstatus_var.velocity.push_back(0.0);
        in_robotstatus_var.effort.push_back(0.0);
    }

    // check VM mapping
    for (unsigned int i = 0; i < rcss.size(); i++)
    {
        if (rcss[i].amIaVM())
        {
            std::vector<std::string> crNames = rcss[i].getCombinedRobotNames();
            for (std::size_t d = 0; d < crNames.size(); d++)
            {
                RTT::log(RTT::Error) << "map_real_robots_2_processingIndex[crNames[d]]: " << map_real_robots_2_processingIndex[crNames[d]] << RTT::endlog();
                RTT::log(RTT::Error) << "map_real_robots_2_startingIndex[crNames[d]]: " << map_real_robots_2_startingIndex[crNames[d]] << RTT::endlog();
                RTT::log(RTT::Error) << "out_torquesProj_var[map_real_robots_2_processingIndex[crNames[d]]]:\n"
                                     << out_torquesProj_var[map_real_robots_2_processingIndex[crNames[d]]] << RTT::endlog();

                rcss[i].addVM2RealMapping(crNames[d], map_real_robots_2_startingIndex[crNames[d]], out_torquesProj_var[map_real_robots_2_processingIndex[crNames[d]]].rows());
                RTT::log(RTT::Error) << "RCS " << rcss[i].getRobotName() << " addVM2RealMapping(" << crNames[d] << ", " << map_real_robots_2_startingIndex[crNames[d]] << ", " << out_torquesProj_var[map_real_robots_2_processingIndex[crNames[d]]].rows() << ")" << RTT::endlog();
            }
        }
    }

    for (unsigned int i = 0; i < rcss.size(); i++)
    {
        rcss[i].initGHC();
        if (rcss[i].amIaVM())
        {
            // get total joints to initialize the projection ports
            in_projection_internal_force_var.push_back(Eigen::MatrixXd::Zero(rcss[i].getTotalJoints(), rcss[i].getTotalJoints()));
            in_projection_dot_internal_force_var.push_back(Eigen::MatrixXd::Zero(rcss[i].getTotalJoints(), rcss[i].getTotalJoints()));
        }
    }

    // this->setNewTargetCS();

    this->preparePorts();
    return true;
}

bool DynamicTaskPrioritization::configureHookInternal()
{
    return true;
}

bool DynamicTaskPrioritization::startHookInternal()
{
    // for (unsigned int portNr = 0; portNr < numInputPorts; portNr++)
    // {
    //     if (!in_torques_ports[portNr]->connected())
    //     {
    //         RTT::log(RTT::Error) << "in_torques_port " << portNr << " is not connected" << RTT::endlog();
    //         return false;
    //     }
    // }

    // for (unsigned int portNr = 0; portNr < numInputPorts; portNr++)
    // {
    //     if (!in_jacobian_ports[portNr]->connected())
    //     {
    //         RTT::log(RTT::Error) << "in_jacobian_port " << portNr << " is not connected" << RTT::endlog();
    //         return false;
    //     }
    // }

    // // if (addCoriolisAndGravity)
    // // {
    // if (!in_coriolisAndGravity_port.connected())
    // {
    //     RTT::log(RTT::Error) << "in_coriolisAndGravity_port is not connected" << RTT::endlog();
    //     return false;
    // }
    // // }

    // if (!out_torques_port.connected())
    // {
    //     RTT::log(RTT::Error) << "out_torques_port is not connected" << RTT::endlog();
    //     return false;
    // }

    return true;
}

void DynamicTaskPrioritization::setNewTargetCS(const std::string &cs_name)
{
    this->new_cs_reached_ = false;
    this->out_new_cs_reached_port.write(this->new_cs_reached_);
    this->new_cs_timed_requested_ = 0.0;
    this->new_cs_name_requested_ = cs_name;
    this->out_new_cs_name_requested_port.write(this->new_cs_name_requested_);
    this->new_cs_requested_ = true;
    this->out_new_cs_requested_port.write(this->new_cs_requested_);

    // Ausgelagert und sync mit updatehookInternal
    // for (unsigned int i = 0; i < rcss.size(); i++)
    // {
    //     rcss[i].setNewTargetCS(cs_name);
    // }
}

void DynamicTaskPrioritization::activateContactSituationResetActivation(const std::string &cs_name, const double time_secs)
{
    this->new_cs_reached_ = false;
    this->out_new_cs_reached_port.write(this->new_cs_reached_);
    this->new_cs_timed_requested_ = time_secs;
    this->new_cs_name_requested_ = cs_name;
    this->out_new_cs_name_requested_port.write(this->new_cs_name_requested_);
    this->new_cs_requested_ = true;
    this->out_new_cs_requested_port.write(this->new_cs_requested_);

    // Ausgelagert und sync mit updatehookInternal
    // this->debug_pointintime = 0;
    // this->activation_inc_steps_ = 0;
    // if (time_secs > 0)
    // {
    //     // Calculate Ticks based on the current running freq.
    //     RTT::Seconds /* double */ _freq_in_secs = this->getPeriod();
    //     if (_freq_in_secs > 0) // Sanity check
    //     {
    //         this->activation_inc_steps_ = _freq_in_secs / time_secs;
    //     }
    // }
}

void DynamicTaskPrioritization::increaseActivation()
{
    if (this->activation_inc_steps_ != 0.0 && this->debug_pointintime < 1)
    {
        this->debug_pointintime += this->activation_inc_steps_;
    }

    if (this->debug_pointintime >= 1)
    {
        this->debug_pointintime = 1.0;
        // Deactivate the increase
        this->activation_inc_steps_ = 0.0;
        // Signal CS stable state reached
        this->new_cs_reached_ = true;
        this->out_new_cs_reached_port.write(this->new_cs_reached_);
    }
}

void DynamicTaskPrioritization::setStartCS(const std::string &cs_name)
{
    for (unsigned int i = 0; i < rcss.size(); i++)
    {
        RTT::log(RTT::Error) << "setStartCS for : " << i << " to " << cs_name << RTT::endlog();
        rcss[i].setStartCS(cs_name);
    }
}

// void DynamicTaskPrioritization::debugTestRandom()
// {
//     RTT::os::TimeService::nsecs start = RTT::os::TimeService::Instance()->getNSecs();

//     // - Name: motion_tracking_1
//     //   ControlObjectives: [RobotKukaLeft_motion_tracking, RobotKukaRight_motion_tracking]
//     in_torques_var[0].torques = Eigen::VectorXd::Random(7 * 2);
//     in_jacobian_var[0] = Eigen::MatrixXd::Random(6 * 2, 7 * 2);

//     // - Name: nullspace_tracking_1
//     //   ControlObjectives: [RobotKukaLeft_nullspace_tracking, RobotKukaRight_nullspace_tracking]
//     in_torques_var[1].torques = Eigen::VectorXd::Random(7 * 2);
//     in_jacobian_var[1].block(0, 0, 7, 7) = Eigen::MatrixXd::Identity(7, 7);
//     in_jacobian_var[1].block(7, 7, 7, 7) = Eigen::MatrixXd::Identity(7, 7);

//     // - Name: force_constraint_1
//     //   ControlObjectives: [RobotKukaLeft_force_constraint, RobotKukaRight_force_constraint]
//     in_torques_var[2].torques = Eigen::VectorXd::Random(7 * 2);
//     in_jacobian_var[2] = Eigen::MatrixXd::Random(6 * 2, 7 * 2);

//     // - Name: force_constraint_2
//     //   ControlObjectives: [BarManipulator_force_constraint]
//     in_torques_var[3].torques = Eigen::VectorXd::Random(14);
//     in_jacobian_var[3] = Eigen::MatrixXd::Random(6, 14);

//     // - Name: motion_tracking_2
//     //   ControlObjectives: [BarManipulator_motion_tracking]
//     in_torques_var[4].torques = Eigen::VectorXd::Random(14);
//     in_jacobian_var[4] = Eigen::MatrixXd::Random(6, 14);

//     // - Name: nullspace_tracking_2
//     //   ControlObjectives: [BarManipulator_nullspace_tracking1, BarManipulator_nullspace_tracking2]
//     in_torques_var[5].torques = Eigen::VectorXd::Random(14 * 2);
//     in_jacobian_var[5].block(0, 0, 14, 14) = Eigen::MatrixXd::Identity(14, 14);
//     in_jacobian_var[5].block(14, 14, 14, 14) = Eigen::MatrixXd::Identity(14, 14);

//     // Folding_SR_0
//     // Folding_SR_1
//     // Folding_CR_2
//     rcss[0].setNewTargetCS("Folding_SR_0"); // TODO set a default!
//     rcss[1].setNewTargetCS("Folding_SR_0");
//     rcss[2].setNewTargetCS("Folding_SR_0");

//     for (unsigned int portNr = 0; portNr < in_torques_ports.size(); portNr++)
//     {
//         for (unsigned int i = 0; i < rcss.size(); i++)
//         {
//             // RTT::log(RTT::Error) << "rcss " << i << " updateJacobian(" << portNr << ",\n" << in_jacobian_var[portNr] << ",\n" << in_torques_var[portNr] << ")" << RTT::endlog();
//             rcss[i].updateJacobian(portNr, in_jacobian_var[portNr], in_torques_var[portNr]);
//             if (i == 2) {
//                 rcss[i].updateInertia(portNr, Eigen::MatrixXd::Random(14, 14));
//             } else {
//                 rcss[i].updateInertia(portNr, Eigen::MatrixXd::Random(7, 7));
//             }
//         }
//     }

//     // rcss[0].updateInertia(Eigen::MatrixXd::Random(7, 7));
//     // rcss[1].updateInertia(Eigen::MatrixXd::Random(7, 7));
//     // rcss[2].updateInertia(Eigen::MatrixXd::Random(14, 14));

//     in_coriolisAndGravity_var = Eigen::VectorXd::Random(14);

//     // TODO REWRITE and also optimize to get rid of the multiple vector accesses.
//     out_torques_var.torques.setZero();
//     unsigned int steps = 0;
//     for (unsigned int i = 0; i < rcss.size(); i++)
//     {
//         // TODO is this working with the reference properly? out_torquesProj_var[i]
//         if (!rcss[i].amIaVM())
//         {
//             rcss[i].updateOutputTorques(debug_pointintime, out_torquesProj_var[i]);
//             out_torques_var.torques.segment(steps, out_torquesProj_var[i].torques.rows()) = out_torquesProj_var[i].torques;
//             // RTT::log(RTT::Error) << rcss[i].getRobotName() << " output:\n" << out_torques_var.torques << RTT::endlog();
//             steps += out_torquesProj_var[i].torques.rows();
//         }
//     }

//     for (unsigned int i = 0; i < rcss.size(); i++)
//     {
//         if (rcss[i].amIaVM())
//         {
//             // RTT::log(RTT::Error) << rcss[i].getRobotName() << " >> updateTorqueCommandFromCombinedRobots: " << out_torques_var.torques.rows() << "\n" << out_torques_var.torques << RTT::endlog();
//             rcss[i].updateTorqueCommandFromCombinedRobots(out_torques_var);
//             rcss[i].updateOutputTorquesVM(debug_pointintime, out_torques_var);
//         }
//     }

//     out_torques_port.write(out_torques_var); // + in_coriolisAndGravity_var;

//     RTT::os::TimeService::nsecs end = RTT::os::TimeService::Instance()->getNSecs(start);
//     RTT::log(RTT::Error) << 1e-6 * end << " milliseconds [" << this->getName() << "]" << RTT::endlog();

//     RTT::log(RTT::Error) << "Trq:\n"
//                          << out_torques_var << RTT::endlog();
// }

void DynamicTaskPrioritization::calculateDebug(const Eigen::MatrixXd &in_inertia, const Eigen::VectorXd &in_h, const Eigen::MatrixXd &in_jacobian, const Eigen::VectorXd &jointT, const Eigen::VectorXd &taskT, Eigen::VectorXd &out)
{
    // out = taskT;
    // return;
    // Eigen::VectorXd ttt = (10 * (pos_rest - in_robotstatus.angles) - 1 * in_robotstatus.velocities + in_h);

    // TODO DLW TEST
    int _dofsize = 7;
    int _numTasks = 2;

    //choose dimensionality of each task
    Eigen::VectorXd _tasksize;
    _tasksize = Eigen::VectorXd::Zero(_numTasks);
    //in this example "task0" and "task1" already consume all degress of freedom
    _tasksize[0] = 6;
    _tasksize[1] = 7;

    //initialize GHC class // DO NOT INITIALIZE HERE!!!!
    GHCProjections GP;
    GP = GHCProjections();
    GP.init(_numTasks, _tasksize, _dofsize);

    // RTT::log(RTT::Error) << "### 1 ###: A =\n" << GP.getAlphas() << RTT::endlog();

    std::vector<Eigen::MatrixXd> allJacobians;
    allJacobians.push_back(in_jacobian);
    allJacobians.push_back(Eigen::MatrixXd::Identity(7, 7));
    GP.setJacobianMatrices(allJacobians);

    GP.setInertiaMatrix(in_inertia);

    Eigen::VectorXd prioritiesVector;
    prioritiesVector = Eigen::VectorXd::Zero(0.5 * (_numTasks * _numTasks + _numTasks));
    //exampleA: strict hierachy with "task0" strict more important that "task1" and "task1" strict more important that "task2"
    prioritiesVector[0] = 0.0;
    prioritiesVector[1] = 0.0;
    prioritiesVector[2] = 0.0;

    int counter = 0;
    for (unsigned int i = 0; i < _numTasks; i++)
    {
        for (unsigned int j = i; j < _numTasks; j++)
        {
            if (prioritiesVector(counter) < 0)
            {
                std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector(counter) << " < 0" << RTT::endlog();
                // return;
            }
            if (prioritiesVector(counter) > 1)
            {
                std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector(counter) << " > 1" << RTT::endlog();
                // return;
            }
            GP.setAlphaIJ(i, j, prioritiesVector(counter));
            // RTT::log(RTT::Error) << "### 2 ######>: set i = " << i << ", j = " << j << " : " << prioritiesVector(counter) << RTT::endlog();
            counter++;
        }
    }

    // RTT::log(RTT::Error) << "### 3 ###: A =\n" << GP.getAlphas() << RTT::endlog();

    std::vector<Eigen::MatrixXd> allProjections;
    for (unsigned int i = 0; i < _numTasks; i++)
    {
        allProjections.push_back(Eigen::MatrixXd::Zero(_dofsize, _dofsize));
    }
    Eigen::VectorXd ranks;
    ranks = Eigen::VectorXd::Zero(_numTasks);
    bool ok = GP.getAllGeneralizedProjectors(allProjections, ranks);

    RTT::log(RTT::Error) << "inertiaInv =\n"
                         << GP.getInertiaMatrixInv() << RTT::endlog();
    RTT::log(RTT::Error) << "inertia =\n"
                         << in_inertia << RTT::endlog();
    RTT::log(RTT::Error) << "jac =\n"
                         << in_jacobian << RTT::endlog();
    RTT::log(RTT::Error) << "grav =\n"
                         << in_h << RTT::endlog();
    RTT::log(RTT::Error) << "nullspace =\n"
                         << allProjections[1] << RTT::endlog();
    RTT::log(RTT::Error) << "trq task =\n"
                         << taskT << RTT::endlog();
    RTT::log(RTT::Error) << "trq ns =\n"
                         << jointT << RTT::endlog();
    RTT::log(RTT::Error) << "nullspace * trq =\n"
                         << (allProjections[1] * jointT) << RTT::endlog();

    // Eigen::VectorXd ggg = jointT;
    // ggg.setZero();

    out = (allProjections[0] * taskT + allProjections[1] * (jointT)); // - in_h));

    RTT::log(RTT::Error) << "out w/o allP_0 =\n"
                         << (taskT + allProjections[1] * jointT) << RTT::endlog();
    RTT::log(RTT::Error) << "out =\n"
                         << out << RTT::endlog();
}

bool DynamicTaskPrioritization::isNewCSReached()
{
    return this->new_cs_reached_;
}

void DynamicTaskPrioritization::updateHookInternal()
{
    // Change Contact Situation in sync with updateHookInternal
    if (this->new_cs_requested_)
    {
        for (unsigned int i = 0; i < rcss.size(); i++)
        {
            rcss[i].setNewTargetCS(this->new_cs_name_requested_);
        }

        this->debug_pointintime = 0;
        this->activation_inc_steps_ = 0;
        if (this->new_cs_timed_requested_ > 0)
        {
            // Calculate Ticks based on the current running freq.
            RTT::Seconds /* double */ _freq_in_secs = this->getPeriod();
            if (_freq_in_secs > 0) // Sanity check
            {
                this->activation_inc_steps_ = _freq_in_secs / this->new_cs_timed_requested_;
            }
        }
        else
        {
            this->debug_pointintime = 1;
        }
        this->new_cs_requested_ = false;
    }

    // #ifdef TIMING
    // RTT::os::TimeService::nsecs start = RTT::os::TimeService::Instance()->getNSecs();

    in_coriolisAndGravity_flow = in_coriolisAndGravity_port.read(in_coriolisAndGravity_var);
    if (in_coriolisAndGravity_flow == RTT::NoData)
    {
        return;
    }
    else if (in_coriolisAndGravity_var.hasNaN())
    {
        RTT::log(RTT::Error) << "NAN in Projection for in_coriolisAndGravity_var " << RTT::endlog();
        return;
    }

    in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    if (in_robotstatus_flow == RTT::NoData)
    {
        return;
    }
    // Convert into Eigen:
    this->in_robotstatus_pos_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(in_robotstatus_var.position.data(), in_robotstatus_var.position.size());
    this->in_robotstatus_vel_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(in_robotstatus_var.velocity.data(), in_robotstatus_var.velocity.size());
    this->in_robotstatus_trq_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(in_robotstatus_var.effort.data(), in_robotstatus_var.effort.size());

    // #endif
    for (unsigned int portNr = 0; portNr < in_torques_ports.size(); portNr++)
    {
        in_torques_flow[portNr] = in_torques_ports[portNr]->read(in_torques_var[portNr]);
        if (in_torques_flow[portNr] == RTT::NoData)
        {
            return;
        }
        else if (in_torques_var[portNr].hasNaN())
        {
            RTT::log(RTT::Error) << "NAN in Projection for in_torques_var: " << in_torques_ports[portNr]->getName() << RTT::endlog();
            return;
        }

        in_jacobian_flow[portNr] = in_jacobian_ports[portNr]->read(in_jacobian_var[portNr]);
        if (in_jacobian_flow[portNr] == RTT::NoData)
        {
            return;
        }
        else if (in_jacobian_var[portNr].hasNaN())
        {
            RTT::log(RTT::Error) << "NAN in Projection for in_jacobian_var" << RTT::endlog();
            return;
        }

        if (in_inertia_ports[portNr]->connected())
        {
            in_inertia_flow[portNr] = in_inertia_ports[portNr]->read(in_inertia_var[portNr]);
            if (in_inertia_flow[portNr] == RTT::NoData)
            {
                return;
            }
            else if (in_inertia_var[portNr].hasNaN())
            {
                RTT::log(RTT::Error) << "NAN in Projection for in_inertia_var" << RTT::endlog();
                return;
            }
        }
        else
        {
            in_inertia_var[portNr].setIdentity();
        }

        // RTT::log(RTT::Error) << "Trying to feed content from ports " << portNr << ": " << in_torques_ports[portNr]->getName() << "!" << RTT::endlog();
        for (unsigned int i = 0; i < rcss.size(); i++)
        {
            // TODO DLW NUR IMMER DIE RCSS AUCH WIRKLICH BERECHNEN, DIE AUCH ZUMINDEST MINIMAL AKTIV SIND! ###############################
            // RTT::log(RTT::Error) << "Trying to update rcss[" << i << "] with jaq and inertia" << RTT::endlog();
            rcss[i].updateJacobian(portNr, in_jacobian_var[portNr], in_torques_var[portNr]);
            rcss[i].updateInertia(portNr, in_inertia_var[portNr]);
        }
    }

    for (unsigned int portNr = 0; portNr < in_projection_dot_internal_force_ports.size(); portNr++)
    {
        in_projection_internal_force_flow = in_projection_internal_force_ports[portNr]->read(in_projection_internal_force_var[portNr]);
        in_projection_dot_internal_force_flow = in_projection_dot_internal_force_ports[portNr]->read(in_projection_dot_internal_force_var[portNr]);
        if ((in_projection_internal_force_flow == RTT::NoData) || (in_projection_dot_internal_force_flow == RTT::NoData))
        {
            return;
        }

        for (unsigned int i = 0; i < rcss.size(); i++)
        {
            if (rcss[i].amIaVM(portNr))
            {
                rcss[i].updateInteralAndExternalProjection(in_projection_internal_force_var[portNr], in_projection_dot_internal_force_var[portNr]);
            }
        }
    }

    // calculateDebug(in_inertia_var[0], in_coriolisAndGravity_var.segment<7>(0), in_jacobian_var[0], in_torques_var[1].torques, in_torques_var[0].torques, out_torques_var.torques);
    // out_torques_port.write(out_torques_var);
    // RTT::os::TimeService::nsecs end = RTT::os::TimeService::Instance()->getNSecs(start);
    // RTT::log(RTT::Error) << 1e-6 * end << " milliseconds [" << this->getName() << "]" << RTT::endlog();

    // if (time_storage.size() < time_storage.capacity())
    // {
    //     time_storage.push_back(1e-6 * end);
    // }
    // return;

    // for (unsigned int i = 0; i < rcss.size(); i++)
    // {
    //     rcss[i].updateInertia(in_weighting_var[0]);
    // }

    // Timed switching of CS refs #32
    increaseActivation();

    // TODO REWRITE and also optimize to get rid of the multiple vector accesses.
    out_torques_var.setZero();
    unsigned int steps = 0;
    for (unsigned int i = 0; i < rcss.size(); i++)
    {
        // TODO is this working with the reference properly? out_torquesProj_var[i]
        if (!rcss[i].amIaVM())
        {
            // rcss[i].updateStatusAndGravity(in_robotstatus_var, in_coriolisAndGravity_var);
            rcss[i].updateOutputTorques(debug_pointintime, out_torquesProj_var[i]);
            out_torques_var.segment(steps, out_torquesProj_var[i].rows()) = out_torquesProj_var[i];
            // RTT::log(RTT::Error) << "segment[" << i << "](" << steps << ", " << out_torquesProj_var[i].rows() << ") =\n" << out_torquesProj_var[i] << RTT::endlog();
            steps += out_torquesProj_var[i].rows();
        }
    }

    if (this->debug_vm_on)
    {
        for (unsigned int i = 0; i < rcss.size(); i++)
        {
            if (rcss[i].amIaVM())
            {
                rcss[i].updateTorqueCommandFromCombinedRobots(out_torques_var, in_robotstatus_pos_, in_robotstatus_vel_, in_robotstatus_trq_, in_coriolisAndGravity_var);
                rcss[i].updateOutputTorquesVM(debug_pointintime, out_torques_var);
            }
        }
    }

    // if (in_robotstatus_flow != RTT::NoData)
    // {
    //     // TODO ALTERNATIVE CALCULATIONS
    //     this->alternateManualCalculationsDebug(in_robotstatus_var,
    //                                            in_jacobian_var[2],
    //                                            in_jacobian_var[0],
    //                                            //  Eigen::MatrixXd &in_projection,
    //                                            in_inertia_var[1],
    //                                            //  Eigen::MatrixXd &in_inertia_c,
    //                                            in_coriolisAndGravity_var,
    //                                            //    Eigen::MatrixXd & in_Pdot,
    //                                            in_torques_var[0],
    //                                            in_torques_var[2] /* TODO not sure about the index here! */,
    //                                            out_torquesss);
    // }

    // if (debug_switch_to_manual)
    // {
    //     out_torques_port.write(out_torquesss); // + in_coriolisAndGravity_var;
    // }
    // else
    // {
    if (debug_switch_to_manual)
    {
        out_torques_var += in_coriolisAndGravity_var;
        if (this->substract_gravity)
        {
            out_torques_var -= in_coriolisAndGravity_var;
        }
        out_torques_port.write(out_torques_var);
    }
    else
    {
        if (this->substract_gravity)
        {
            out_torques_var -= in_coriolisAndGravity_var;
        }
        out_torques_port.write(out_torques_var); // + in_coriolisAndGravity_var;
    }
    // }

    // TODO debug output ################################## KANN MAN SO NICHT MACHEN MEGA HACK!!! ################################## START
    // for (unsigned int i = 0; i < rcss.size(); i++)
    // {
    Eigen::MatrixXd out_alphas;
    rcss[0].getCurrentAlphas(out_alphas);
    out_current_alphas_port.write(out_alphas);
    out_current_projected_trq_cmds_port.write(rcss[0].torque_commands_proj_states_stacked);
    out_current_projection_matrices_port.write(rcss[0].allProjections_stacked);
    // }
    // (debug_pointintime);
    // TODO debug output ################################## KANN MAN SO NICHT MACHEN MEGA HACK!!! ################################## END

    // #ifdef TIMING
    // RTT::os::TimeService::nsecs end = RTT::os::TimeService::Instance()->getNSecs(start);
    // RTT::log(RTT::Error) << 1e-6 * end << " milliseconds [" << this->getName() << "]" << RTT::endlog();
    //
    // if (time_storage.size() < time_storage.capacity())
    // {
    //     time_storage.push_back(1e-6 * end);
    // }
    // #endif

    out_debug_pointintime_port.write(debug_pointintime);
}

void DynamicTaskPrioritization::stopHookInternal()
{
    size_t size = time_storage.size();
    double median = 0;

    if (size != 0)
    {
        sort(time_storage.begin(), time_storage.end());
        if (size % 2 == 0)
        {
            median = (time_storage[size / 2 - 1] + time_storage[size / 2]) / 2;
        }
        else
        {
            median = time_storage[size / 2];
        }
    }

    PRELOG(Error) << "MEDIAN: " << this->getName() << " = " << median << RTT::endlog();
    // stops the component (update hookInternal wont be  called anymore)
}

void DynamicTaskPrioritization::cleanupHookInternal()
{
    // cleaning the component data
    portsArePrepared = false;
}

// TODO get from configuration...! Because it differs!
void DynamicTaskPrioritization::setDOFsize(unsigned int DOFsize)
{
    // assert(DOFsize > 0);
    this->DOFsize = DOFsize;
    // this->identityDOFsizeDOFsize = Eigen::MatrixXd::Identity(DOFsize, DOFsize);
    // nullspaces.clear();
    // projections.clear();
    // for (unsigned int portNr = 0; portNr < numInputPorts; portNr++)
    // {
    //     nullspaces.push_back(Eigen::MatrixXd::Identity(DOFsize, DOFsize));
    //     projections.push_back(Eigen::MatrixXd::Identity(DOFsize, DOFsize));
    // }
}

void DynamicTaskPrioritization::preparePorts()
{
    // if (portsArePrepared)
    // {
    //     for (unsigned int portNr = 0; portNr < numInputPorts; portNr++)
    //     {
    //         ports()->removePort("in_torques" + boost::lexical_cast<std::string>(portNr) + "_port");
    //         ports()->removePort("in_jacobian" + boost::lexical_cast<std::string>(portNr) + "_port");
    //         ports()->removePort("in_activation" + boost::lexical_cast<std::string>(portNr) + "_port");
    //         ports()->removePort("in_weighting" + boost::lexical_cast<std::string>(portNr) + "_port");
    //     }
    //     ports()->removePort("in_inertiaInv_port");
    //     ports()->removePort("in_coriolisAndGravity_port");
    //     ports()->removePort("out_torques_port");
    //     for (unsigned int portNr = 0; portNr < numInputPorts; portNr++)
    //     {
    //         ports()->removePort("out_torquesProj" + boost::lexical_cast<std::string>(portNr) + "_port");
    //     }
    // }

    //prepare input
    // in_torques_var.clear();
    // in_torques_flow.clear();
    // in_torques_ports.clear();
    // for (unsigned int portNr = 0; portNr < numInputPorts; portNr++)
    // {
    //     in_torques_var.push_back(Eigen::VectorXd::Zero(DOFsize));
    //     in_torques_var[portNr].torques.setZero();
    //     boost::shared_ptr<RTT::InputPort<Eigen::VectorXd>> tmpPort1(new RTT::InputPort<Eigen::VectorXd>());
    //     tmpPort1->setName("in_torques" + boost::lexical_cast<std::string>(portNr) + "_port");
    //     tmpPort1->doc("Input port for torque values");
    //     ports()->addPort(*tmpPort1);
    //     in_torques_flow.push_back(RTT::NoData);
    //     in_torques_ports.push_back(tmpPort1);
    // }

    // in_jacobian_var.clear();
    // in_jacobian_flow.clear();
    // in_jacobian_ports.clear();
    // for (unsigned int portNr = 0; portNr < numInputPorts; portNr++)
    // {
    //     in_jacobian_var.push_back(Eigen::MatrixXd::Zero(Tasksize.at(portNr), DOFsize));
    //     boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>> tmpPort2(new RTT::InputPort<Eigen::MatrixXd>());
    //     tmpPort2->setName("in_jacobian" + boost::lexical_cast<std::string>(portNr) + "_port");
    //     tmpPort2->doc("Input port for jacobian matrix");
    //     ports()->addPort(*tmpPort2);
    //     in_jacobian_flow.push_back(RTT::NoData);
    //     in_jacobian_ports.push_back(tmpPort2);
    // }

    // in_weighting_var.clear();
    // in_weighting_flow.clear();
    // in_weighting_ports.clear();
    // for (unsigned int portNr = 0; portNr < numInputPorts; portNr++)
    // {
    //     in_weighting_var.push_back(Eigen::MatrixXd::Identity(DOFsize, DOFsize));
    //     boost::shared_ptr<RTT::InputPort<Eigen::MatrixXd>> tmpPort3(new RTT::InputPort<Eigen::MatrixXd>());
    //     tmpPort3->setName("in_weighting" + boost::lexical_cast<std::string>(portNr) + "_port");
    //     tmpPort3->doc("Input port for weighting matrix");
    //     ports()->addPort(*tmpPort3);
    //     in_weighting_flow.push_back(RTT::NoData);
    //     in_weighting_ports.push_back(tmpPort3);
    // }

    in_coriolisAndGravity_var.setZero();
    in_coriolisAndGravity_port.setName("in_coriolisAndGravity_port");
    in_coriolisAndGravity_port.doc("Input port for reading coriolis and gravity vector");
    ports()->addPort(in_coriolisAndGravity_port);
    in_coriolisAndGravity_flow = RTT::NoData;

    for (unsigned int index_robotstatus = 0; index_robotstatus < in_robotstatus_var.position.size(); index_robotstatus++)
    {
        in_robotstatus_var.position[index_robotstatus] = 0.0;
        in_robotstatus_var.velocity[index_robotstatus] = 0.0;
        in_robotstatus_var.effort[index_robotstatus] = 0.0;
    }
    in_robotstatus_pos_.setZero();
    in_robotstatus_vel_.setZero();
    in_robotstatus_trq_.setZero();
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for reading the current robot status");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    //prepare output
    // out_torques_var = Eigen::VectorXd::Zero(DOFsize);
    out_torques_var.setZero();
    out_torques_port.setName("out_torques_port");
    out_torques_port.doc("Output port for sending torque values");
    out_torques_port.setDataSample(out_torques_var);
    ports()->addPort(out_torques_port);

    // ################### TODO MEGA HACK KANN MAN SO NICHT MACHEN!!! ###################
    out_current_alphas_port.setName("out_current_alphas_port");
    out_current_alphas_port.doc("???");
    Eigen::MatrixXd tmpAlphasM;
    rcss[0].getCurrentAlphas(tmpAlphasM);
    RTT::log(RTT::Fatal) << "tmpAlphasM rows = " << tmpAlphasM.rows() << ", cols = " << tmpAlphasM.cols() << ":\n" << tmpAlphasM << RTT::endlog();
    out_current_alphas_port.setDataSample(tmpAlphasM);
    ports()->addPort(out_current_alphas_port);

    out_current_projected_trq_cmds_port.setName("out_current_projected_trq_cmds_port");
    out_current_projected_trq_cmds_port.doc("???");
    out_current_projected_trq_cmds_port.setDataSample(rcss[0].torque_commands_proj_states_stacked);
    ports()->addPort(out_current_projected_trq_cmds_port);

    out_current_projection_matrices_port.setName("out_current_projection_matrices_port");
    out_current_projection_matrices_port.doc("???");
    out_current_projection_matrices_port.setDataSample(rcss[0].allProjections_stacked);
    ports()->addPort(out_current_projection_matrices_port);
    // ################### TODO MEGA HACK KANN MAN SO NICHT MACHEN!!! ###################

    // out_torquesProj_var.clear();
    // out_torquesProj_ports.clear();
    // for (unsigned int portNr = 0; portNr < numInputPorts; portNr++)
    // {
    //     out_torquesProj_var.push_back(Eigen::VectorXd::Zero(DOFsize));
    //     out_torquesProj_var[portNr].torques.setZero();
    //     boost::shared_ptr<RTT::OutputPort<Eigen::VectorXd>> tmpPort4(new RTT::OutputPort<Eigen::VectorXd>());
    //     tmpPort4->setName("out_torquesProj" + boost::lexical_cast<std::string>(portNr) + "_port");
    //     tmpPort4->doc("Output port for torque values");
    //     tmpPort4->setDataSample(out_torquesProj_var[portNr]);
    //     ports()->addPort(*tmpPort4);
    //     out_torquesProj_ports.push_back(tmpPort4);
    // }

    portsArePrepared = true;
}

void DynamicTaskPrioritization::alternateManualCalculationsDebug(Eigen::VectorXd &in_robotstatus_pos,
                                                                 Eigen::VectorXd &in_robotstatus_vel,
                                                                 Eigen::VectorXd &in_robotstatus_trq,
                                                                 const Eigen::MatrixXd &in_jToBeCstr,
                                                                 const Eigen::MatrixXd &in_jToBeMoved,
                                                                 //  Eigen::MatrixXd &in_projection,
                                                                 Eigen::MatrixXd &in_inertia,
                                                                 //  Eigen::MatrixXd &in_inertia_c,
                                                                 Eigen::VectorXd &in_h,
                                                                 //  Eigen::MatrixXd &in_Pdot,
                                                                 Eigen::VectorXd &in_torquesTask,
                                                                 Eigen::VectorXd &in_torquesCstr,
                                                                 Eigen::VectorXd &out_torques)
{
    MatrixDecomposition matDecomp = MatrixDecomposition();
    matDecomp.epsilon = 1.0e-06;
    matDecomp.epsilon = 0.01;
    matDecomp.borderA = 0.1;
    matDecomp.borderB = 0.01;
    matDecomp.changeRows = false;
    matDecomp.useAauto = true;
    matDecomp.useAuser = true;
    double regfactor = 0.0001;

    Eigen::MatrixXd jc = in_jToBeCstr;
    // jc.row(2).setZero();

    // RTT::log(RTT::Error) << "in_jToBeCstr (" << in_jToBeCstr.rows() << ", " << in_jToBeCstr.cols() << ") =\n"
    //                      << in_jToBeCstr << RTT::endlog();
    // RTT::log(RTT::Error) << "jc (" << jc.rows() << ", " << jc.cols() << ") =\n"
    //                      << jc << RTT::endlog();
    // RTT::log(RTT::Error) << "in_inertia (" << in_inertia.rows() << ", " << in_inertia.cols() << ") =\n"
    //                      << in_inertia << RTT::endlog();
    // RTT::log(RTT::Error) << "in_h (" << in_h.rows() << ", " << in_h.cols() << ") =\n"
    //                      << in_h << RTT::endlog();
    // RTT::log(RTT::Error) << "in_torquesTask =\n"
    //                      << in_torquesTask.torques << RTT::endlog();
    // RTT::log(RTT::Error) << "in_torquesCstr =\n"
    //                      << in_torquesCstr.torques << RTT::endlog();

    Eigen::VectorXd Aauto = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd activation = Eigen::VectorXd::Ones(6);
    if (!this->debug_activeee)
    {
        jc.setZero();
        activation.setZero();
    }

    Eigen::MatrixXd out_P = Eigen::MatrixXd::Zero(7, 7);
    Eigen::MatrixXd out_Pdot = Eigen::MatrixXd::Zero(7, 7);

    int out_rankSVD_var = -1;

    matDecomp.computeSVDnullspace(jc, activation, out_P, out_rankSVD_var, Aauto);

    // Calculate dP = ( P(t) - P(t-1) ) / dt
    double end_time = (1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks()));

    double dt = end_time - start_time;
    if (previousP.isZero())
    {
        start_time = (1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks()));
        previousP = out_P;
    }
    else if (dt < 0.0001)
    {
    }
    else
    {
        // time-derivative of projection
        out_Pdot = (out_P - previousP) / dt;
    }
    start_time = end_time;
    previousP = out_P;

    //Compute constrained inertia (Eq. under Eq. 11 in Valerio's paper)
    Eigen::MatrixXd out_MCstr = out_P * in_inertia + Eigen::MatrixXd::Identity(7, 7) - out_P;
    //RTT::log(RTT::Error)<<this->getName()<<" Debug4"<<RTT::endlog();
    //Compute constrained inertia inverse times projection
    Eigen::MatrixXd out_MCstrInv = out_MCstr.inverse();
    //RTT::log(RTT::Error)<<this->getName()<<" Debug5"<<RTT::endlog();
    //Compute constrained inertia inverse times projection
    Eigen::MatrixXd out_MCstrInvP = out_MCstrInv * out_P;

    // NULLSPACE FOR THE MOTION PART
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_solver_lambda_c_global_;
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singular_values_lambda_c_global_;
    svd_solver_lambda_c_global_ = Eigen::JacobiSVD<Eigen::MatrixXd>(6, 6);
    singular_values_lambda_c_global_.resize(6);

    Eigen::MatrixXd lambda_global_ = (in_jToBeMoved * out_MCstrInvP * in_jToBeMoved.transpose());
    svd_solver_lambda_c_global_.compute(lambda_global_, Eigen::ComputeFullU | Eigen::ComputeFullV);

    singular_values_lambda_c_global_ = svd_solver_lambda_c_global_.singularValues();
    for (int i = 0; i < singular_values_lambda_c_global_.size(); i++)
    {
        if (singular_values_lambda_c_global_(i) < 1.e-06)
        {
            singular_values_lambda_c_global_(i) = 0;
        }
        else
        {
            singular_values_lambda_c_global_(i) = 1 / singular_values_lambda_c_global_(i);
        }
    }

    lambda_global_ = svd_solver_lambda_c_global_.matrixV().leftCols(singular_values_lambda_c_global_.size()) * singular_values_lambda_c_global_.asDiagonal() * svd_solver_lambda_c_global_.matrixU().leftCols(singular_values_lambda_c_global_.size()).transpose();
    Eigen::MatrixXd nullspace = (Eigen::MatrixXd::Identity(7, 7) - (in_jToBeMoved.transpose() * lambda_global_ * in_jToBeMoved * out_MCstrInvP));
    in_torquesTask += nullspace * in_h;

    // PROJECTION PART!

    // out_torquesMotion.torques = in_torquesTask.torques; // 1 MOTION
    // if (add_h_vector)
    // {
    //     in_torquesTask.torques += in_h; // 2 MOTION
    // }

    in_torquesCstr += in_h; // 1 FORCE
    if (this->debug_activeee)
    {
        RTT::log(RTT::Error) << "1! in_torquesCstr =\n"
                             << in_torquesCstr << RTT::endlog();
    }

    in_torquesCstr += in_inertia * out_MCstr.inverse() * (out_P * in_torquesTask - out_P * in_h + out_Pdot * in_robotstatus_vel);
    if (this->debug_activeee)
    {
        RTT::log(RTT::Error) << "2! in_torquesCstr =\n"
                             << in_torquesCstr << RTT::endlog();
    }

    in_torquesTask = out_P * in_torquesTask; // 3 MOTION
    // debug_tau_task = in_torquesTask;

    in_torquesCstr = (Eigen::MatrixXd::Identity(7, 7) - out_P) * in_torquesCstr;
    if (this->debug_activeee)
    {
        RTT::log(RTT::Error) << "3! out_P =\n"
                             << out_P << RTT::endlog();
        RTT::log(RTT::Error) << "3! I-out_P =\n"
                             << (Eigen::MatrixXd::Identity(7, 7) - out_P) << RTT::endlog();
        RTT::log(RTT::Error) << "3! in_torquesCstr =\n"
                             << in_torquesCstr << RTT::endlog();
    }
    // debug_tau_cstr = in_torquesCstr;

    out_torques = in_torquesTask + in_torquesCstr; //(Eigen::MatrixXd::Identity(7,7) - in_projection) * (in_torquesCstr.torques + in_h); //out_torquesForce.torques;

    // if (this->isnanVector(out_torques.torques) || this->isinfVector(out_torques.torques))
    // {
    //     // PRELOG(Error) << "NAN or INF? h = " << in_h << RTT::endlog();
    //     if (!last_nan_inf_detected)
    //     {
    //         PRELOG(Critical) << "Detected NAN or INF torques! Trying to recover by sending h-vector!" << RTT::endlog();
    //     }
    //     last_nan_inf_detected = true;
    //     // out_torques.torques.setZero();
    //     out_torques.torques = in_h;
    //     // out_torquesMotion.torques.setZero();
    //     out_torquesMotion.torques = in_h;
    //     out_torquesForce.torques.setZero();
    // }
    // else
    // {
    //     if (last_nan_inf_detected)
    //     {
    //         PRELOG(Warning) << "Everything back to normal!" << RTT::endlog();
    //     }
    //     last_nan_inf_detected = false;
    // }
}

void DynamicTaskPrioritization::displayCurrentState()
{
    PRELOG(Error) << "############## DynamicTaskPrioritization State begin " << RTT::endlog();
    for (unsigned int portNr = 0; portNr < in_torques_ports.size(); portNr++)
    {
        PRELOG(Error) << " in_torques_var[" << portNr << "] \n"
                  << in_torques_var[portNr] << RTT::endlog();
    }

    for (unsigned int portNr = 0; portNr < in_torques_ports.size(); portNr++)
    {
        PRELOG(Error) << " in_jacobian_var[" << portNr << "] \n"
                  << in_jacobian_var[portNr] << RTT::endlog();
    }

    for (unsigned int portNr = 0; portNr < in_torques_ports.size(); portNr++)
    {
        PRELOG(Error) << " in_inertia_var[" << portNr << "] \n"
                  << in_inertia_var[portNr] << RTT::endlog();
    }

    PRELOG(Error) << " in_coriolisAndGravity_var \n"
              << in_coriolisAndGravity_var << RTT::endlog();

    PRELOG(Error) << " out_torques_var \n"
              << out_torques_var << RTT::endlog();

    PRELOG(Error) << " out_torques_var size = " <<  out_torques_var.size() << ", out_torquesProj_var size = " <<  out_torquesProj_var.size() << RTT::endlog();
    for (unsigned int portNr = 0; portNr < out_torquesProj_var.size(); portNr++)
    {
        PRELOG(Error) << " out_torquesProj_var[" << portNr << "] \n"
                  << out_torquesProj_var[portNr] << RTT::endlog();
    }

    for (unsigned int portNr = 0; portNr < nullspaces.size(); portNr++)
    {
        PRELOG(Error) << " nullspaces[" << portNr << "] \n"
                  << nullspaces[portNr] << RTT::endlog();
    }

    for (unsigned int portNr = 0; portNr < projections.size(); portNr++)
    {
        PRELOG(Error) << " projections[" << portNr << "] \n"
                  << projections[portNr] << RTT::endlog();
    }

    // for (unsigned int portNr = 0; portNr < in_torques_ports.size(); portNr++)
    // {
    //     PRELOG(Error) << "original torques [" << portNr << "] \n"
    //               << in_torques_var[portNr] << RTT::endlog();
    //     PRELOG(Error) << "projected torques [" << portNr << "] \n"
    //               << out_torquesProj_var[portNr] << RTT::endlog();
    // }

    PRELOG(Error) << "############## DynamicTaskPrioritization State end " << RTT::endlog();
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::prioritization::DynamicTaskPrioritization)
