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

#include "../../include/cosima-controller/task/rtt_task_describer_synthesis.hpp"

#include <fstream>
#include <iostream>

using namespace cosima;
using namespace task;

TaskDescriberSynthesis::TaskDescriberSynthesis(std::string const &name) : RTT::TaskContext(name)
{
    //prepare operations
    // addOperation("setDOFsize", &TaskDescriberSynthesis::setDOFsize, this).doc("set DOF size");
    addOperation("displayCurrentState", &TaskDescriberSynthesis::displayCurrentState, this).doc("print current state");
    addOperation("loadYAMLConfig", &TaskDescriberSynthesis::loadYAMLConfig, this);

    addOperation("writeMappingConfigurationToFile", &TaskDescriberSynthesis::writeMappingConfigurationToFile, this);

    addOperation("activateContactSituation", &TaskDescriberSynthesis::activateContactSituation, this);
    addOperation("activateContactSituationResetActivation", &TaskDescriberSynthesis::activateContactSituationResetActivation, this);
    // addOperation("loadModel", &TaskDescriberSynthesis::loadModel, this).doc("load model");
    // addOperation("addChain", &TaskDescriberSynthesis::addChain, this).doc("add chain");
    // addOperation("addChainWithWorldOffset", &TaskDescriberSynthesis::addChainWithWorldOffset, this).doc("add chain with world offset");

    addOperation("isNewCSReached", &TaskDescriberSynthesis::isNewCSReached, this);

    debug_pointintime = 0;
    addProperty("debug_pointintime", debug_pointintime);

    portsArePrepared = false;

    // gravity_vectorKDL = KDL::Vector();
    // gravity_vectorKDL.data[0] = 0.0;
    // gravity_vectorKDL.data[1] = 0.0;
    // gravity_vectorKDL.data[2] = -9.81;

    // time_storage.reserve(100000);
    // time_storage_robot_containers.reserve(100000);
    // time_storage_vm.reserve(100000);
    // time_storage_co_time.reserve(100000);
    // time_storage_co_gemini.reserve(100000);
    // time_storage_send.reserve(100000);

    this->activation_inc_steps_ = 0;

    this->new_cs_requested_ = false;
    this->new_cs_name_requested_ = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
    this->new_cs_timed_requested_ = 0.0;
    this->new_cs_reached_ = true; // ?

    out_debug_pointintime_port.setName("out_debug_pointintime_port");
    out_debug_pointintime_port.doc("Output port for sending the current point in time for the transition");
    out_debug_pointintime_port.setDataSample(debug_pointintime);
    ports()->addPort(out_debug_pointintime_port);
}

bool TaskDescriberSynthesis::loadYAMLConfig(const std::string &file)
{
    YAML::Node config = YAML::LoadFile(file);

    //////////////////
    // GENERAL PART //
    //////////////////

    YAML::Node rootControlObjectivesConfig = config["RootControlObjectives"];
    if (!rootControlObjectivesConfig.IsDefined() || rootControlObjectivesConfig.IsNull())
    {
        RTT::log(RTT::Error) << "RootControlObjectives is not defined properly!" << RTT::endlog();
        return false;
    }

    /////////////////////////////////////////////////
    // PARSING PORTS IN FORM OF CONTROL COMPONENTS //
    /////////////////////////////////////////////////
    std::map<std::string, std::shared_ptr<RobotContainer>> _map_robot;
    std::map<std::string, std::shared_ptr<VMContainer>> _map_vms;
    std::map<std::string, unsigned int> _map_robot_2_jointdof;
    // std::map<std::string, std::pair<unsigned int, unsigned int>> _map_robot_2_start_task_joint_dof;

    unsigned int totalRealRobotJointSpace = 0;
    unsigned int totalRealRobotTaskSpace = 0;
    unsigned int totalRealRobotTaskPoseQuaternion = 0;
    for (std::size_t i = 0; i < rootControlObjectivesConfig.size(); i++)
    {
        YAML::Node entry = rootControlObjectivesConfig[i];

        YAML::Node amVM = entry["VM"];

        YAML::Node robot_entry = entry["Robot"];
        std::string robotName = robot_entry.as<std::string>();
        unsigned int jointDOF = entry["JointDof"].as<unsigned int>();

        _map_robot_2_jointdof[robotName] = jointDOF;

        if (!amVM.IsDefined() || amVM.IsNull())
        {
            std::shared_ptr<RobotContainer> robot = std::shared_ptr<RobotContainer>(new RobotContainer(robotName));
            robot->setJacobianPositionInGlobalJacobian(totalRealRobotTaskSpace, totalRealRobotJointSpace, 6, jointDOF);
            robot->setInertiaPositionInGlobalJacobian(totalRealRobotJointSpace, totalRealRobotJointSpace, jointDOF, jointDOF);
            robot->setGCPositionInGlobalJacobian(totalRealRobotJointSpace, jointDOF);
            robot->setCartPosInGlobal(totalRealRobotTaskPoseQuaternion);

            // _map_robot_2_start_task_joint_dof[robotName] = std::make_pair(totalRealRobotTaskSpace, totalRealRobotJointSpace);
            totalRealRobotJointSpace += jointDOF;
            totalRealRobotTaskSpace += 6;
            totalRealRobotTaskPoseQuaternion += 7;

            vec_robots_.push_back(robot);
            _map_robot[robotName] = robot;
        }
    }

    //////////////////////////////////////////
    // IN THE MEAN TIME PARSING ALL THE VMs //
    //////////////////////////////////////////
    for (std::size_t i = 0; i < rootControlObjectivesConfig.size(); i++)
    {
        YAML::Node entry = rootControlObjectivesConfig[i];
        YAML::Node amVM = entry["VM"];

        if (amVM.IsDefined() && !amVM.IsNull())
        {
            YAML::Node robot_entry = entry["Robot"];
            std::string robotName = robot_entry.as<std::string>();
            unsigned int jointDOF = entry["JointDof"].as<unsigned int>();

            // TODO change to suitable/desired object! GeometryType
            std::shared_ptr<VMContainer> vm_inst = std::shared_ptr<VMContainer>(new VMContainer(robotName, GeometryType::BOX));

            unsigned int totalTaskDofsRobot = 0;
            unsigned int totalJointDofsRobot = 0;
            // Go through all the real robots and find the robots in the order that is specified in the VM
            for (unsigned int rc = 0; rc < vec_robots_.size(); rc++)
            {
                std::shared_ptr<RobotContainer> rob = vec_robots_[rc];
                for (std::size_t d = 0; d < amVM.size(); d++)
                {
                    std::string vmRname = amVM[d].as<std::string>();
                    if (rob->getRobotName().compare(vmRname) == 0)
                    {
                        totalTaskDofsRobot += rob->getTaskAndJointDof().first;
                        totalJointDofsRobot += rob->getTaskAndJointDof().second;
                        break;
                    }
                }
            }

            vm_inst->initializeDimensions(totalTaskDofsRobot, totalJointDofsRobot);

            // Initialize the total Task and Joint Dofs of the combined real robots and the single ones for each real robot individually.
            unsigned int countRobotTaskDof = 0;
            unsigned int countRobotJointDof = 0;
            unsigned int countRobotTaskPoseQuaternion = 0;
            for (unsigned int rc = 0; rc < vec_robots_.size(); rc++)
            {
                std::shared_ptr<RobotContainer> rob = vec_robots_[rc];
                for (std::size_t d = 0; d < amVM.size(); d++)
                {
                    std::string vmRname = amVM[d].as<std::string>();
                    if (rob->getRobotName().compare(vmRname) == 0)
                    {
                        unsigned int taskd = rob->getTaskAndJointDof().first;
                        unsigned int jointd = rob->getTaskAndJointDof().second;

                        vm_inst->addRealRobot(rob,
                                              countRobotTaskDof,
                                              countRobotJointDof,
                                              taskd,
                                              jointd,
                                              countRobotJointDof,
                                              countRobotJointDof,
                                              jointd,
                                              jointd,
                                              countRobotJointDof,
                                              jointd,
                                              countRobotTaskPoseQuaternion);

                        countRobotTaskDof += taskd;
                        countRobotJointDof += jointd;
                        countRobotTaskPoseQuaternion += 7;
                        break;
                    }
                }
            }
            vm_inst->initializeAssociatedRobotRelations();

            _map_vms[robotName] = vm_inst;
            vec_vms_.push_back(vm_inst);
        }
    }

    // Create (from Fkin) Input Ports
    in_J_var = Eigen::MatrixXd::Zero(totalRealRobotTaskSpace, totalRealRobotJointSpace);
    in_J_Input_port.setName("in_jacobian_port");
    in_J_Input_port.doc("");
    ports()->addPort(in_J_Input_port);
    J_Flow = RTT::NoData;

    in_J_Dot_var = Eigen::MatrixXd::Zero(totalRealRobotTaskSpace, totalRealRobotJointSpace);
    in_J_Dot_Input_port.setName("in_jacobian_dot_port");
    in_J_Dot_Input_port.doc("");
    ports()->addPort(in_J_Dot_Input_port);
    J_Dot_Flow = RTT::NoData;

    in_M_var = Eigen::MatrixXd::Zero(totalRealRobotJointSpace, totalRealRobotJointSpace);
    in_M_Input_port.setName("in_inertia_port");
    in_M_Input_port.doc("");
    ports()->addPort(in_M_Input_port);
    M_Flow = RTT::NoData;

    in_GC_var = Eigen::VectorXd::Zero(totalRealRobotJointSpace);
    in_GC_Input_port.setName("in_gc_port");
    in_GC_Input_port.doc("");
    ports()->addPort(in_GC_Input_port);
    GC_Flow = RTT::NoData;

    in_robotstatus_var = sensor_msgs::JointState();
    this->in_robotstatus_pos_ = Eigen::VectorXd::Zero(totalRealRobotJointSpace);
    this->in_robotstatus_vel_ = Eigen::VectorXd::Zero(totalRealRobotJointSpace);
    this->in_robotstatus_trq_ = Eigen::VectorXd::Zero(totalRealRobotJointSpace);
    for (unsigned int index_robotstatus = 0; index_robotstatus < totalRealRobotJointSpace; index_robotstatus++)
    {
        in_robotstatus_var.position.push_back(0.0);
        in_robotstatus_var.velocity.push_back(0.0);
        in_robotstatus_var.effort.push_back(0.0);
    }
    in_robotstatus_Input_port.setName("in_robotstatus_port");
    in_robotstatus_Input_port.doc("");
    ports()->addPort(in_robotstatus_Input_port);
    robotstatus_Flow = RTT::NoData;

    in_CartPos_var = Eigen::VectorXd(totalRealRobotTaskPoseQuaternion);
    in_CartPos_Input_port.setName("in_CartPos_port");
    in_CartPos_Input_port.doc("");
    ports()->addPort(in_CartPos_Input_port);
    CartPos_Flow = RTT::NoData;

    in_CartVel_var = Eigen::VectorXd(totalRealRobotTaskSpace);
    in_CartVel_Input_port.setName("in_CartVel_port");
    in_CartVel_Input_port.doc("");
    ports()->addPort(in_CartVel_Input_port);
    CartVel_Flow = RTT::NoData;

    ////////////////////////////////
    // GET ALL CONTROL OBJECTIVES //
    ////////////////////////////////
    std::map<std::string, std::shared_ptr<ControlObjectiveContainer>> _map_cos;

    for (std::size_t i = 0; i < rootControlObjectivesConfig.size(); i++)
    {
        YAML::Node entry = rootControlObjectivesConfig[i];
        YAML::Node robot = entry["Robot"];
        std::string robotName = robot.as<std::string>();
        unsigned int jointDOF = entry["JointDof"].as<unsigned int>();

        YAML::Node amVM = entry["VM"];
        bool amIaVM = true;
        if (!amVM.IsDefined() || amVM.IsNull())
        {
            amIaVM = false;
        }

        YAML::Node cos = entry["ControlObjectives"];
        RTT::log(RTT::Error) << " Amount of CO " << cos.size() << RTT::endlog();

        for (std::size_t j = 0; j < cos.size(); j++)
        {
            std::string co_name = cos[j]["Name"].as<std::string>();
            unsigned int taskdof = cos[j]["OperationalFrame"]["Dof"].as<unsigned int>();
            std::string frame_name = cos[j]["OperationalFrame"]["Name"].as<std::string>();
            std::string type = cos[j]["OperationalFrame"]["Type"].as<std::string>();
            // RTT::log(RTT::Error) << " - " << co_name << ", DoF: " << taskdof << ", type: " << type << RTT::endlog();
            std::shared_ptr<ControlObjectiveContainer> co = std::shared_ptr<ControlObjectiveContainer>(new ControlObjectiveContainer(co_name));
            co->setOperationalFrame(frame_name, type, taskdof, jointDOF);
            co->setIamVM(amIaVM);
            // RTT::log(RTT::Error) << "RCS: " << robotName << " addControlObjectiveTask(" << co_name << ", " << j << ", " << taskdof << ")" << RTT::endlog();

            if (!amIaVM)
            {
                co->initializeDimensions(taskdof, jointDOF);
                co->setAssociatedRobot(_map_robot[robotName]);
                // co->addRealRobot(_map_robot[robotName],
                //                  0,
                //                  0,
                //                  6,
                //                  jointDOF,
                //                  0,
                //                  0,
                //                  jointDOF,
                //                  jointDOF,
                //                  0,
                //                  jointDOF);
            }
            else
            {
                co->initializeDimensions(taskdof, jointDOF);
                co->setAssociatedRobot(_map_vms[robotName]);
                // // get all involved robots in global order (Needs to be this way otherwise the optimization in addJMG do not work!)
                // unsigned int totalTaskDofsRobot = 0;
                // unsigned int totalJointDofsRobot = 0;
                // for (unsigned int rc = 0; rc < vec_robots_.size(); rc++)
                // {
                //     std::shared_ptr<RobotContainer> rob = vec_robots_[rc];
                //     for (std::size_t d = 0; d < amVM.size(); d++)
                //     {
                //         std::string vmRname = amVM[d].as<std::string>();
                //         if (rob->getRobotName().compare(vmRname) == 0)
                //         {
                //             totalTaskDofsRobot += rob->getTaskAndJointDof().first;
                //             totalJointDofsRobot += rob->getTaskAndJointDof().second;
                //             break;
                //         }
                //     }
                // }
                // unsigned int countRobotTaskDof = 0;
                // unsigned int countRobotJointDof = 0;
                // for (unsigned int rc = 0; rc < vec_robots_.size(); rc++)
                // {
                //     std::shared_ptr<RobotContainer> rob = vec_robots_[rc];
                //     for (std::size_t d = 0; d < amVM.size(); d++)
                //     {
                //         std::string vmRname = amVM[d].as<std::string>();
                //         if (rob->getRobotName().compare(vmRname) == 0)
                //         {
                //             unsigned int taskd = rob->getTaskAndJointDof().first;
                //             if (type.compare("Chain") == 0)
                //             {
                //                 co->initializeDimensions(totalJointDofsRobot, totalJointDofsRobot);
                //                 taskd = rob->getTaskAndJointDof().second;
                //             }
                //             else
                //             {
                //                 co->initializeDimensions(totalTaskDofsRobot, totalJointDofsRobot);
                //             }
                //             unsigned int jointd = rob->getTaskAndJointDof().second;
                //             co->addRealRobot(rob,
                //                              countRobotTaskDof,
                //                              countRobotJointDof,
                //                              taskd,
                //                              jointd,
                //                              countRobotJointDof,
                //                              countRobotJointDof,
                //                              jointd,
                //                              jointd,
                //                              countRobotJointDof,
                //                              jointd);
                //             countRobotTaskDof += taskd;
                //             countRobotJointDof += jointd;
                //             break;
                //         }
                //     }
                // }
            }
            vec_cos_.push_back(co);
            _map_cos[co_name] = co;
        }
    }

    ///////////////////////////////////
    // CREATE OUTPUT PORTS FOR CTRLs //
    ///////////////////////////////////

    // Create Output Ports (for Control Components)
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

        // Create control component for port etc.
        std::shared_ptr<ControlComponentContainer> ctrl = std::shared_ptr<ControlComponentContainer>(new ControlComponentContainer(port_name_val));

        // Check all ControlObjectives
        YAML::Node port_co_split_seq = port["ControlObjectives"];
        if (!port_co_split_seq.IsDefined() || port_co_split_seq.IsNull() || !port_co_split_seq.IsSequence())
        {
            RTT::log(RTT::Error) << "ControlObjectives are not defined in port " << i << "!" << RTT::endlog();
            return false;
        }

        unsigned int _totalTaskDof = 0;
        unsigned int _totalJointDof = 0;
        for (std::size_t j = 0; j < port_co_split_seq.size(); j++)
        {
            std::string co_name = port_co_split_seq[j].as<std::string>();
            // get co ptr
            std::shared_ptr<ControlObjectiveContainer> co_tmp = _map_cos[co_name];
            if (!co_tmp)
            {
                RTT::log(RTT::Error) << "ControlObjective ptr not found for " << co_name << "!" << RTT::endlog();
                return false;
            }
            unsigned int _debugHelper_TaskDof = co_tmp->getOperationalFrame().taskdof;
            unsigned int _debugHelper_JointDof = co_tmp->getOperationalFrame().jointdof;
            co_tmp->assignControlComponent(ctrl);
            co_tmp->setCtrlOutputPositions(_totalTaskDof,
                                           _totalJointDof,
                                           _debugHelper_TaskDof,
                                           _debugHelper_JointDof,
                                           _totalJointDof,
                                           _totalJointDof,
                                           _debugHelper_JointDof,
                                           _debugHelper_JointDof,
                                           _totalJointDof,
                                           _debugHelper_JointDof);
            _totalTaskDof += _debugHelper_TaskDof;
            _totalJointDof += _debugHelper_JointDof;
        }

        ctrl->initializeDimensions(this,
                                   _totalTaskDof,
                                   _totalJointDof,
                                   _totalJointDof,
                                   _totalJointDof,
                                   _totalJointDof);
        // Save to global storage
        vec_ccs_.push_back(ctrl);
    }

    ///////////////////////////
    // PARSE ALL THE FILTERS //
    ///////////////////////////
    std::map<std::string, Filter_Struct> _map_filters;

    YAML::Node filtersConfig = config["Filters"];
    if (filtersConfig.IsDefined() && !filtersConfig.IsNull())
    {
        // RTT::log(RTT::Error) << "Filters is not defined properly!" << RTT::endlog();
        // return false;
        // }
        for (std::size_t i = 0; i < filtersConfig.size(); i++)
        {
            std::string filterName = filtersConfig[i]["Filter"].as<std::string>();
            std::string filterType = filtersConfig[i]["Type"].as<std::string>();
            YAML::Node filterMatrixNode = filtersConfig[i]["Data"];

            Eigen::MatrixXd _filter_matrix = Eigen::MatrixXd::Zero(filterMatrixNode.size(), filterMatrixNode[0].size());
            for (std::size_t p = 0; p < filterMatrixNode.size(); p++)
            {
                YAML::Node innerCells = filterMatrixNode[p];
                for (std::size_t q = 0; q < innerCells.size(); q++)
                {
                    _filter_matrix(p, q) = innerCells[q].as<double>();
                }
            }

            Filter_Struct fs;
            fs.name = filterName;
            fs.type = filterType;
            fs.data = _filter_matrix;
            _map_filters[filterName] = fs;
        }
    }

    ///////////////////////////////////////////////////////////////////
    // PARSE ALL THE CONTACT SITUATIONS AND LINK THEM TO THE FILTERS //
    ///////////////////////////////////////////////////////////////////

    for (std::size_t i = 0; i < rootControlObjectivesConfig.size(); i++)
    {
        YAML::Node entry = rootControlObjectivesConfig[i];
        YAML::Node robot = entry["Robot"];

        std::string robotName = robot.as<std::string>();
        RTT::log(RTT::Error) << "> " << robotName << RTT::endlog();

        YAML::Node cs = entry["ContactSituations"];
        RTT::log(RTT::Error) << " Amount of CS " << cs.size() << RTT::endlog();

        // unsigned int jointDOF = entry["JointDof"].as<unsigned int>();
        // RTT::log(RTT::Error) << " Joint Dof controlled by robot " << jointDOF << RTT::endlog();

        // First all Cs!
        for (std::size_t h = 0; h < cs.size(); h++)
        {
            std::string cs_name = cs[h]["Name"].as<std::string>();

            YAML::Node filter = cs[h]["Filters"];
            for (std::size_t v = 0; v < filter.size(); v++)
            {
                YAML::Node innerFilter = filter[v];
                for (YAML::const_iterator it = innerFilter.begin(); it != innerFilter.end(); it++)
                {
                    std::string filterControlObjectiveName = it->first.as<std::string>();
                    std::string filterName = it->second.as<std::string>();

                    std::shared_ptr<ControlObjectiveContainer> co_tmp = _map_cos[filterControlObjectiveName];

                    // Check for filter and if it is not contained in _map_filters treat it as Filter_Identity!
                    Filter_Struct fs_tmp;
                    if (_map_filters.count(filterName) > 0)
                    {
                        fs_tmp = _map_filters[filterName];
                    }
                    else
                    {
                        fs_tmp.name = "Filter_Identity";
                        fs_tmp.type = "None";
                        // RTT::log(RTT::Error) << ">>>>>> CREATE IDENTITY for " << co_tmp->getName() << ": " << co_tmp->getOperationalFrame().taskdof << ", " << co_tmp->getOperationalFrame().taskdof << RTT::endlog();
                        fs_tmp.data = Eigen::MatrixXd::Identity(co_tmp->getOperationalFrame().taskdof, co_tmp->getOperationalFrame().taskdof);
                    }

                    if (filterName.find(":M:") != std::string::npos)
                    {
                        break;
                    }
                    else if (filterName.find(":C:") != std::string::npos)
                    {
                        // co_tmp->addContactSituation(cs_name, fs_tmp, true);
                        RTT::log(RTT::Error) << "[Everything but Ms] Found filter for CS = " << cs_name << ", CO = " << filterControlObjectiveName << ", F = " << filterName << RTT::endlog();

                        RTT::log(RTT::Error) << this->getName() << ": CS = " << cs_name << ", fs_tmp.name = " << fs_tmp.name << ", type = 1 (:C:)" << RTT::endlog();
                        co_tmp->addContactSituation(cs_name, fs_tmp, 1, NULL, COKind::SubspaceFirstOrder, COKind::None);
                    }
                    else if (filterName.compare("Filter_Identity") == 0)
                    {
                        RTT::log(RTT::Error) << "[Everything but Ms] Found filter for CS = " << cs_name << ", CO = " << filterControlObjectiveName << ", F = " << filterName << RTT::endlog();
                        RTT::log(RTT::Error) << this->getName() << ": CS = " << cs_name << ", fs_tmp.name = " << fs_tmp.name << ", type = 0 (:??:)" << RTT::endlog();
                        //     Filter_Struct fs_tmp;
                        //     fs_tmp.name = "Filter_Identity";
                        //     fs_tmp.type = "None";
                        //     fs_tmp.data = Eigen::MatrixXd::Identity(co_tmp->getOperationalFrame().taskdof, co_tmp->getOperationalFrame().taskdof);
                        //     // co_tmp->addContactSituation(cs_name, fs_tmp, false);
                        //     co_tmp->addContactSituation(cs_name, fs_tmp, 0);
                        co_tmp->addContactSituation(cs_name, fs_tmp, 0, NULL, COKind::None, COKind::None);
                    }
                    else
                    {
                        RTT::log(RTT::Error) << "[Everything but Ms] Found filter for CS = " << cs_name << ", CO = " << filterControlObjectiveName << ", F = " << filterName << RTT::endlog();

                        // co_tmp->addContactSituation(cs_name, fs_tmp, false);
                        RTT::log(RTT::Error) << this->getName() << ": CS = " << cs_name << ", fs_tmp.name = " << fs_tmp.name << ", type = 0 (:??:)" << RTT::endlog();
                        // COKind::C might not be correct here though...
                        co_tmp->addContactSituation(cs_name, fs_tmp, 0, NULL, COKind::None, COKind::None); // TODO Perhaps: PotentialSubspace or just treat as SubspaceSecondOrder without Gemini!
                    }

                    break; // TODO make sure that it only contains one!
                }
            }
        }

        // Now all Ms!
        for (std::size_t h = 0; h < cs.size(); h++)
        {
            std::string cs_name = cs[h]["Name"].as<std::string>();

            YAML::Node filter = cs[h]["Filters"];
            for (std::size_t v = 0; v < filter.size(); v++)
            {
                YAML::Node innerFilter = filter[v];
                for (YAML::const_iterator it = innerFilter.begin(); it != innerFilter.end(); it++)
                {
                    std::string filterControlObjectiveName = it->first.as<std::string>();
                    std::string filterName = it->second.as<std::string>();

                    std::shared_ptr<ControlObjectiveContainer> co_tmp = _map_cos[filterControlObjectiveName];

                    // Check for filter and if it is not contained in _map_filters treat it as Filter_Identity!
                    Filter_Struct fs_tmp;
                    if (_map_filters.count(filterName) > 0)
                    {
                        fs_tmp = _map_filters[filterName];
                    }
                    else
                    {
                        fs_tmp.name = "Filter_Identity";
                        fs_tmp.type = "None";
                        fs_tmp.data = Eigen::MatrixXd::Identity(co_tmp->getOperationalFrame().taskdof, co_tmp->getOperationalFrame().taskdof);
                    }

                    if (filterName.find(":M:") != std::string::npos)
                    {
                        RTT::log(RTT::Error) << "[Ms] Found filter for CS = " << cs_name << ", CO = " << filterControlObjectiveName << ", F = " << filterName << RTT::endlog();

                        // Find first CO that uses the :C: version of the filter in the same CS!
                        std::shared_ptr<ControlObjectiveContainer> candidate;
                        for (std::pair<std::string, std::shared_ptr<ControlObjectiveContainer>> element : _map_cos)
                        {
                            if (element.second->containsFilterGemini(cs_name, filterName, ":M:"))
                            {
                                candidate = element.second;
                                // Use first one
                                break;
                            }
                        }
                        if (!candidate)
                        {
                            RTT::log(RTT::Error) << "Could not find CO that uses the gemini of " << filterName << RTT::endlog();
                            // return false;
                            co_tmp->addContactSituation(cs_name, fs_tmp, 0, candidate, COKind::SubspaceSecondOrder, COKind::None);
                            RTT::log(RTT::Error) << this->getName() << ": Added CS = " << cs_name << ", fs_tmp.name = " << fs_tmp.name << ", COKind::SubspaceSecondOrder, COKind::None" << RTT::endlog();
                        }
                        else
                        {
                            RTT::log(RTT::Error) << "Found Gemini of " << filterName << ": " << candidate->getName() << RTT::endlog();
                            co_tmp->addContactSituation(cs_name, fs_tmp, 0, candidate, COKind::SubspaceSecondOrder, COKind::SubspaceFirstOrder);
                            RTT::log(RTT::Error) << this->getName() << ": Added CS = " << cs_name << ", fs_tmp.name = " << fs_tmp.name << ", COKind::SubspaceSecondOrder, COKind::SubspaceFirstOrder" << RTT::endlog();
                        }
                        // co_tmp->addContactSituationWithProjectionGemini(cs_name, fs_tmp, candidate);
                    }

                    break; // TODO make sure that it only contains one!
                }
            }
        }
    }

    return true;
}

bool TaskDescriberSynthesis::configureHook()
{
    // intializations and object creations go here. Each component should run this before being able to run
    return true;
}

bool TaskDescriberSynthesis::startHook()
{
    return true;
}

void TaskDescriberSynthesis::writeMappingConfigurationToFile(const std::string file)
{
    std::string _nodes_real_robots = "digraph g {\nsplines=\"line\"\n// compound\n\ngraph [\nrankdir = \"LR\"\nrank=\"same\"\nnodesep=\"1 equally\"\nranksep=\"3\"];\nnode [\nfontsize = \"16\"\nshape = \"ellipse\"\n];\nedge [\n];\n";

    // Collect all input ports
    _nodes_real_robots += "// inputports\nsubgraph cluster_inputports {\nstyle=filled;\ncolor=lightgrey;\nlabel = \"Input ports\";\n\n// inputports_J\nsubgraph cluster_inputports_J {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"J\";\n\"inputports_J\" [\nlabel = \"{0|0|6|<f0> 7}| {6|7|6|<f1> 7}\"\nshape = \"record\"\n];\n}\n\n// inputports_J_dot\nsubgraph cluster_inputports_J_dot {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"J_dot\";\n\"inputports_J_dot\" [\nlabel = \"{0|0|6|<f0> 7}| {6|7|6|<f1> 7}\"\nshape = \"record\"\n];\n}\n\n// inputports_M\nsubgraph cluster_inputports_M {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"M\";\n\"inputports_M\" [\nlabel = \"{0|0|7|<f0> 7}| {7|7|7|<f1> 7}\"\nshape = \"record\"\n];\n}\n\n// inputports_GC\nsubgraph cluster_inputports_GC {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"GC\";\n\"inputports_GC\" [\nlabel = \"{0|<f0> 7}| {7|<f1> 7}\"\nshape = \"record\"\n];\n}\n\n}\n";

    // collect all real robots
    for (unsigned int r_id = 0; r_id < vec_robots_.size(); r_id++)
    {
        _nodes_real_robots += "subgraph cluster_" + vec_robots_[r_id]->getRobotName() + " {";
        _nodes_real_robots += "style=filled;\ncolor=lightgrey;\nlabel = \"" + vec_robots_[r_id]->getRobotName() + "\"";
        // J
        unsigned int j_i = 0;
        unsigned int j_j = 0;
        unsigned int j_p = 0;
        unsigned int j_q = 0;
        vec_robots_[r_id]->getDebug_ReadJacobianFromGlobalPort(j_i, j_j, j_p, j_q);
        _nodes_real_robots += "subgraph cluster_" + vec_robots_[r_id]->getRobotName() + "_J {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"J\";\n\"" + vec_robots_[r_id]->getRobotName() + "_J\" [\nlabel = \"{<f0> 0|0|" + std::to_string(j_p) + "|<f1> " + std::to_string(j_q) + "}\"\nshape = \"record\"\n];\n}\n";
        // J_dot
        _nodes_real_robots += "subgraph cluster_" + vec_robots_[r_id]->getRobotName() + "_J_dot {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"J_dot\";\n\"" + vec_robots_[r_id]->getRobotName() + "_J_dot\" [\nlabel = \"{<f0> 0|0|" + std::to_string(j_p) + "|<f1> " + std::to_string(j_q) + "}\"\nshape = \"record\"\n];\n}\n";
        // M
        unsigned int m_i = 0;
        unsigned int m_j = 0;
        unsigned int m_p = 0;
        unsigned int m_q = 0;
        vec_robots_[r_id]->getDebug_ReadInertiaFromGlobalPort(m_i, m_j, m_p, m_q);
        _nodes_real_robots += "subgraph cluster_" + vec_robots_[r_id]->getRobotName() + "_M {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"M\";\n\"" + vec_robots_[r_id]->getRobotName() + "_M\" [\nlabel = \"{<f0> 0|0|" + std::to_string(m_p) + "|<f1> " + std::to_string(m_q) + "}\"\nshape = \"record\"\n];\n}\n";
        // GC
        unsigned int gc_i = 0;
        unsigned int gc_p = 0;
        vec_robots_[r_id]->getDebug_ReadGCFromGlobalPort(gc_i, gc_p);
        _nodes_real_robots += "subgraph cluster_" + vec_robots_[r_id]->getRobotName() + "_GC {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"GC\";\n\"" + vec_robots_[r_id]->getRobotName() + "_GC\" [\nlabel = \"{<f0> 0|<f1>" + std::to_string(gc_p) + "}\"\nshape = \"record\"\n];\n}\n";

        _nodes_real_robots += "}\n";
    }

    // Connect all input ports to the real robots
    for (unsigned int r_id = 0; r_id < vec_robots_.size(); r_id++)
    {
        _nodes_real_robots += "\"inputports_J\":f" + std::to_string(r_id) + " -> \"" + vec_robots_[r_id]->getRobotName() + "_J\":f0;\n";
        _nodes_real_robots += "\"inputports_J_dot\":f" + std::to_string(r_id) + " -> \"" + vec_robots_[r_id]->getRobotName() + "_J_dot\":f0;\n";
        _nodes_real_robots += "\"inputports_M\":f" + std::to_string(r_id) + " -> \"" + vec_robots_[r_id]->getRobotName() + "_M\":f0;\n";
        _nodes_real_robots += "\"inputports_GC\":f" + std::to_string(r_id) + " -> \"" + vec_robots_[r_id]->getRobotName() + "_GC\":f0;\n";
    }

    // collect all VMs
    for (unsigned int r_id = 0; r_id < vec_vms_.size(); r_id++)
    {
        _nodes_real_robots += "subgraph cluster_" + vec_vms_[r_id]->getRobotName() + " {";
        _nodes_real_robots += "style=filled;\ncolor=lightgrey;\nlabel = \"" + vec_vms_[r_id]->getRobotName() + "\"";
        // J
        unsigned int j_i = 0;
        unsigned int j_j = 0;
        unsigned int j_p = 0;
        unsigned int j_q = 0;
        _nodes_real_robots += "subgraph cluster_" + vec_vms_[r_id]->getRobotName() + "_J {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"J\";\n\"" + vec_vms_[r_id]->getRobotName() + "_J\" [\nlabel = \"";
        for (unsigned int ir = 0; ir < vec_vms_[r_id]->involvedRealRobots_.size(); ir++)
        {
            vec_vms_[r_id]->getDebug_ReadJacobianFromGlobalPort(ir, j_i, j_j, j_p, j_q);
            _nodes_real_robots += "{<f" + std::to_string(ir) + "> " + std::to_string(j_i) + "|" + std::to_string(j_j) + "|" + std::to_string(j_p) + "|" + std::to_string(j_q) + "}";
            if (ir < vec_vms_[r_id]->involvedRealRobots_.size() - 1)
            {
                _nodes_real_robots += "|";
            }
        }
        _nodes_real_robots += "\"\nshape = \"record\"\n];\n}\n";
        // J_dot
        _nodes_real_robots += "subgraph cluster_" + vec_vms_[r_id]->getRobotName() + "_J_dot {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"J_dot\";\n\"" + vec_vms_[r_id]->getRobotName() + "_J_dot\" [\nlabel = \"";
        for (unsigned int ir = 0; ir < vec_vms_[r_id]->involvedRealRobots_.size(); ir++)
        {
            vec_vms_[r_id]->getDebug_ReadJacobianFromGlobalPort(ir, j_i, j_j, j_p, j_q);
            _nodes_real_robots += "{<f" + std::to_string(ir) + "> " + std::to_string(j_i) + "|" + std::to_string(j_j) + "|" + std::to_string(j_p) + "|" + std::to_string(j_q) + "}";
            if (ir < vec_vms_[r_id]->involvedRealRobots_.size() - 1)
            {
                _nodes_real_robots += "|";
            }
        }
        _nodes_real_robots += "\"\nshape = \"record\"\n];\n}\n";
        // M
        _nodes_real_robots += "subgraph cluster_" + vec_vms_[r_id]->getRobotName() + "_M {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"M\";\n\"" + vec_vms_[r_id]->getRobotName() + "_M\" [\nlabel = \"";
        for (unsigned int ir = 0; ir < vec_vms_[r_id]->involvedRealRobots_.size(); ir++)
        {
            vec_vms_[r_id]->getDebug_ReadInertiaFromGlobalPort(ir, j_i, j_j, j_p, j_q);
            _nodes_real_robots += "{<f" + std::to_string(ir) + "> " + std::to_string(j_i) + "|" + std::to_string(j_j) + "|" + std::to_string(j_p) + "|" + std::to_string(j_q) + "}";
            if (ir < vec_vms_[r_id]->involvedRealRobots_.size() - 1)
            {
                _nodes_real_robots += "|";
            }
        }
        _nodes_real_robots += "\"\nshape = \"record\"\n];\n}\n";
        // GC
        _nodes_real_robots += "subgraph cluster_" + vec_vms_[r_id]->getRobotName() + "_GC {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"GC\";\n\"" + vec_vms_[r_id]->getRobotName() + "_GC\" [\nlabel = \"";
        for (unsigned int ir = 0; ir < vec_vms_[r_id]->involvedRealRobots_.size(); ir++)
        {
            vec_vms_[r_id]->getDebug_ReadGCFromGlobalPort(ir, j_i, j_p);
            _nodes_real_robots += "{<f" + std::to_string(ir) + "> " + std::to_string(j_i) + "|" + std::to_string(j_p) + "}";
            if (ir < vec_vms_[r_id]->involvedRealRobots_.size() - 1)
            {
                _nodes_real_robots += "|";
            }
        }
        _nodes_real_robots += "\"\nshape = \"record\"\n];\n}\n";

        _nodes_real_robots += "}\n";
    }

    // Connect all real robots and vms
    for (unsigned int r_id = 0; r_id < vec_vms_.size(); r_id++)
    {
        for (unsigned int ir = 0; ir < vec_vms_[r_id]->involvedRealRobots_.size(); ir++)
        {
            _nodes_real_robots += "\"" + vec_vms_[r_id]->involvedRealRobots_[ir]->getRobotName() + "_J\":f1 -> \"" + vec_vms_[r_id]->getRobotName() + "_J\":f" + std::to_string(ir) + ";\n";
            _nodes_real_robots += "\"" + vec_vms_[r_id]->involvedRealRobots_[ir]->getRobotName() + "_J_dot\":f1 -> \"" + vec_vms_[r_id]->getRobotName() + "_J_dot\":f" + std::to_string(ir) + ";\n";
            _nodes_real_robots += "\"" + vec_vms_[r_id]->involvedRealRobots_[ir]->getRobotName() + "_M\":f1 -> \"" + vec_vms_[r_id]->getRobotName() + "_M\":f" + std::to_string(ir) + ";\n";
            _nodes_real_robots += "\"" + vec_vms_[r_id]->involvedRealRobots_[ir]->getRobotName() + "_GC\":f1 -> \"" + vec_vms_[r_id]->getRobotName() + "_GC\":f" + std::to_string(ir) + ";\n";
        }
    }

    std::map<std::string, std::vector<JMGC_Control_Position_Struct>> _map_ctrl_to_port_config;
    std::map<std::string, std::vector<std::string>> _map_ctrl_to_co_config;
    // Parse all COs
    _nodes_real_robots += "subgraph cluster_COS {\n";
    _nodes_real_robots += "style=filled;\ncolor=white;\nlabel = \"\"";
    for (unsigned int co_id = 0; co_id < vec_cos_.size(); co_id++)
    {
        _nodes_real_robots += "// " + vec_cos_[co_id]->getName() + "\nsubgraph cluster_" + vec_cos_[co_id]->getName() + " {\nstyle=filled;\ncolor=lightgrey;\nlabel = \"" + vec_cos_[co_id]->getName() + "\";\n";
        // Add inputs
        // J
        _nodes_real_robots += "subgraph cluster_" + vec_cos_[co_id]->getName() + "_J {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"J\";\n\"" + vec_cos_[co_id]->getName() + "_J\" [\nlabel = \"{<f0> 0|0|" + std::to_string(vec_cos_[co_id]->getOperationalFrame().taskdof) + "|<f1> " + std::to_string(vec_cos_[co_id]->getOperationalFrame().jointdof) + "}\"\nshape = \"record\"\n];\n}\n";
        // J_dot
        _nodes_real_robots += "subgraph cluster_" + vec_cos_[co_id]->getName() + "_J_dot {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"J_dot\";\n\"" + vec_cos_[co_id]->getName() + "_J_dot\" [\nlabel = \"{<f0> 0|0|" + std::to_string(vec_cos_[co_id]->getOperationalFrame().taskdof) + "|<f1> " + std::to_string(vec_cos_[co_id]->getOperationalFrame().jointdof) + "}\"\nshape = \"record\"\n];\n}\n";
        // M
        _nodes_real_robots += "subgraph cluster_" + vec_cos_[co_id]->getName() + "_M {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"M\";\n\"" + vec_cos_[co_id]->getName() + "_M\" [\nlabel = \"{<f0> 0|0|" + std::to_string(vec_cos_[co_id]->getOperationalFrame().jointdof) + "|<f1> " + std::to_string(vec_cos_[co_id]->getOperationalFrame().jointdof) + "}\"\nshape = \"record\"\n];\n}\n";
        // GC
        _nodes_real_robots += "subgraph cluster_" + vec_cos_[co_id]->getName() + "_GC {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"GC\";\n\"" + vec_cos_[co_id]->getName() + "_GC\" [\nlabel = \"{<f0> 0|<f1>" + std::to_string(vec_cos_[co_id]->getOperationalFrame().jointdof) + "}\"\nshape = \"record\"\n];\n}\n";

        // Filter
        _nodes_real_robots += "subgraph cluster_" + vec_cos_[co_id]->getName() + "_Filter {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"Filter\";\n\"" + vec_cos_[co_id]->getName() + "_Filter\" [\nlabel = \"<f0>\"\nshape = \"record\"\n];\n}\n";
        // Connect internally to filter
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_J\":f1 -> \"" + vec_cos_[co_id]->getName() + "_Filter\":f0;\n";
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_J_dot\":f1 -> \"" + vec_cos_[co_id]->getName() + "_Filter\":f0;\n";
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_M\":f1 -> \"" + vec_cos_[co_id]->getName() + "_Filter\":f0;\n";
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_GC\":f1 -> \"" + vec_cos_[co_id]->getName() + "_Filter\":f0;\n";

        // Add outputs
        // Jf
        _nodes_real_robots += "subgraph cluster_" + vec_cos_[co_id]->getName() + "_Jf {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"Jf\";\n\"" + vec_cos_[co_id]->getName() + "_Jf\" [\nlabel = \"{<f0> 0|0|" + std::to_string(vec_cos_[co_id]->getOperationalFrame().taskdof) + "|<f1> " + std::to_string(vec_cos_[co_id]->getOperationalFrame().jointdof) + "}\"\nshape = \"record\"\n];\n}\n";
        // Jf_dot
        _nodes_real_robots += "subgraph cluster_" + vec_cos_[co_id]->getName() + "_Jf_dot {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"Jf_dot\";\n\"" + vec_cos_[co_id]->getName() + "_Jf_dot\" [\nlabel = \"{<f0> 0|0|" + std::to_string(vec_cos_[co_id]->getOperationalFrame().taskdof) + "|<f1> " + std::to_string(vec_cos_[co_id]->getOperationalFrame().jointdof) + "}\"\nshape = \"record\"\n];\n}\n";
        // Mc
        _nodes_real_robots += "subgraph cluster_" + vec_cos_[co_id]->getName() + "_Mc {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"Mc\";\n\"" + vec_cos_[co_id]->getName() + "_Mc\" [\nlabel = \"{<f0> 0|0|" + std::to_string(vec_cos_[co_id]->getOperationalFrame().jointdof) + "|<f1> " + std::to_string(vec_cos_[co_id]->getOperationalFrame().jointdof) + "}\"\nshape = \"record\"\n];\n}\n";
        // P
        _nodes_real_robots += "subgraph cluster_" + vec_cos_[co_id]->getName() + "_P {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"P\";\n\"" + vec_cos_[co_id]->getName() + "_P\" [\nlabel = \"{<f0> 0|0|" + std::to_string(vec_cos_[co_id]->getOperationalFrame().jointdof) + "|<f1> " + std::to_string(vec_cos_[co_id]->getOperationalFrame().jointdof) + "}\"\nshape = \"record\"\n];\n}\n";
        // P_dot
        _nodes_real_robots += "subgraph cluster_" + vec_cos_[co_id]->getName() + "_P_dot {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"P_dot\";\n\"" + vec_cos_[co_id]->getName() + "_P_dot\" [\nlabel = \"{<f0> 0|0|" + std::to_string(vec_cos_[co_id]->getOperationalFrame().jointdof) + "|<f1> " + std::to_string(vec_cos_[co_id]->getOperationalFrame().jointdof) + "}\"\nshape = \"record\"\n];\n}\n";
        // Connect internally to output
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_Filter\" : f0->\"" + vec_cos_[co_id]->getName() + "_Mc\" : f0;\n";
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_Filter\" : f0->\"" + vec_cos_[co_id]->getName() + "_P\" : f0;\n";
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_Filter\" : f0->\"" + vec_cos_[co_id]->getName() + "_P_dot\" : f0;\n";
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_Filter\" : f0->\"" + vec_cos_[co_id]->getName() + "_Jf\" : f0;\n";
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_Filter\" : f0->\"" + vec_cos_[co_id]->getName() + "_Jf_dot\" : f0;\n";

        _nodes_real_robots += "}\n";

        // Create global connections from associated robot
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getDebug_AssociatedRobot()->getRobotName() + "_J\" : f1->\"" + vec_cos_[co_id]->getName() + "_J\" : f0;\n";
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getDebug_AssociatedRobot()->getRobotName() + "_J_dot\" : f1->\"" + vec_cos_[co_id]->getName() + "_J_dot\" : f0;\n";
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getDebug_AssociatedRobot()->getRobotName() + "_M\" : f1->\"" + vec_cos_[co_id]->getName() + "_M\" : f0;\n";
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getDebug_AssociatedRobot()->getRobotName() + "_GC\" : f1->\"" + vec_cos_[co_id]->getName() + "_GC\" : f0;\n";

        // Store assigned ctrl parts
        _map_ctrl_to_port_config[vec_cos_[co_id]->getAssignedControlComponent()->getName()].push_back(vec_cos_[co_id]->getDebug_CtrlOutputPositions());
        _map_ctrl_to_co_config[vec_cos_[co_id]->getAssignedControlComponent()->getName()].push_back(vec_cos_[co_id]->getName());
    }
    _nodes_real_robots += "}\n";

    _nodes_real_robots += "subgraph cluster_CCS {\n";
    _nodes_real_robots += "style=filled;\ncolor=white;\nlabel = \"\"";

    std::map<std::string, std::pair<std::string, unsigned int>> _map_co_to_ctrl_and_port_id;
    for (unsigned int c_id = 0; c_id < vec_ccs_.size(); c_id++)
    {
        std::string _nodes_ctrl_Jf = "";
        std::string _nodes_ctrl_Jf_dot = "";
        std::string _nodes_ctrl_Mc = "";
        std::string _nodes_ctrl_P = "";
        std::string _nodes_ctrl_P_dot = "";
        std::string _nodes_ctrl_GC = "";

        std::vector<unsigned int> _already_checked;
        // for each field
        for (unsigned int i = 0; i < _map_ctrl_to_port_config[vec_ccs_[c_id]->getName()].size(); i++)
        {
            JMGC_Control_Position_Struct smallest;
            smallest.j_i_ = 100000;
            smallest.j_j_ = 100000;
            smallest.j_p_ = 100000;
            smallest.j_q_ = 100000;
            smallest.m_i_ = 100000;
            smallest.m_j_ = 100000;
            smallest.m_p_ = 100000;
            smallest.m_q_ = 100000;
            smallest.gc_i_ = 100000;
            smallest.gc_p_ = 100000;
            std::string co_name = "";

            // find the smallest
            for (unsigned int j = 0; j < _map_ctrl_to_port_config[vec_ccs_[c_id]->getName()].size(); j++)
            {
                bool already_checked = false;
                for (unsigned int ac = 0; ac < _already_checked.size(); ac++)
                {
                    if (j == _already_checked[ac])
                    {
                        already_checked = true;
                        break;
                    }
                }
                if (already_checked)
                {
                    continue;
                }

                if (_map_ctrl_to_port_config[vec_ccs_[c_id]->getName()][j].j_i_ < smallest.j_i_)
                {
                    smallest = _map_ctrl_to_port_config[vec_ccs_[c_id]->getName()][j];
                    co_name = _map_ctrl_to_co_config[vec_ccs_[c_id]->getName()][j];
                    _already_checked.push_back(j);
                }
                else if (_map_ctrl_to_port_config[vec_ccs_[c_id]->getName()][j].j_i_ == smallest.j_i_ && _map_ctrl_to_port_config[vec_ccs_[c_id]->getName()][j].j_j_ < smallest.j_j_)
                {
                    smallest = _map_ctrl_to_port_config[vec_ccs_[c_id]->getName()][j];
                    co_name = _map_ctrl_to_co_config[vec_ccs_[c_id]->getName()][j];
                    _already_checked.push_back(j);
                }
            }

            _map_co_to_ctrl_and_port_id[co_name] = std::make_pair(vec_ccs_[c_id]->getName(), i);

            if (i > 0)
            {
                _nodes_ctrl_Jf += " | ";
                _nodes_ctrl_Jf_dot += " | ";
                _nodes_ctrl_Mc += "|";
                _nodes_ctrl_P += "|";
                _nodes_ctrl_P_dot += "|";
                _nodes_ctrl_GC += "|";
            }
            // Jf
            _nodes_ctrl_Jf += "{<f" + std::to_string(i) + "> " + std::to_string(smallest.j_i_) + "|" + std::to_string(smallest.j_j_) + "|" + std::to_string(smallest.j_p_) + "|" + std::to_string(smallest.j_q_) + "}";
            // Jf_dot
            _nodes_ctrl_Jf_dot += "{<f" + std::to_string(i) + "> " + std::to_string(smallest.j_i_) + "|" + std::to_string(smallest.j_j_) + "|" + std::to_string(smallest.j_p_) + "|" + std::to_string(smallest.j_q_) + "}";
            // Mc
            _nodes_ctrl_Mc += "{<f" + std::to_string(i) + "> " + std::to_string(smallest.m_i_) + "|" + std::to_string(smallest.m_j_) + "|" + std::to_string(smallest.m_p_) + "|" + std::to_string(smallest.m_q_) + "}";
            // P
            _nodes_ctrl_P += "{<f" + std::to_string(i) + "> " + std::to_string(smallest.m_i_) + "|" + std::to_string(smallest.m_j_) + "|" + std::to_string(smallest.m_p_) + "|" + std::to_string(smallest.m_q_) + "}";
            // P_dot
            _nodes_ctrl_P_dot += "{<f" + std::to_string(i) + "> " + std::to_string(smallest.m_i_) + "|" + std::to_string(smallest.m_j_) + "|" + std::to_string(smallest.m_p_) + "|" + std::to_string(smallest.m_q_) + "}";
            // GC
            _nodes_ctrl_GC += "{<f" + std::to_string(i) + "> " + std::to_string(smallest.gc_i_) + "|" + std::to_string(smallest.gc_p_) + "}";
        }

        // Fit Ctrl together
        _nodes_real_robots += "subgraph cluster_" + vec_ccs_[c_id]->getName() + " {";
        _nodes_real_robots += "style=filled;\ncolor=lightgrey;\nlabel = \"" + vec_ccs_[c_id]->getName() + "\"";
        // J
        _nodes_real_robots += "subgraph cluster_" + vec_ccs_[c_id]->getName() + "_J {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"J\";\n\"" + vec_ccs_[c_id]->getName() + "_J\" [\nlabel = \"" + _nodes_ctrl_Jf + "\"\nshape = \"record\"\n];\n}\n";
        // J_dot
        _nodes_real_robots += "subgraph cluster_" + vec_ccs_[c_id]->getName() + "_J_dot {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"J_dot\";\n\"" + vec_ccs_[c_id]->getName() + "_J_dot\" [\nlabel = \"" + _nodes_ctrl_Jf_dot + "\"\nshape = \"record\"\n];\n}\n";
        // Mc
        _nodes_real_robots += "subgraph cluster_" + vec_ccs_[c_id]->getName() + "_M {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"M\";\n\"" + vec_ccs_[c_id]->getName() + "_M\" [\nlabel = \"" + _nodes_ctrl_Mc + "\"\nshape = \"record\"\n];\n}\n";
        // P
        _nodes_real_robots += "subgraph cluster_" + vec_ccs_[c_id]->getName() + "_P {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"P\";\n\"" + vec_ccs_[c_id]->getName() + "_P\" [\nlabel = \"" + _nodes_ctrl_P + "\"\nshape = \"record\"\n];\n}\n";
        // P_dot
        _nodes_real_robots += "subgraph cluster_" + vec_ccs_[c_id]->getName() + "_P_dot {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"P_dot\";\n\"" + vec_ccs_[c_id]->getName() + "_P_dot\" [\nlabel = \"" + _nodes_ctrl_P_dot + "\"\nshape = \"record\"\n];\n}\n";
        // GC
        _nodes_real_robots += "subgraph cluster_" + vec_ccs_[c_id]->getName() + "_GC {\nstyle=filled;\ncolor=white;\nnode [style=filled,color=black, fillcolor=white];\nlabel = \"GC\";\n\"" + vec_ccs_[c_id]->getName() + "_GC\" [\nlabel = \"" + _nodes_ctrl_GC + "\"\nshape = \"record\"\n];\n}\n";

        _nodes_real_robots += "}\n";
    }
    _nodes_real_robots += "}\n";

    // Connect all COs -> Ctrls
    for (unsigned int co_id = 0; co_id < vec_cos_.size(); co_id++)
    {
        // Jf -> J
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_Jf\":f1 -> \"" + _map_co_to_ctrl_and_port_id[vec_cos_[co_id]->getName()].first + "_J\":f" + std::to_string(_map_co_to_ctrl_and_port_id[vec_cos_[co_id]->getName()].second) + ";";
        // Jf_dot -> J_dot
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_Jf_dot\":f1 -> \"" + _map_co_to_ctrl_and_port_id[vec_cos_[co_id]->getName()].first + "_J_dot\":f" + std::to_string(_map_co_to_ctrl_and_port_id[vec_cos_[co_id]->getName()].second) + ";";
        // Mc -> M
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_Mc\":f1 -> \"" + _map_co_to_ctrl_and_port_id[vec_cos_[co_id]->getName()].first + "_M\":f" + std::to_string(_map_co_to_ctrl_and_port_id[vec_cos_[co_id]->getName()].second) + ";";
        // P -> P
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_P\":f1 -> \"" + _map_co_to_ctrl_and_port_id[vec_cos_[co_id]->getName()].first + "_P\":f" + std::to_string(_map_co_to_ctrl_and_port_id[vec_cos_[co_id]->getName()].second) + ";";
        // P_dot -> P_dot
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_P_dot\":f1 -> \"" + _map_co_to_ctrl_and_port_id[vec_cos_[co_id]->getName()].first + "_P_dot\":f" + std::to_string(_map_co_to_ctrl_and_port_id[vec_cos_[co_id]->getName()].second) + ";";
        // GC -> GC
        _nodes_real_robots += "\"" + vec_cos_[co_id]->getName() + "_GC\":f1 -> \"" + _map_co_to_ctrl_and_port_id[vec_cos_[co_id]->getName()].first + "_GC\":f" + std::to_string(_map_co_to_ctrl_and_port_id[vec_cos_[co_id]->getName()].second) + ";";
    }

    _nodes_real_robots += "}\n";

    std::ofstream fileout(file);
    fileout << _nodes_real_robots;
}

bool TaskDescriberSynthesis::isNewCSReached()
{
    return this->new_cs_reached_;
}

void TaskDescriberSynthesis::updateHook()
{
    // Change Contact Situation in sync with updateHook
    if (this->new_cs_requested_)
    {
        for (unsigned int co_id = 0; co_id < vec_cos_.size(); co_id++)
        {
            vec_cos_[co_id]->activateContactSituation(this->new_cs_name_requested_);
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

    // RTT::os::TimeService::nsecs start = RTT::os::TimeService::Instance()->getNSecs();
    // 1) Read all ports
    // for (unsigned int p = 0; p < vec_J_Input_Ports.size(); p++) {
    // J
    J_Flow = in_J_Input_port.read(in_J_var);
    // J dot
    J_Dot_Flow = in_J_Dot_Input_port.read(in_J_Dot_var);
    // M
    M_Flow = in_M_Input_port.read(in_M_var);
    // GC
    GC_Flow = in_GC_Input_port.read(in_GC_var);
    // robotstatus
    robotstatus_Flow = in_robotstatus_Input_port.read(in_robotstatus_var);

    // Cart Pos feedback
    CartPos_Flow = in_CartPos_Input_port.read(in_CartPos_var);
    // Cart Vel feedback
    CartVel_Flow = in_CartVel_Input_port.read(in_CartVel_var);
    // }

    if (J_Flow == RTT::NoData ||
        J_Dot_Flow == RTT::NoData ||
        M_Flow == RTT::NoData ||
        GC_Flow == RTT::NoData ||
        robotstatus_Flow == RTT::NoData ||
        CartPos_Flow == RTT::NoData ||
        CartVel_Flow == RTT::NoData)
    {
        return;
    }

    // RTT::log(RTT::Fatal) << "Size in_robotstatus_var.position.size() = " << in_robotstatus_var.position.size() << RTT::endlog();
    // Eigen::VectorXd _tmp_my_pos = Eigen::VectorXd::Zero(in_robotstatus_var.position.size());
    // for (unsigned int iiiiii = 0; iiiiii < in_robotstatus_var.position.size(); iiiiii++)
    // {
    //     _tmp_my_pos[iiiiii] = in_robotstatus_var.position[iiiiii];
    // }

    // Convert into Eigen:
    this->in_robotstatus_pos_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(in_robotstatus_var.position.data(), in_robotstatus_var.position.size());
    this->in_robotstatus_vel_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(in_robotstatus_var.velocity.data(), in_robotstatus_var.velocity.size());
    this->in_robotstatus_trq_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(in_robotstatus_var.effort.data(), in_robotstatus_var.effort.size());

    // for (unsigned int iiiiii = 0; iiiiii < in_robotstatus_var.position.size(); iiiiii++)
    // {
    //     RTT::log(RTT::Fatal) << "_tmp_my_pos[" << iiiiii << "] = " << _tmp_my_pos[iiiiii] << RTT::endlog();
    //     RTT::log(RTT::Fatal) << "this->in_robotstatus_pos_[" << iiiiii << "] = " << this->in_robotstatus_pos_[iiiiii] << RTT::endlog();
    // }

    // RTT::log(RTT::Fatal) << "this->in_robotstatus_pos_ = " << this->in_robotstatus_pos_ << RTT::endlog();

    // Timed switching of CS refs #32
    increaseActivation();

    // RTT::log(RTT::Error) << "in_J_var =\n"
    //                      << in_J_var << "\nin_J_Dot_var =\n"
    //                      << in_J_Dot_var
    //                      << "\nin_M_var =\n"
    //                      << in_M_var
    //                      << "\nin_GC_var =\n"
    //                      << in_GC_var
    //                      << "\nin_robotstatus_var =\n"
    //                      << in_robotstatus_var
    //                      << "\nin_CartPos_var =\n"
    //                      << in_CartPos_var
    //                      << "\nin_CartVel_var =\n"
    //                      << in_CartVel_var
    //                      << RTT::endlog();

    // //DEBUG!!!!!!!
    // in_J_var.setRandom();
    // in_J_Dot_var.setRandom();
    // in_M_var.setRandom();
    // in_GC_var.setRandom();

    // for (unsigned int c = 0; c < vec_ccs_.size(); c++)
    // {
    //     std::shared_ptr<ControlComponentContainer> ctrl = vec_ccs_[c];
    //     ctrl->printDebug();
    // }
    // for (unsigned int co_id = 0; co_id < vec_cos_.size(); co_id++)
    // {
    //     std::shared_ptr<ControlObjectiveContainer> co = vec_cos_[co_id];
    //     co->printDebug();
    // }
    // //DEBUG!!!!!!!

    // RTT::os::TimeService::nsecs start_robot_containers = RTT::os::TimeService::Instance()->getNSecs();
    // Update the real robot containers
    for (unsigned int r_id = 0; r_id < vec_robots_.size(); r_id++)
    {
        // RTT::log(RTT::Error) << "UPDATE: " << r_id << " " << vec_robots_[r_id]->getRobotName() << " > > > in_CartPos_var=\n"
        //                      << in_CartPos_var << RTT::endlog();
        // RTT::log(RTT::Error) << "UPDATE<>: " << r_id << " " << vec_robots_[r_id]->getRobotName() << " > > > in_CartVel_var=\n"
        //                      << in_CartVel_var << RTT::endlog();
        vec_robots_[r_id]->updateJMG(in_J_var, in_J_Dot_var, in_M_var, in_GC_var, this->in_robotstatus_pos_, this->in_robotstatus_vel_, this->in_robotstatus_trq_, in_CartPos_var, in_CartVel_var);
    }
    // RTT::os::TimeService::nsecs end_robot_containers = RTT::os::TimeService::Instance()->getNSecs(start_robot_containers);
    // if (time_storage_robot_containers.size() < time_storage_robot_containers.capacity())
    // {
    //     time_storage_robot_containers.push_back(1e-6 * end_robot_containers);
    // }

    // RTT::os::TimeService::nsecs start_vm = RTT::os::TimeService::Instance()->getNSecs();
    // Update the VM robot containers
    for (unsigned int r_id = 0; r_id < vec_vms_.size(); r_id++)
    {
        // RTT::log(RTT::Error) << "UPDATE: " << r_id << " " << vec_vms_[r_id]->getRobotName() << RTT::endlog();
        std::shared_ptr<VMContainer> vm = vec_vms_[r_id];
        for (unsigned int ir = 0; ir < vm->involvedRealRobots_.size(); ir++)
        {
            std::shared_ptr<ManipulatorDataSupplier> r = vm->involvedRealRobots_[ir];
            // RTT::log(RTT::Error) << "ADDDDDDDDD to " << vm->getRobotName() << " r->cart_pos_=\n"
            //                      << r->cart_pos_ << "\nr->cart_vel_=\n"
            //                      << r->cart_vel_ << RTT::endlog();
            vm->updateJMGForRobot(ir, r->jacobian_, r->jacobian_dot_, r->inertia_, r->gc_, r->robotstatus_pos_, r->robotstatus_vel_, r->robotstatus_trq_, r->cart_pos_, r->cart_vel_);
            vm->compute();
        }
    }
    // RTT::os::TimeService::nsecs end_vm = RTT::os::TimeService::Instance()->getNSecs(start_vm);
    // if (time_storage_vm.size() < time_storage_vm.capacity())
    // {
    //     time_storage_vm.push_back(1e-6 * end_vm);
    // }

    // RTT::os::TimeService::nsecs start_co_time = RTT::os::TimeService::Instance()->getNSecs();
    // 2) For each not gemini-dependent co
    for (unsigned int co_id = 0; co_id < vec_cos_.size(); co_id++) // f(N1, N2, N3) = sum(co_id from 1 to N1)(sum(r_id from 1 to N2)(1 + 1 + sum(ir from 1 to N3)(...)))
    {
        std::shared_ptr<ControlObjectiveContainer> co = vec_cos_[co_id];
        if (co->currentlyNeedsToRetrievePFromGemini())
        {
            continue;
        }

        co->drawJMGFromAssociatedRobot();

        // TODO WHAT KIND OF ACTIVATION IS THIS?
        co->applyFilter(debug_pointintime /* activation */);

        std::shared_ptr<ControlComponentContainer> ctrl = co->getAssignedControlComponent();
        co->outputFilteredJMG(ctrl->jacobian_, ctrl->jacobian_dot_, ctrl->inertia_, ctrl->gc_, ctrl->inertia_c_, ctrl->P_, ctrl->P_dot_, ctrl->robotstate_pos_, ctrl->robotstate_vel_, ctrl->robotstate_trq_, ctrl->int_wrench_, ctrl->vm_fdb_cart_pos_, ctrl->vm_fdb_cart_vel_);
    }
    // RTT::os::TimeService::nsecs end_co_time = RTT::os::TimeService::Instance()->getNSecs(start_co_time);
    // if (time_storage_co_time.size() < time_storage_co_time.capacity())
    // {
    //     time_storage_co_time.push_back(1e-6 * end_co_time);
    // }

    // RTT::os::TimeService::nsecs start_co_gemini = RTT::os::TimeService::Instance()->getNSecs();
    // 3) For each gemini-dependent co
    for (unsigned int co_id = 0; co_id < vec_cos_.size(); co_id++) // f(N1, N2, N3) = sum(co_id from 1 to N1)(sum(r_id from 1 to N2)(1 + 1 + sum(ir from 1 to N3)(...)))
    {
        std::shared_ptr<ControlObjectiveContainer> co = vec_cos_[co_id];
        if (!co->currentlyNeedsToRetrievePFromGemini())
        {
            continue;
        }

        co->drawJMGFromAssociatedRobot();

        // TODO WHAT KIND OF ACTIVATION IS THIS?
        co->applyFilter(debug_pointintime /* activation */);

        std::shared_ptr<ControlComponentContainer> ctrl = co->getAssignedControlComponent();
        co->outputFilteredJMG(ctrl->jacobian_, ctrl->jacobian_dot_, ctrl->inertia_, ctrl->gc_, ctrl->inertia_c_, ctrl->P_, ctrl->P_dot_, ctrl->robotstate_pos_, ctrl->robotstate_vel_, ctrl->robotstate_trq_, ctrl->int_wrench_ /* Only required if this is a VM */, ctrl->vm_fdb_cart_pos_ /* Only required if this is a VM */, ctrl->vm_fdb_cart_vel_ /* Only required if this is a VM */);
    }
    // RTT::os::TimeService::nsecs end_co_gemini = RTT::os::TimeService::Instance()->getNSecs(start_co_gemini);
    // if (time_storage_co_gemini.size() < time_storage_co_gemini.capacity())
    // {
    //     time_storage_co_gemini.push_back(1e-6 * end_co_gemini);
    // }

    // RTT::os::TimeService::nsecs start_send = RTT::os::TimeService::Instance()->getNSecs();
    // 4) Send stuff
    for (unsigned int c = 0; c < vec_ccs_.size(); c++)
    {
        std::shared_ptr<ControlComponentContainer> ctrl = vec_ccs_[c];
        ctrl->sendJMG();
    }
    // RTT::os::TimeService::nsecs end_send = RTT::os::TimeService::Instance()->getNSecs(start_send);
    // if (time_storage_send.size() < time_storage_send.capacity())
    // {
    //     time_storage_send.push_back(1e-6 * end_send);
    // }

    // RTT::os::TimeService::nsecs end = RTT::os::TimeService::Instance()->getNSecs(start);
    // // RTT::log(RTT::Error) << 1e-6 * end << " milliseconds [" << this->getName() << "]" << RTT::endlog();

    // if (time_storage.size() < time_storage.capacity())
    // {
    //     time_storage.push_back(1e-6 * end);
    // }

    out_debug_pointintime_port.write(debug_pointintime);
}

void TaskDescriberSynthesis::activateContactSituation(const std::string &csName)
{
    this->new_cs_reached_ = false;
    this->new_cs_timed_requested_ = 0.0;
    this->new_cs_name_requested_ = csName;
    this->new_cs_requested_ = true;

    // Ausgelagert und sync mit updatehook
    // for (unsigned int co_id = 0; co_id < vec_cos_.size(); co_id++)
    // {
    //     vec_cos_[co_id]->activateContactSituation(csName);
    // }
}

void TaskDescriberSynthesis::activateContactSituationResetActivation(const std::string &cs_name, const double time_secs)
{
    this->new_cs_reached_ = false;
    this->new_cs_timed_requested_ = time_secs;
    this->new_cs_name_requested_ = cs_name;
    this->new_cs_requested_ = true;

    // Ausgelagert und sync mit updatehook
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

void TaskDescriberSynthesis::increaseActivation()
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
    }
}

bool TaskDescriberSynthesis::exists_test(const std::string &name)
{
    if (FILE *file = fopen(name.c_str(), "r"))
    {
        fclose(file);
        return true;
    }
    else
    {
        return false;
    }
}

void TaskDescriberSynthesis::stopHook()
{
    // std::map<double>
    //
    // TODO DLW : OK VIELLEICHT MUSS ICH VM UND NORMALE TASKS TRENNEN? WO IST DIE ECHTE ABHAENGIGKEIT DENN?
    //
    // stops the component (update hook wont be  called anymore)

    // std::ofstream myfile;
    // myfile.open ("time_storage.csv");
    // for (unsigned int i = 0; i < time_storage.size(); i++)
    // {
    //     if (i > 0)
    //     {
    //         myfile << ",";
    //     }
    //     myfile << time_storage[i];
    // }
    // myfile.close();

    // myfile.open ("time_storage_robot_containers.csv");
    // for (unsigned int i = 0; i < time_storage_robot_containers.size(); i++)
    // {
    //     if (i > 0)
    //     {
    //         myfile << ",";
    //     }
    //     myfile << time_storage_robot_containers[i];
    // }
    // myfile.close();

    // myfile.open ("time_storage_vm.csv");
    // for (unsigned int i = 0; i < time_storage_vm.size(); i++)
    // {
    //     if (i > 0)
    //     {
    //         myfile << ",";
    //     }
    //     myfile << time_storage_vm[i];
    // }
    // myfile.close();

    // myfile.open ("time_storage_co_time.csv");
    // for (unsigned int i = 0; i < time_storage_co_time.size(); i++)
    // {
    //     if (i > 0)
    //     {
    //         myfile << ",";
    //     }
    //     myfile << time_storage_co_time[i];
    // }
    // myfile.close();

    // myfile.open ("time_storage_co_gemini.csv");
    // for (unsigned int i = 0; i < time_storage_co_gemini.size(); i++)
    // {
    //     if (i > 0)
    //     {
    //         myfile << ",";
    //     }
    //     myfile << time_storage_co_gemini[i];
    // }
    // myfile.close();

    // myfile.open ("time_storage_send.csv");
    // for (unsigned int i = 0; i < time_storage_send.size(); i++)
    // {
    //     if (i > 0)
    //     {
    //         myfile << ",";
    //     }
    //     myfile << time_storage_send[i];
    // }
    // myfile.close();

    // for (unsigned int r_id = 0; r_id < vec_vms_.size(); r_id++)
    // {
    //     std::shared_ptr<VMContainer> vm = vec_vms_[r_id];
    //     for (unsigned int ir = 0; ir < vm->involvedRealRobots_.size(); ir++)
    //     {
    //         std::shared_ptr<ManipulatorDataSupplier> r = vm->involvedRealRobots_[ir];
    //         vm->writeDebugTiming();
    //     }
    // }

    // RTT::log(RTT::Error) << "Wrote all timing information!" << RTT::endlog();
}

void TaskDescriberSynthesis::cleanupHook()
{
    // cleaning the component data
    portsArePrepared = false;
}

void TaskDescriberSynthesis::preparePorts()
{
    portsArePrepared = true;
}

void TaskDescriberSynthesis::displayCurrentState()
{
}

//this macro should appear only once per library
// ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::task::TaskDescriberSynthesis)
