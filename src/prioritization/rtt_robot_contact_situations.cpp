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
#include "../../include/cosima-controller/prioritization/rtt_robot_contact_situations.hpp"
#include <rtt/Logger.hpp>

using namespace cosima;
using namespace prioritization;

RobotContactSituations::RobotContactSituations() : amVM(false), transitionIndex(0.0)
{
    this->debug_print_state_ = false;
}

RobotContactSituations::RobotContactSituations(const unsigned int numberOfCS, const unsigned int jointDoF, const unsigned int numTasks, bool amVM) : transitionIndex(0.0)
{
    this->debug_print_state_ = false;
    this->jointDoF = jointDoF;
    this->amVM = amVM;
    this->numTasks = numTasks;
    this->portIndex_ = 0; // Perhaps -1?

    this->controlObjectivesList.resize(this->numTasks);
    if (this->amVM)
    {
        // This need to be done for the "CombinedRobotTask".
        this->numTasks += 1;
    }

    this->tasksizes = Eigen::VectorXd::Zero(this->numTasks);
    for (unsigned int i = 0; i < this->numTasks; i++)
    {
        // this->compensation_for_cots.resize(this->numTasks);
        std::vector<int> tmp;
        this->compensation_for_cots.push_back(tmp);
    }

    this->lastActiveAlpha = Eigen::MatrixXd::Zero(this->numTasks, this->numTasks);
    this->selection = Eigen::MatrixXd::Zero(this->numTasks, this->numTasks);

    this->ranks = Eigen::VectorXd::Zero(this->numTasks);

    // this->inertia_ = Eigen::MatrixXd::Zero(this->jointDoF, this->jointDoF);

    // for (unsigned int i = 0; i < numberOfCS; i++) {
    //     this->alpha_per_cs.push_back(Eigen::MatrixXd::Zero(this->numTasks, this->numTasks));
    // }

    for (unsigned int i = 0; i < this->numTasks; i++)
    {
        // this->tasksizes[i] = 6;
        this->torque_commands.push_back(Eigen::VectorXd::Zero(this->jointDoF));
        this->allProjections.push_back(Eigen::MatrixXd::Zero(this->jointDoF, this->jointDoF));
    }

    this->torque_commands_proj_states_stacked = Eigen::VectorXd::Zero(this->torque_commands.size() * this->jointDoF);
    this->allProjections_stacked = Eigen::MatrixXd::Zero(this->torque_commands.size() * this->jointDoF, this->jointDoF);

    identityDOFsizeDOFsize = Eigen::MatrixXd::Identity(this->jointDoF, this->jointDoF);
}

bool RobotContactSituations::amIaVM(unsigned int portIndex)
{
    return this->amVM && (portIndex == this->portIndex_);
}

bool RobotContactSituations::amIaVM()
{
    return this->amVM;
}

std::vector<std::string> RobotContactSituations::getCombinedRobotNames()
{
    return combinedRobotNames;
}

void RobotContactSituations::addRobot2CombinedRobots(const std::string &name)
{
    combinedRobotNames.push_back(name);
}

void RobotContactSituations::addVM2RealMapping(const std::string &name, const unsigned int startingIndexInVector, const unsigned int jointDof)
{
    VMCombinedRobotStorage v;
    v.name = name;
    v.startingIndexInVector = startingIndexInVector;
    v.jointDof = jointDof;
    mapping_from_VMmanipulator_to_REALmanipulator.push_back(v);
}

void RobotContactSituations::setRobotName(const std::string &name)
{
    this->robotName = name;
}

std::string RobotContactSituations::getRobotName()
{
    return this->robotName;
}

// TODO do not forget to set this!
bool RobotContactSituations::setStartCS(const std::string &name)
{
    // if (alpha_per_cs.count(name) > 0)
    if (this->alpha_per_cs.find(name) != this->alpha_per_cs.end())
    {
        this->lastActiveAlpha = alpha_per_cs[name];
    }
    else
    {
        // If the chosen CS is not available, just take the first one!
        RTT::log(RTT::Error) << "robot: " << this->getRobotName() << ", alpha_per_cs.size() = " << alpha_per_cs.size() << ", name=" << name << RTT::endlog();
        RTT::log(RTT::Error) << "alpha_per_cs.begin()->second:\n"
                             << alpha_per_cs.begin()->second << RTT::endlog();
        this->lastActiveAlpha = alpha_per_cs.begin()->second;
    }
    ghcProj.setAlphas(this->lastActiveAlpha);
    RTT::log(RTT::Error) << this->getRobotName() << " called setStartCS( " << name << " ) and set this->lastActiveAlpha to\n"
                         << this->lastActiveAlpha << RTT::endlog();
}

void RobotContactSituations::updateInteralAndExternalProjection(Eigen::MatrixXd &internal, Eigen::MatrixXd &internal_dot)
{
    this->projection_internal_ = internal;
    this->projection_internal_dot_ = internal_dot;
}

bool RobotContactSituations::addCSAlpha(const std::string &csName, const Eigen::MatrixXd &alpha, const std::vector<std::string> &header)
{
    // if (alpha.rows() != this->numTasks) {
    //     RTT::log(RTT::Fatal) << "alpha.rows (" << alpha.rows() << ") != numTasks (" << this->numTasks << ")!" << RTT::endlog();
    //     return false;
    // }
    // if (alpha.cols() != this->numTasks) {
    //     RTT::log(RTT::Fatal) << "alpha.cols (" << alpha.cols() << ") != numTasks (" << this->numTasks << ")!" << RTT::endlog();
    //     return false;
    // }

    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(alpha.rows(), alpha.cols());
    if (this->amVM)
    {
        m = Eigen::MatrixXd::Zero(alpha.rows() + 1, alpha.cols() + 1);
    }
    // m.resize(alpha.rows(), alpha.cols());

    // sort first
    std::map<std::string, unsigned int> map_coNames_2_headerIndex;
    // get controlObjectivesList...
    unsigned int colSize = this->controlObjectivesList.size();
    // if (this->amVM)
    // {
    //     // colSize -= 1;
    // }

    for (unsigned int i = 0; i < colSize; i++)
    {
        // find index of header
        for (unsigned int h = 0; h < header.size(); h++)
        {
            if (controlObjectivesList[i].compare(header[h]) == 0)
            {
                map_coNames_2_headerIndex[controlObjectivesList[i]] = h;
            }
        }
    }
    // reorder the matrix
    for (unsigned int i = 0; i < colSize; i++)
    {
        for (unsigned int j = 0; j < colSize; j++)
        {
            m(i, j) = alpha(map_coNames_2_headerIndex[controlObjectivesList[i]], map_coNames_2_headerIndex[controlObjectivesList[j]]);
        }
    }

    if (this->amVM)
    {
        // deactivate the combined robot task that is always the last one!
        for (unsigned int i = 0; i < colSize; i++)
        {
            m(i, colSize) = 0.0;
            m(colSize, i) = 1.0;
        }
        m(colSize, colSize) = 1.0;
    }
    // // Debug print matrix m
    // RTT::log(RTT::Error) << "alpha:\n" << alpha << RTT::endlog();
    // RTT::log(RTT::Error) << "m:\n" << m << RTT::endlog();

    std::string csNameTmp = csName;

    RTT::log(RTT::Error) << "Add CS for " << csNameTmp << " with alpha =\n"
                         << alpha << ",\nresult m =\n"
                         << m << RTT::endlog();

    alpha_per_cs[csNameTmp] = m;
    return true;
}

unsigned int RobotContactSituations::getControlObjectiveTaskIndex(const std::string &name)
{
    for (unsigned int i = 0; i < controlObjectivesList.size(); i++)
    {
        if (controlObjectivesList[i].compare(name) == 0)
        {
            return i;
        }
    }
    return -1;
}

unsigned int RobotContactSituations::getTotalJoints()
{
    return this->jointDoF;
}

// Call this as last step!
void RobotContactSituations::initGHC()
{
    if (this->amVM)
    {
        this->tasksizes[this->tasksizes.rows() - 1] = this->jointDoF; //mapping_from_VMmanipulator_to_REALmanipulator.size() * 7 /* Hardcoded DOF TODO DLW */;
        // this->controlObjectivesList[index] = coName; // ??? Glaube das brauche ich nicht!
    }
    // RTT::log(RTT::Error) << "initGHC() " << this->getRobotName() << ": tasksizes = " << this->tasksizes << RTT::endlog();
    for (unsigned int i = 0; i < this->tasksizes.size(); i++)
    {
        // if (this->amVM && (i == this->tasksizes.size() - 1))
        // {
        //     this->jacs.push_back(Eigen::MatrixXd::Identity(this->tasksizes[i], this->jointDoF));
        // } else
        // {
        // RTT::log(RTT::Error) << "initGHC() " << this->getRobotName() << ": i = " << i << ", jointDoF = " << jointDoF << ", this->tasksizes[i] =\n" << this->tasksizes[i] << RTT::endlog();
        this->jacs.push_back(Eigen::MatrixXd::Zero(this->tasksizes[i], this->jointDoF));
        // }
        // RTT::log(RTT::Error) << "" << this->getRobotName() << ": this->jacs.push_back ZERO: this->tasksizes[i] = " << this->tasksizes[i] << ", this->jointDoF = " << this->jointDoF << RTT::endlog();
    }
    // if (this->amVM) {
    //     this->addControlObjectiveTask("CombinedRobots", this->numTasks - 1, this->jointDoF);
    // }
    this->updateJCombinedRobots();

    projection_internal_ = Eigen::MatrixXd::Zero(this->jointDoF, this->jointDoF);
    projection_internal_dot_ = Eigen::MatrixXd::Identity(this->jointDoF, this->jointDoF);

    robotstatus_pos_ = Eigen::VectorXd::Zero(this->jointDoF);
    robotstatus_vel_ = Eigen::VectorXd::Zero(this->jointDoF);
    robotstatus_trq_ = Eigen::VectorXd::Zero(this->jointDoF);
    coriolisAndGravity_ = Eigen::VectorXd::Zero(this->jointDoF);

    // RTT::log(RTT::Error) << "" << this->getRobotName() << ": this->numTasks = " << this->numTasks << ", this->tasksizes = " << this->tasksizes << ", this->jointDoF = " << this->jointDoF << RTT::endlog();
    ghcProj.init(this->numTasks, this->tasksizes, this->jointDoF);
}

void RobotContactSituations::addControlObjectiveTask(const std::string &coName, const unsigned int index, const unsigned int taskDoF)
{
    this->tasksizes[index] = taskDoF;
    this->controlObjectivesList[index] = coName;
}

void RobotContactSituations::addCompensationForCOT(const unsigned int index, const unsigned int index_task_to_be_compensated_for)
{
    this->compensation_for_cots[index].push_back(index_task_to_be_compensated_for);
}

void RobotContactSituations::addPortMapping(const unsigned int portId, const unsigned int coIndex, const unsigned int jac_begin_i, const unsigned int jac_begin_j, const unsigned int selfRowSize, const unsigned int tau_begin)
{
    BlockStorage b;
    b.coIndex = coIndex;
    b.jac_begin_i = jac_begin_i;
    b.jac_begin_j = jac_begin_j;
    b.p = selfRowSize;
    b.q = this->jointDoF;
    b.tau_begin = tau_begin * this->jointDoF;
    RTT::log(RTT::Error) << this->getRobotName() << ": addPortMapping for " << portId << RTT::endlog();
    this->coIdandPortPartIdPerPort[portId].push_back(b);
}

void RobotContactSituations::updateJCombinedRobots()
{
    if (this->amVM)
    {
        // TODO assuming that jointDof is the same with the combined manipulators -> However, this could be different though... but not in our kuka case!
        // this->singleRobotJointDoFSize = this->mapping_from_VMmanipulator_to_REALmanipulator[0].jointDof; // TODO DLW kann das weg?
        // if (!this->jacs[this->jacs.size() - 1].isIdentity())
        // {
        this->jacs[this->jacs.size() - 1].setIdentity(this->jointDoF, this->jointDoF);
        // }
    }
}

void RobotContactSituations::updateTorqueCommandFromCombinedRobots(const Eigen::VectorXd &metaCmds, const Eigen::VectorXd &robotstatus_pos, const Eigen::VectorXd &robotstatus_vel, const Eigen::VectorXd &robotstatus_trq, const Eigen::VectorXd &coriolisAndGravity)
{
    // TODO this assumes that we have the right order as we have the control objectives for each robot in the torque vector and the VM: [robot1, robot2] statement.
    for (unsigned int i = 0; i < this->mapping_from_VMmanipulator_to_REALmanipulator.size(); i++)
    {
        VMCombinedRobotStorage v = mapping_from_VMmanipulator_to_REALmanipulator[i];
        // TODO assuming that jointDof is the same with the combined manipulators -> However, this could be different though... but not in our kuka case!
        // if (i == 0) {
        // RTT::log(RTT::Error) << ">>> this->torque_commands[this->torque_commands.size() -1]:\n" << this->torque_commands[this->torque_commands.size() -1] << RTT::endlog();
        // RTT::log(RTT::Error) << ">>> this->torque_commands[...].segment(" << (i * v.jointDof) << ", " << v.jointDof << "):\n" << this->torque_commands[this->torque_commands.size() -1].segment(i * v.jointDof, v.jointDof) << RTT::endlog();
        // RTT::log(RTT::Error) << ">>> metaCmds.torques:\n" << metaCmds.torques << RTT::endlog();
        // RTT::log(RTT::Error) << ">>> metaCmds.torques.segment(" << v.startingIndexInVector << ", " << v.jointDof << "):\n" << metaCmds.torques.segment(v.startingIndexInVector, v.jointDof) << RTT::endlog();
        this->torque_commands[this->torque_commands.size() - 1].segment(i * v.jointDof, v.jointDof) = metaCmds.segment(v.startingIndexInVector, v.jointDof);

        this->robotstatus_pos_.segment(i * v.jointDof, v.jointDof) = robotstatus_pos.segment(v.startingIndexInVector, v.jointDof);
        this->robotstatus_vel_.segment(i * v.jointDof, v.jointDof) = robotstatus_vel.segment(v.startingIndexInVector, v.jointDof);
        this->robotstatus_trq_.segment(i * v.jointDof, v.jointDof) = robotstatus_trq.segment(v.startingIndexInVector, v.jointDof);
        // RTT::log(RTT::Error) << "this->robotstatus_.angles =\n"
        //                      << this->robotstatus_.angles << RTT::endlog();

        this->coriolisAndGravity_.segment(i * v.jointDof, v.jointDof) = coriolisAndGravity.segment(v.startingIndexInVector, v.jointDof);
        // RTT::log(RTT::Error) << "this->coriolisAndGravity_ =\n"
        //                      << this->coriolisAndGravity_ << RTT::endlog();
        // } else {
        //     this->torque_commands[this->torque_commands.size() -1].segment(i * mapping_from_VMmanipulator_to_REALmanipulator[i - 1].jointDof, v.jointDof) = metaCmds.torques.segment(v.startingIndexInVector * v.startingDofInVector, v.jointDof);
        // }
    }
}

void RobotContactSituations::updateJacobian(const unsigned int portId, const Eigen::MatrixXd &metaJac, const Eigen::VectorXd &metaCmds)
{
    if (this->coIdandPortPartIdPerPort.count(portId) > 0)
    {
        // RTT::log(RTT::Error) << this->getRobotName() << ">>> " << portId << " USES THE PORTS DATA!" << RTT::endlog();

        std::vector<BlockStorage> p_vec = this->coIdandPortPartIdPerPort[portId];
        for (unsigned int i = 0; i < p_vec.size(); i++)
        {
            BlockStorage p = p_vec[i];
            // RTT::log(RTT::Error) << this->getRobotName() << " > Set Jacobian at " << p.coIndex << " with " << p.jac_begin_i << ", " << p.jac_begin_j << ", " << p.p << ", " << p.q << ":\n" << metaJac.block(p.jac_begin_i, p.jac_begin_j, p.p, p.q) << RTT::endlog();
            this->jacs[p.coIndex] = metaJac.block(p.jac_begin_i, p.jac_begin_j, /* this->tasksizes[p.coIndex] */ p.p, p.q);
            // RTT::log(RTT::Error) << this->getRobotName() << ">>> >> J =\n"
            //                      << this->jacs[p.coIndex] << "\n from Meta =\n"
            //                      << metaJac << RTT::endlog();
            // RTT::log(RTT::Error) << this->getRobotName() << "> Set Torques at " << p.coIndex << " with " << p.tau_begin << ", " << p.q << ":\n" << metaCmds.torques.segment(p.tau_begin, p.q) << RTT::endlog();
            this->torque_commands[p.coIndex] = metaCmds.segment(p.tau_begin, p.q);
            // RTT::log(RTT::Error) << this->getRobotName() << ">>> >> tau =\n"
            //                      << this->torque_commands[p.coIndex] << "\n from Meta =\n"
            //                      << metaCmds << RTT::endlog();
        }
    }
    // else
    // {
    //     RTT::log(RTT::Error) << this->getRobotName() << "+!+!+! " << portId << " NOTTTTTTTTTTTTTTTTTT!" << RTT::endlog();
    // }
}

void RobotContactSituations::setNewTargetCS(const std::string &csName)
{
    this->lastActiveAlpha = ghcProj.getAlphas();
    // Calculate the selection matrix for the transition: A -> B.
    // For each inequal entry calculate the interval according to % * (top - bot) + bot = val.
    // Where A is bot, B it top, Selection is (top - bot), and % is transitionIndex.

    bool csIsRelevantForUs = this->alpha_per_cs.find(csName) != this->alpha_per_cs.end(); //(this->alpha_per_cs.count(csName) > 0);
    // if (csIsRelevantForUs)
    // {
    //     RTT::log(RTT::Error) << this->getRobotName() << " called setNewTargetCS with " << csName << "yayyyyy!" << RTT::endlog();
    // }

    // RTT::log(RTT::Error) << this->getRobotName() << " called setNewTargetCS with " << csName << ": this->alpha_per_cs.count(csName) = " << this->alpha_per_cs.count(csName) << ",,, " << this->alpha_per_cs[csName] << RTT::endlog();
    // for (const auto &myPair : this->alpha_per_cs)
    // {
    //     RTT::log(RTT::Error) << this->getRobotName() << " called setNewTargetCS with " << csName << ", " << myPair.first << RTT::endlog();
    // }
    float tg = 1.0;
    Eigen::MatrixXd tmpCSAlpha;
    if (csIsRelevantForUs)
    {
        RTT::log(RTT::Error) << this->getRobotName() << " is relevant!!!!" << RTT::endlog();
        tg = 1.0;
        tmpCSAlpha = this->alpha_per_cs[csName];
        if (this->amVM)
        {
            unsigned int lastIndex = this->numTasks - 1;
            for (unsigned int i = 0; i < this->numTasks; i++)
            {
                for (unsigned int j = 0; j < this->numTasks; j++)
                {
                    if (i == j)
                    {
                        if (i == lastIndex)
                        {
                            // Turn CombinedRobot Task OFF.
                            tmpCSAlpha(i, j) = 1.0;
                        }
                    }
                    else if (j == lastIndex)
                    {
                        // Make all other Tasks higher than the CombinedRobot Task.
                        tmpCSAlpha(i, j) = 0.0;
                    }
                    else if (i == lastIndex)
                    {
                        // Make CombinedRobot Task into the nullspace of all other VM Tasks.
                        tmpCSAlpha(i, j) = 1.0;
                    }
                }
            }
        }
    }
    else
    {
        RTT::log(RTT::Error) << this->getRobotName() << " is NOT relevant!!!!" << RTT::endlog();
        tg = 0.0;
        tmpCSAlpha = this->lastActiveAlpha;
        if (this->amVM)
        {
            unsigned int lastIndex = this->numTasks - 1;
            for (unsigned int i = 0; i < this->numTasks; i++)
            {
                for (unsigned int j = 0; j < this->numTasks; j++)
                {
                    if (i == j)
                    {
                        if (i == lastIndex)
                        {
                            // Turn CombinedRobot Task ON.
                            tmpCSAlpha(i, j) = 0.0;
                        }
                        else
                        {
                            // Turn all other Tasks OFF.
                            tmpCSAlpha(i, j) = 1.0;
                        }
                    }
                    else if (j == lastIndex)
                    {
                        // Make all VM Tasks into the nullspace of the CombinedRobot Task.
                        tmpCSAlpha(i, j) = 1.0;
                    }
                    else if (i == lastIndex)
                    {
                        // Make the CombinedRobot Task the highest one.
                        tmpCSAlpha(i, j) = 0.0;
                    }
                }
            }
        }
    }

    this->selection.setZero();

    // if (this->amVM)
    // {
    //     RTT::log(RTT::Error) << "tmpCSAlpha=\n"
    //                          << tmpCSAlpha << "\nlastActiveAlpha=\n"
    //                          << lastActiveAlpha << RTT::endlog();
    //     RTT::log(RTT::Error) << "=======" << RTT::endlog();
    // }

    // unsigned int lastIndex = this->numTasks - 1;
    for (unsigned int i = 0; i < this->numTasks; i++)
    {
        for (unsigned int j = 0; j < this->numTasks; j++)
        {
            float aij = this->lastActiveAlpha(i, j);
            float bij = tmpCSAlpha(i, j);

            // if (this->amVM)
            // {
            //     if (j == lastIndex)
            //     {
            //         this->selection(i, j) = (1 - tg) - aij;
            //         continue;
            //     }
            //     if (i == lastIndex)
            //     {
            //         this->selection(i, j) = tg - aij;
            //         continue;
            //     }
            //     if (i == j)
            //     {
            //         this->selection(i, j) = (1 - tg) - aij;
            //         continue;
            //     }
            // }
            // else
            if (aij != bij)
            {
                this->selection(i, j) = bij - aij;
            }
        }
    }

    if (this->amVM)
    {
        RTT::log(RTT::Error) << this->getRobotName() << " called setNewTargetCS with " << csName << ": relevant(tg) = " << tg << "\nlastActiveAlpha = \n"
                             << this->lastActiveAlpha << "\nselection =\n"
                             << this->selection << "\n Predict Alpha=\n"
                             << (this->selection + this->lastActiveAlpha) << RTT::endlog();
    }
}

// void RobotContactSituations::updateStatusAndGravity(const Eigen::VectorXd &robotstatus_pos, const Eigen::VectorXd &robotstatus_vel, const Eigen::VectorXd &robotstatus_trq, const Eigen::VectorXd &coriolisAndGravity)
// {
//     this->robotstatus_.angles = robotstatus.angles.segment(this->startingIndex_, this->jointDoF);
//     this->robotstatus_.velocities = robotstatus.velocities.segment(this->startingIndex_, this->jointDoF);
//     this->robotstatus_.torques = robotstatus.torques.segment(this->startingIndex_, this->jointDoF);

//     this->coriolisAndGravity_ = coriolisAndGravity.segment(this->startingIndex_, this->jointDoF);
// }

// void RobotContactSituations::setStartingIndex(unsigned int startingIndex)
// {
//     this->startingIndex_ = startingIndex;
// }

void RobotContactSituations::debug_print_state(bool debug)
{
    debug_print_state_ = debug;
}

void RobotContactSituations::updateInertia(const unsigned int portId, const Eigen::MatrixXd &inertia)
{
    if (this->coIdandPortPartIdPerPort.count(portId) > 0)
    {

        BlockStorage p = this->coIdandPortPartIdPerPort[portId][0]; // TODO ugly hack!!!!
        // RTT::log(RTT::Error) << this->getRobotName() << " PortId " << portId << " INERTIA: " << inertia.block(p.jac_begin_j, p.jac_begin_j, p.q, p.q).rows() << ", " << inertia.block(p.jac_begin_j, p.jac_begin_j, p.q, p.q).cols() << ":\n"
        //                      << inertia.block(p.jac_begin_j, p.jac_begin_j, p.q, p.q) << RTT::endlog();
        ghcProj.setInertiaMatrix(inertia.block(p.jac_begin_j, p.jac_begin_j, p.q, p.q));
    }
}

void RobotContactSituations::getCurrentAlphas(Eigen::MatrixXd &out_alphas)
{
    out_alphas = ghcProj.getAlphas();
}

void RobotContactSituations::updateOutputTorques(const float transitionIndex, Eigen::VectorXd &torques_out)
{
    // this->transitionIndex = transitionIndex;
    ghcProj.setJacobianMatrices(this->jacs);
    ghcProj.setAlphas((transitionIndex * this->selection) + this->lastActiveAlpha);

    if (debug_print_state_)
    {
        RTT::log(RTT::Error) << "##################################################" << RTT::endlog();

        for (unsigned int kk = 0; kk < this->jacs.size(); kk++)
        {
            RTT::log(RTT::Error) << this->getRobotName() << ": this->jacs[" << kk << "] =\n"
                                 << (this->jacs[kk]) << RTT::endlog();
        }

        RTT::log(RTT::Error) << this->getRobotName() << ": this->lastActiveAlpha =\n"
                             << (this->lastActiveAlpha) << RTT::endlog();
        RTT::log(RTT::Error) << this->getRobotName() << ": this->selection =\n"
                             << (this->selection) << RTT::endlog();
        RTT::log(RTT::Error) << this->getRobotName() << ": transitionIndex * this->selection =\n"
                             << (transitionIndex * this->selection) << RTT::endlog();
        RTT::log(RTT::Error) << this->getRobotName() << ": (transitionIndex * this->selection) + this->lastActiveAlpha =\n"
                             << ((transitionIndex * this->selection) + this->lastActiveAlpha) << RTT::endlog();

        RTT::log(RTT::Error) << this->getRobotName() << ": inertia =\n"
                             << ghcProj.getInertiaMatrix() << RTT::endlog();
        RTT::log(RTT::Error) << this->getRobotName() << ": inertiaInv =\n"
                             << ghcProj.getInertiaMatrixInv() << RTT::endlog();
    }
    bool ok = ghcProj.getAllGeneralizedProjectors(allProjections, ranks);

    torques_out.setZero();
    for (unsigned int i = 0; i < this->numTasks; i++)
    {
        // if (debug_print_state_)
        // {
        //     RTT::log(RTT::Error) << this->getRobotName() << ": jac[" << i << "] =\n"
        //                          << (this->jacs[i]) << RTT::endlog();
        //     RTT::log(RTT::Error) << this->getRobotName() << ": allProjections[" << i << "] =\n"
        //                          << (allProjections[i]) << RTT::endlog();
        //     RTT::log(RTT::Error) << this->getRobotName() << ": this->torque_commands[" << i << "] =\n"
        //                          << (this->torque_commands[i]) << RTT::endlog();
        //     RTT::log(RTT::Error) << this->getRobotName() << ": allProjections[i] * this->torque_commands[" << i << "] =\n"
        //                          << (allProjections[i] * this->torque_commands[i]) << RTT::endlog();
        // }

        // if (this->compensation_for_cots.size() > i)
        // {
        //     Eigen::VectorXd v = this->torque_commands[i];
        //     std::vector<int> tmp_compen_vec = this->compensation_for_cots[i];
        //     for (unsigned int comp_index = 0; comp_index < tmp_compen_vec.size(); comp_index++)
        //     {
                
        //     }
        //     v = (Eigen::MatrixXd::Identity(7, 7) - in_P_var[eleIdx]) * (v + in_h_var + in_M_var * in_Mc_var[eleIdx].inverse() * (in_P_var[eleIdx] * this->torque_commands[eleIdx] - in_P_var[eleIdx] * in_h_var + in_Pdot_var[eleIdx] * in_robotstatus_var.velocities));
        // }
        

        // TODO ######################### TODO ######################### TODO: If one task needs to compensate for another, add the compensation torques here!
        // Eigen::VectorXd v = this->torque_commands[i];
        // if (taskNeedsToCompensateOtherTasks.count(i) > 0)
        // {
        //     for (unsigned int eleIdx : taskNeedsToCompensateOtherTasks[i])
        //     {
        //         v = (Eigen::MatrixXd::Identity(7, 7) - in_P_var[eleIdx]) * (v + in_h_var + in_M_var * in_Mc_var[eleIdx].inverse() * (in_P_var[eleIdx] * this->torque_commands[eleIdx] - in_P_var[eleIdx] * in_h_var + in_Pdot_var[eleIdx] * in_robotstatus_var.velocities));
        //     }
        // }
        


        Eigen::VectorXd ttmmpp = allProjections[i] * this->torque_commands[i];
        torque_commands_proj_states_stacked.segment(i * this->jointDoF, this->jointDoF) = ttmmpp;
        torques_out += ttmmpp; //v;

        allProjections_stacked.block(i * this->jointDoF, 0, this->jointDoF, this->jointDoF) = allProjections[i];

        if (debug_print_state_)
        {
            RTT::log(RTT::Error) << this->getRobotName() << ": torques_out.torques += ==>\n"
                                 << (torques_out) << RTT::endlog();
        }
    }

    if (debug_print_state_)
    {
        RTT::log(RTT::Error) << this->getRobotName() << "COMPLETE Torques out ==>\n"
                             << (torques_out) << RTT::endlog();
    }

    // if (this->debug_print_state_)
    // {
    //     this->debug_print_state_ = false;
    //     RTT::log(RTT::Error) << this->getRobotName() << ":\nSelection :\n"
    //                          << this->selection << "\nt = " << transitionIndex << "\n(transitionIndex * this->selection) =\n"
    //                          << (transitionIndex * this->selection) << "\nthis->lastActiveAlpha =\n"
    //                          << this->lastActiveAlpha << RTT::endlog();
    //     for (unsigned int i = 0; i < this->numTasks; i++)
    //     {
    //         RTT::log(RTT::Error) << this->getRobotName() << ":\nCurrent Alphas:\n"
    //                              << ghcProj.getAlphas() << "\ntorque_commands[" << i << "] =\n"
    //                              << this->torque_commands[i] << "\nJac[" << i << "] =\n"
    //                              << this->jacs[i] << RTT::endlog();
    //     }
    // }
}

void RobotContactSituations::setPortIdforVMSpecificPorts(unsigned int portIndex)
{
    this->portIndex_ = portIndex;
}

void RobotContactSituations::updateOutputTorquesVM(const float transitionIndex, Eigen::VectorXd &torques_out)
{
    ghcProj.setJacobianMatrices(this->jacs);

    ghcProj.setAlphas((transitionIndex * this->selection) + this->lastActiveAlpha);

    bool ok = ghcProj.getAllGeneralizedProjectors(allProjections, ranks);

    Eigen::VectorXd vec = Eigen::VectorXd::Zero(torques_out.rows()); // TODO we can also move this and only do the initialization once!
    for (unsigned int i = 1; i < this->numTasks - 1; i++)
    {
        vec += /* projection_internal_ * */ allProjections[i] * this->torque_commands[i];
    }

    // ############## NEW IDEA ###################################################################################################
    //
    // ### 1) Project all VM relevant task the DynGHC way, except for the internal force task!
    // ### 2) Use the ProjComb logic to combine the internal force task and the projection from 1)
    // ### => P_if * allProjections[0] * this->torque_commands[0] + P_ef * (allProjections[1] * this->torque_commands[1] + allProjections[2] * this->torque_commands[2]) + allProjections[3] * this->torque_commands[3]
    // ### 3) Also insert the compensation and play around with that!
    //
    // ############## NEW IDEA ###################################################################################################

    // torques_out.torques = this->torque_commands[2]; // Only test!

    Eigen::VectorXd out_torquesForce = this->torque_commands[0];
    Eigen::VectorXd out_torquesMotion = vec;
    // projection_internal_ = identityDOFsizeDOFsize - projection_internal_;

    Eigen::MatrixXd in_inertia_c = projection_internal_ * this->ghcProj.getInertiaMatrix() + identityDOFsizeDOFsize - projection_internal_;

    // out_torquesForce += this->coriolisAndGravity_; // ?
    out_torquesForce += this->ghcProj.getInertiaMatrix() * in_inertia_c.inverse() * (projection_internal_ * out_torquesMotion - projection_internal_ * this->coriolisAndGravity_ + projection_internal_dot_ * this->robotstatus_vel_);

    out_torquesMotion = projection_internal_ * out_torquesMotion;

    out_torquesForce = (identityDOFsizeDOFsize - projection_internal_) * out_torquesForce;

    Eigen::VectorXd outtt = (out_torquesMotion + allProjections[0] * out_torquesForce);
    if (ghcProj.getAlphaLastEntry() < 1.0) // HACk -.- TODO DLW could be a problem if we do a slow switch...
    {
        outtt += allProjections[this->numTasks - 1] * this->torque_commands[this->numTasks - 1];
    }

    // Eigen::VectorXd outtt = (out_torquesMotion + allProjections[0] * out_torquesForce); // + allProjections[this->numTasks - 1] * this->torque_commands[this->numTasks - 1];
    //##########################################

    for (unsigned int i = 0; i < this->mapping_from_VMmanipulator_to_REALmanipulator.size(); i++)
    {
        VMCombinedRobotStorage v = mapping_from_VMmanipulator_to_REALmanipulator[i];
        // TODO assuming that jointDof is the same with the combined manipulators -> However, this could be different though... but not in our kuka case!
        // if (i == 0) {

        // TODO DLW USE BELOW!
        torques_out.segment(v.startingIndexInVector, v.jointDof) = outtt.segment(i * v.jointDof, v.jointDof);
        // RTT::log(RTT::Error) << this->getRobotName() << ": torques_out.torques.segment(" << v.startingIndexInVector << ", " << v.jointDof << ") =\n"
        //                      << vec.segment(i * v.jointDof, v.jointDof) << RTT::endlog();

        // } else {
        //     this->torque_commands[this->torque_commands.size() -1].segment(i * mapping_from_VMmanipulator_to_REALmanipulator[i - 1].jointDof, v.jointDof) = metaCmds.torques.segment(v.startingIndexInVector * v.startingDofInVector, v.jointDof);
        // }
    }

    // RTT::log(RTT::Error) << "##################################################" << RTT::endlog();

    // for (unsigned int kk = 0; kk < this->jacs.size(); kk++)
    // {
    //     RTT::log(RTT::Error) << this->getRobotName() << ": this->jacs[" << kk << "] =\n"
    //                          << (this->jacs[kk]) << RTT::endlog();
    // }

    // RTT::log(RTT::Error) << this->getRobotName() << ": this->lastActiveAlpha =\n"
    //                      << (this->lastActiveAlpha) << RTT::endlog();
    // RTT::log(RTT::Error) << this->getRobotName() << ": this->selection =\n"
    //                      << (this->selection) << RTT::endlog();
    // RTT::log(RTT::Error) << this->getRobotName() << ": transitionIndex * this->selection =\n"
    //                      << (transitionIndex * this->selection) << RTT::endlog();
    // RTT::log(RTT::Error) << this->getRobotName() << ": (transitionIndex * this->selection) + this->lastActiveAlpha =\n"
    //                      << ((transitionIndex * this->selection) + this->lastActiveAlpha) << RTT::endlog();

    // RTT::log(RTT::Error) << this->getRobotName() << ": inertia =\n"
    //                      << ghcProj.getInertiaMatrix() << RTT::endlog();
    // RTT::log(RTT::Error) << this->getRobotName() << ": inertiaInv =\n"
    //                      << ghcProj.getInertiaMatrixInv() << RTT::endlog();
}

// void RobotContactSituations::logDebugInfo(const RTT::LoggerLevel lvl) {
//     // 1) Name of the robot is VM? and this->jointDoF
//     // 2) Number of CS
//     // 3) For each CS:
//     //      - Get Alpha Matrix
//     // 4) Get current this->selection matrix
//     // 5) Get current ghcProj.getAlphas();
//     // 6) Get Robot Inertia Matrix: inertia (TODO I don't know if we need to adjust the M, I need to read up on this!)
//     // 7) Get all this->tasksizes[] with this->controlObjectivesList[]
//     // 8) Get all this->jacs[] dimensions
//     // 9) Get all this->torque_commands[] dimensions

//     // RTT::log(lvl) << "Robot Container: " << this->robotName << "\n"
//     // << "# CS" << "" << "\n"
//     // << ""
//     // << RTT::endlog();
// }
