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

#include "../../include/cosima-controller/controller/rtt_joint_pd_controller.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

using namespace cosima;
using namespace controller;

RTTJointPDCtrl::RTTJointPDCtrl(std::string const &name) : RTT::TaskContext(name), total_dof_size(0), useFilter(true), include_gravity(true)
{
    addOperation("addRobot", &RTTJointPDCtrl::addRobot, this)
        .doc("Add a robot in terms of the sequential (controlled) DoF")
        .arg("dof", "degrees of freedom");
    addProperty("include_gravity", this->include_gravity)
        .doc("Include the gravity term in the commanded torques (default: true).");

    addOperation("setGains", &RTTJointPDCtrl::setGains, this);
    addOperation("setPositionCmd", &RTTJointPDCtrl::setPositionCmd, this);

    this->kp = 0.0;
    addProperty("kp", this->kp);
    this->kd = 0.0;
    addProperty("kd", this->kd);

    this->timeStep = 0.005;
    addProperty("timeStep", this->timeStep);

    this->vec_robot_dof.clear();

    this->useFilter = true;
    addProperty("useFilter", this->useFilter);
    this->maxVelRad = 0.2;
    addProperty("maxVelRad", this->maxVelRad);

    this->gain_speed = 1.0;
    addProperty("gain_speed", this->gain_speed);
    this->target_kp = this->kp;
    addProperty("target_kp", this->target_kp);
    this->target_kd = this->kd;
    addProperty("target_kd", this->target_kd);

    // trap_gen_gains = KDL::VelocityProfile_Trap(2.0, 0.1);

    
    addProperty("qError", this->qError);
    addProperty("qdError", this->qdError);

    addProperty("lastCommand", this->lastCommand);
}

void RTTJointPDCtrl::setGains(const double &kp, const double &kd)
{
    // this->kp = kp;
    // this->kd = kd;

    this->target_kp = kp;
    this->target_kd = kd;   
}

double RTTJointPDCtrl::getOrocosTime()
{
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

void RTTJointPDCtrl::setPositionCmd(const Eigen::VectorXd &config)
{
    // Sanity check
    if (config.size() != this->total_dof_size)
    {
        return;
    }

    for (unsigned int i = 0; i < this->virtual_joint_cmd_var.positions.size(); i++)
    {
        this->virtual_joint_cmd_var.positions[i] = config(i);
    }
    this->virtual_joint_cmd_var.velocities.clear();
    this->virtual_joint_cmd_var.accelerations.clear();
    this->virtual_joint_cmd_var.effort.clear();
    this->virtual_joint_cmd_flow = RTT::NewData;
}

bool RTTJointPDCtrl::configureHook()
{
    if (this->vec_robot_dof.size() <= 0)
    {
        PRELOG(Warning) << "Please add at least one robot, before configuring this component!" << RTT::endlog();
        return false;
    }
    // count the total DoF size of the added robots
    this->total_dof_size = 0;

    in_robotstatus_var = sensor_msgs::JointState();
    in_joint_cmd_var = trajectory_msgs::JointTrajectoryPoint();
    for (unsigned int i = 0; i < this->vec_robot_dof.size(); i++)
    {
        this->total_dof_size += this->vec_robot_dof[i];
        for (unsigned int j = 0; j < this->vec_robot_dof[i]; j++)
        {
            in_robotstatus_var.position.push_back(0.0);
            in_robotstatus_var.velocity.push_back(0.0);
            in_robotstatus_var.effort.push_back(0.0);

            in_joint_cmd_var.positions.push_back(0.0);
            in_joint_cmd_var.velocities.push_back(0.0);
            in_joint_cmd_var.accelerations.push_back(0.0);
            in_joint_cmd_var.effort.push_back(0.0);

            virtual_joint_cmd_var.positions.push_back(0.0);
            virtual_joint_cmd_var.velocities.push_back(0.0);
            virtual_joint_cmd_var.accelerations.push_back(0.0);
            virtual_joint_cmd_var.effort.push_back(0.0);
        }
    }

    in_joint_cmd_port.setName("in_joint_cmd_port");
    in_joint_cmd_port.doc("InputPort for reading joint-space commands");
    ports()->addPort(in_joint_cmd_port);
    in_joint_cmd_flow = RTT::NoData;
    virtual_joint_cmd_flow = RTT::NoData;

    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("InputPort for reading the current robot joint-space status");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    in_coriolisAndGravity_var = Eigen::VectorXd::Zero(this->total_dof_size);
    in_coriolisAndGravity_port.setName("in_coriolisAndGravity_port");
    in_coriolisAndGravity_port.doc("InputPort for reading coriolisAndGravity vector");
    ports()->addPort(in_coriolisAndGravity_port);
    in_coriolisAndGravity_flow = RTT::NoData;

    in_inertia_var = Eigen::MatrixXd::Zero(this->total_dof_size, this->total_dof_size);
    in_inertia_port.setName("in_inertia_port");
    in_inertia_port.doc("InputPort for reading joint-space inertia");
    ports()->addPort(in_inertia_port);
    in_inertia_flow = RTT::NoData;

    out_torques_var = Eigen::VectorXd::Zero(this->total_dof_size);
    out_torques_port.setName("out_torques_port");
    out_torques_port.doc("OutputPort for sending the command torques");
    out_torques_port.setDataSample(out_torques_var);
    ports()->addPort(out_torques_port);

    qError = Eigen::VectorXd::Zero(this->total_dof_size);
    qdError = Eigen::VectorXd::Zero(this->total_dof_size);
    ones = Eigen::VectorXd::Ones(this->total_dof_size);

    errorT = Eigen::VectorXd::Zero(this->total_dof_size);
    lastCommand = Eigen::VectorXd::Zero(this->total_dof_size);
    scalingVec = Eigen::VectorXd::Ones(this->total_dof_size);

    // householderQR_solver = Eigen::HouseholderQR<Eigen::MatrixXd>(this->total_dof_size, this->total_dof_size);
    ldlt_solver = Eigen::LDLT<Eigen::MatrixXd>(this->total_dof_size);

    return true;
}

bool RTTJointPDCtrl::startHook()
{
    return true;
}

void RTTJointPDCtrl::updateHook()
{
    out_torques_var.setZero();

    in_joint_cmd_flow = in_joint_cmd_port.read(in_joint_cmd_var);
    if (in_joint_cmd_flow != RTT::NewData)
    {
        if (virtual_joint_cmd_flow == RTT::NewData)
        {
            in_joint_cmd_var = virtual_joint_cmd_var;
            virtual_joint_cmd_flow = RTT::OldData;

            PRELOG(Error) << "Received (virtual) command: " << virtual_joint_cmd_var << RTT::endlog();
        }
    }
    // else if (in_joint_cmd_flow == RTT::NewData)
    // {
    //     // PRELOG(Error) << "Received command: " << in_joint_cmd_var << RTT::endlog();
    // }

    in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    if (in_robotstatus_flow == RTT::NoData)
    {
        PRELOG(Error) << "Houston we have a problem! No data on in_robotstatus_port." << RTT::endlog();
        return; // Return is better, because in this case we don't send a command at all!
    }

    in_inertia_flow = in_inertia_port.read(in_inertia_var);
    if (in_inertia_flow == RTT::NoData)
    {
        PRELOG(Error) << "Houston we have a problem! No data on in_inertia_port." << RTT::endlog();
        return; // Return is better, because in this case we don't send a command at all!
    }

    in_coriolisAndGravity_flow = in_coriolisAndGravity_port.read(in_coriolisAndGravity_var);
    if (in_coriolisAndGravity_flow == RTT::NoData)
    {
        PRELOG(Error) << "Houston we have a problem! No data on in_coriolisAndGravity_port." << RTT::endlog();
        return; // Return is better, because in this case we don't send a command at all!
    }

    // Velocity filtering mechanism
    scalingVec.fill(1.0);
    if (this->useFilter)
    {
        for (unsigned int i = 0; i < this->total_dof_size; i++)
        {
            errorT(i) = this->in_joint_cmd_var.positions[i] - lastCommand(i);
        }
        unsigned int offset_counter = 0;
        for (unsigned int i = 0; i < this->vec_robot_dof.size(); i++)
        {
            unsigned int dofsize = this->vec_robot_dof.at(i);

            float jointDistance = errorT.segment(offset_counter, dofsize).norm();
            float jointDistPerPeriodStep = maxVelRad * this->getPeriod();
            if (jointDistance > jointDistPerPeriodStep)
            {
                scalingVec.segment(offset_counter, dofsize).fill(jointDistPerPeriodStep / jointDistance);
            }
            offset_counter += dofsize;
        }

        lastCommand = lastCommand + errorT.cwiseProduct(scalingVec);
    }
    else
    {
        for (unsigned int i = 0; i < this->total_dof_size; i++)
        {
            lastCommand(i) = this->in_joint_cmd_var.positions[i];
        }
    }

    // Here is an alternate way to convert from std to eigen
    // Eigen::VectorXd fdb_positions = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(in_robotstatus_var.position.data(), in_robotstatus_var.position.size());
    // Eigen::VectorXd fdb_velocities = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(in_robotstatus_var.velocity.data(), in_robotstatus_var.velocity.size());
    for (unsigned int i = 0; i < this->total_dof_size; i++)
    {
        qError(i) = lastCommand(i) - in_robotstatus_var.position[i];
        qdError(i) = this->in_joint_cmd_var.velocities[i] - in_robotstatus_var.velocity[i];
    }

    // adjust gains
    kp += (this->target_kp - kp) * (this->gain_speed * this->getPeriod());
    kd += (this->target_kd - kd) * (this->gain_speed * this->getPeriod());

    Eigen::MatrixXd Kd = (kd * ones).asDiagonal();

    Eigen::VectorXd pd = ((kp * ones).asDiagonal()) * qError.matrix() + Kd * qdError.matrix();

    Eigen::MatrixXd M = in_inertia_var + Kd * timeStep;

    // https://eigen.tuxfamily.org/dox-devel/group__TopicLinearAlgebraDecompositions.html
    // https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
    ldlt_solver.compute(M);
    Eigen::VectorXd qddot = ldlt_solver.solve(-in_coriolisAndGravity_var + pd);
    // Solver alternative: Slow but very accurate
    // Eigen::VectorXd qddot = M.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    out_torques_var = pd - (Kd * qddot) * timeStep;
    if (include_gravity)
    {
        out_torques_var += in_coriolisAndGravity_var;
    }

    // // Debug simple controller
    // for (unsigned int i = 0; i < this->total_dof_size; i++)
    // {
    //     out_torques_var(i) = kp * qError(i) + kd * qdError(i);
    //     //+ in_coriolisAndGravity_var(i);
    // }

    out_torques_port.write(out_torques_var);
}

void RTTJointPDCtrl::stopHook()
{
}

void RTTJointPDCtrl::cleanupHook()
{
    if (this->ports()->getPort("in_joint_cmd_port"))
    {
        this->ports()->removePort("in_joint_cmd_port");
    }

    if (this->ports()->getPort("in_robotstatus_port"))
    {
        this->ports()->removePort("in_robotstatus_port");
    }

    if (this->ports()->getPort("in_coriolisAndGravity_port"))
    {
        this->ports()->removePort("in_coriolisAndGravity_port");
    }

    if (this->ports()->getPort("out_torques_port"))
    {
        this->ports()->removePort("out_torques_port");
    }

    if (this->ports()->getPort("in_inertia_port"))
    {
        this->ports()->removePort("in_inertia_port");
    }
}

bool RTTJointPDCtrl::addRobot(const unsigned int &dof)
{
    if (dof <= 0)
    {
        PRELOG(Warning) << "In order to add a robot, it needs to have at least DoF(" << dof << ") > 0" << RTT::endlog();
        return false;
    }
    this->vec_robot_dof.push_back(dof);
    return true;
}

//this macro should appear only once per library
// ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::controller::RTTJointPDCtrl)
