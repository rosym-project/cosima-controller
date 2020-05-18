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

#include "../../../include/cosima-controller/controller/gravity_compensation/rtt_joint_gravity_compensation.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

using namespace cosima;
using namespace controller;

RTTJointGravComp::RTTJointGravComp(std::string const &name) : RTT::TaskContext(name), include_gravity(true), total_dof_size(0)
{
    addOperation("addRobot", &RTTJointGravComp::addRobot, this)
        .doc("Add a robot in terms of the sequential (controlled) DoF")
        .arg("dof", "degrees of freedom");
    addProperty("include_gravity", this->include_gravity)
        .doc("Include the gravity term in the commanded torques (default: true).");

    this->vec_robot_dof.clear();
}

double RTTJointGravComp::getOrocosTime()
{
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

bool RTTJointGravComp::configureHook()
{
    if (this->vec_robot_dof.size() <= 0)
    {
        PRELOG(Warning) << "Please add at least one robot, before configuring this component!" << RTT::endlog();
        return false;
    }
    // count the total DoF size of the added robots
    this->total_dof_size = 0;

    in_robotstatus_var = sensor_msgs::JointState();
    for (unsigned int i = 0; i < this->vec_robot_dof.size(); i++)
    {
        this->total_dof_size += this->vec_robot_dof[i];
        for (unsigned int j = 0; j < this->vec_robot_dof[i]; j++)
        {
            in_robotstatus_var.position.push_back(0.0);
            in_robotstatus_var.velocity.push_back(0.0);
            in_robotstatus_var.effort.push_back(0.0);
        }
    }

    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("InputPort for reading the current robot joint-space status");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    in_coriolisAndGravity_var = Eigen::VectorXd::Zero(this->total_dof_size);
    in_coriolisAndGravity_port.setName("in_coriolisAndGravity_port");
    in_coriolisAndGravity_port.doc("InputPort for reading coriolisAndGravity vector");
    ports()->addPort(in_coriolisAndGravity_port);
    in_coriolisAndGravity_flow = RTT::NoData;

    out_torques_var = Eigen::VectorXd::Zero(this->total_dof_size);
    out_torques_port.setName("out_torques_port");
    out_torques_port.doc("OutputPort for sending the command torques");
    out_torques_port.setDataSample(out_torques_var);
    ports()->addPort(out_torques_port);

    return true;
}

bool RTTJointGravComp::startHook()
{
    return true;
}

void RTTJointGravComp::updateHook()
{
    out_torques_var.setZero();

    in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    if (in_robotstatus_flow == RTT::NoData)
    {
        PRELOG(Error) << "Houston we have a problem! No data on in_robotstatus_port." << RTT::endlog();
        return; // Return is better, because in this case we don't send a command at all!
    }

    if (this->include_gravity)
    {
        in_coriolisAndGravity_flow = in_coriolisAndGravity_port.read(in_coriolisAndGravity_var);
        if (in_coriolisAndGravity_flow == RTT::NoData)
        {
            PRELOG(Error) << "Houston we have a problem! No data on in_coriolisAndGravity_port." << RTT::endlog();
            return; // Return is better, because in this case we don't send a command at all!
        }

        out_torques_var = in_coriolisAndGravity_var;
    }

    out_torques_port.write(out_torques_var);
}

void RTTJointGravComp::stopHook()
{
}

void RTTJointGravComp::cleanupHook()
{
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
}

bool RTTJointGravComp::addRobot(const unsigned int &dof)
{
    if (dof <= 0)
    {
        PRELOG(Error) << "In order to add a robot, it needs to have at least DoF(" << dof << ") > 0" << RTT::endlog();
        return false;
    }
    this->vec_robot_dof.push_back(dof);
    return true;
}

//this macro should appear only once per library
// ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::controller::RTTJointGravComp)
