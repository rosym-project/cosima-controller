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

#include "../../include/cosima-controller/trajectories/rtt_linear_trajectory.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

using namespace cosima;
using namespace trajectories;

LinearTrajectory::LinearTrajectory(std::string const &name) : RTT::TaskContext(name), once(false), DoF(7), new_js_command(false), max_rad(0.0872665), timestep(0.0)
{
    this->addProperty("max_rad", max_rad);

    this->addOperation("setJointTargetEigen", &LinearTrajectory::setJointTargetEigen, this);
}

double LinearTrajectory::getOrocosTime()
{
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

bool LinearTrajectory::configureHook()
{
    if (ports()->getPort("in_robotstatus_port"))
    {
        ports()->removePort("in_robotstatus_port");
    }
    if (ports()->getPort("out_joint_cmd_port"))
    {
        ports()->removePort("out_joint_cmd_port");
    }

    this->in_robotstatus_var = sensor_msgs::JointState();
    this->out_joint_cmd_var = trajectory_msgs::JointTrajectoryPoint();
    for (unsigned int i = 0; i < this->DoF; i++)
    {
        in_robotstatus_var.position.push_back(0.0);
        in_robotstatus_var.velocity.push_back(0.0);
        in_robotstatus_var.effort.push_back(0.0);

        out_joint_cmd_var.positions.push_back(0.0);
        out_joint_cmd_var.velocities.push_back(0.0);
        out_joint_cmd_var.accelerations.push_back(0.0);
        out_joint_cmd_var.effort.push_back(0.0);
    }

    this->in_robotstatus_flow = RTT::NoData;
    this->in_robotstatus_port.setName("in_robotstatus_port");
    this->in_robotstatus_port.doc("Input port for receiving the current jointspace status");
    ports()->addPort(this->in_robotstatus_port);

    this->out_joint_cmd_port.setName("out_joint_cmd_port");
    this->out_joint_cmd_port.doc("Output port for sending the desired jointspace command");
    this->out_joint_cmd_port.setDataSample(this->out_joint_cmd_var);
    ports()->addPort(this->out_joint_cmd_port);

    this->js_target = Eigen::VectorXd::Zero(this->DoF);
    this->js_current = Eigen::VectorXd::Zero(this->DoF);

    this->js_out = Eigen::VectorXd::Zero(this->DoF);

    return true;
}

bool LinearTrajectory::startHook()
{
    this->timestep = 0.0;
    this->once = false;
    this->new_js_command = false;
    // double t = (time_diff - starttrajectory_time);
    //         if (t <= js_trap_generator.Duration())
    //         {
    //             initialBoxPosition(0) = js_trap_generator.Pos(t);
    //         }
    //  js_trap_generator.SetProfile(t_dist(0), initialBoxPositionBAK(0));
    // this->js_trap_generator = KDL::VelocityProfile_Trap(2.0, 0.1);
    return true;
}

void LinearTrajectory::updateHook()
{
    if (this->new_js_command)
    {
        // get current joint position
        this->in_robotstatus_flow = this->in_robotstatus_port.read(this->in_robotstatus_var);
        if (this->in_robotstatus_flow == RTT::NoData)
        {
            return;
        }

        for (unsigned int i = 0; i < this->DoF; i++)
        {
            this->js_current(i) = this->in_robotstatus_var.position[i];
        } 

        this->js_out = this->js_current;

        this->new_js_command = false;
        this->once = true;
        this->timestep = 0.0;

        // Calculate norm direction of error vector
        // Eigen::VectorXd js_error = this->js_target - this->js_current;
        // Eigen::VectorXd js_error_normalized = js_error.normalized();
        // Eigen::VectorXd out = this->js_current + js_error_normalized * (this->max_rad * this->getPeriod());
        // this->js_current += js_error_normalized * (this->max_rad * this->getPeriod());
    
        
    }

    if (!this->once)
    {
        return;
    }

    Eigen::VectorXd js_error = this->js_target - this->js_current;
    Eigen::VectorXd js_error_normalized = js_error.normalized();
    this->js_out += js_error_normalized * (this->max_rad * this->getPeriod());
    if ((this->js_out-this->js_current).norm() >= js_error.norm())
    {
        this->js_out = this->js_target;
    }

    // Convert to output
    for (unsigned int i = 0; i < this->DoF; i++)
    {
        out_joint_cmd_var.positions[i] = this->js_out(i);
        out_joint_cmd_var.velocities[i] = 0.0;
        out_joint_cmd_var.accelerations[i] = 0.0;
        out_joint_cmd_var.effort[i] = 0.0;
    }
    out_joint_cmd_port.write(out_joint_cmd_var);
}

void LinearTrajectory::stopHook()
{
}

void LinearTrajectory::cleanupHook()
{
}

void LinearTrajectory::setJointTargetEigen(const Eigen::VectorXd &target)
{
    for (unsigned int i = 0; i < this->DoF; i++)
    {
        this->js_target(i) = target(i);
    }
    this->new_js_command = true;
}

//this macro should appear only once per library
// ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::trajectories::LinearTrajectory)
