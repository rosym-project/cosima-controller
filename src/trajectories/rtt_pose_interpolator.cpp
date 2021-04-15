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

#include "../../include/cosima-controller/trajectories/rtt_pose_interpolator.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

using namespace cosima;
using namespace trajectories;

PoseInterpolator::PoseInterpolator(std::string const &name) : cogimon::RTTIntrospectionBase(name)
{
    traj_max_vel = 2.0;
    traj_max_acc = 0.1;
    traj_time = 1.5;
    first_iter = true;

    ignore_traj = false;

    this->addProperty("traj_max_vel", traj_max_vel);
    this->addProperty("traj_max_acc", traj_max_acc);
    this->addProperty("traj_time", traj_time);
    this->addProperty("ignore_traj", ignore_traj);
}

double PoseInterpolator::getOrocosTime()
{
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

bool PoseInterpolator::configureHookInternal()
{
    if (this->ports()->getPort("in_pose_port"))
    {
        this->ports()->removePort("in_pose_port");
    }

    if (this->ports()->getPort("in_current_pose_port"))
    {
        this->ports()->removePort("in_current_pose_port");
    }

    if (this->ports()->getPort("out_pose_matrix_port"))
    {
        this->ports()->removePort("out_pose_matrix_port");
    }

    if (this->ports()->getPort("out_pose_port"))
    {
        this->ports()->removePort("out_pose_port");
    }

    in_pose_var = geometry_msgs::Pose();
    in_pose_port.setName("in_pose_port");
    in_pose_port.doc("InputPort to receive the initial pose command");
    ports()->addPort(in_pose_port);
    in_pose_flow = RTT::NoData;

    in_pose_pt_var = geometry_msgs::Pose();
    in_pose_pt_port.setName("in_pose_pt_port");
    in_pose_pt_port.doc("InputPort to receive the pass-through pose commands (use for high frequency)");
    ports()->addPort(in_pose_pt_port);
    in_pose_pt_flow = RTT::NoData;

    // in_current_pose_var = Eigen::MatrixXd::Zero(4,4);
    // in_current_pose_port.setName("in_current_pose_port");
    // in_current_pose_port.doc("InputPort to receive the current pose matrix");
    // ports()->addPort(in_current_pose_port);
    // in_current_pose_flow = RTT::NoData;
    in_current_pose_var = geometry_msgs::Pose();
    in_current_pose_port.setName("in_current_pose_port");
    in_current_pose_port.doc("InputPort to receive the current pose matrix");
    ports()->addPort(in_current_pose_port);
    in_current_pose_flow = RTT::NoData;

    out_pose_matrix_var = Eigen::MatrixXd::Zero(4,4);
    out_pose_matrix_port.setName("out_pose_matrix_port");
    out_pose_matrix_port.doc("OutputPort for the interpolated pose matrix commands");
    out_pose_matrix_port.setDataSample(out_pose_matrix_var);
    ports()->addPort(out_pose_matrix_port);

    out_pose_var = geometry_msgs::Pose();
    out_pose_port.setName("out_pose_port");
    out_pose_port.doc("OutputPort for the interpolated pose commands");
    out_pose_port.setDataSample(out_pose_var);
    ports()->addPort(out_pose_port);

    first = Eigen::Vector3d::Zero();
    start = Eigen::Vector3d::Zero();
    goal = Eigen::Vector3d::Zero();
    diff = Eigen::Vector3d::Zero();
    qFirst = Eigen::Quaterniond::Identity();
    qStart = Eigen::Quaterniond::Identity();
    qGoal = Eigen::Quaterniond::Identity();

    return true;
}

bool PoseInterpolator::startHookInternal()
{
    st = this->getOrocosTime();
    once = false;
    first_iter = true;
    return true;
}

void PoseInterpolator::updateHookInternal()
{
    in_current_pose_flow = in_current_pose_port.read(in_current_pose_var);
    if (in_current_pose_flow == RTT::NoData)
    {
        return;
    }

    if (first_iter)
    {
        // first = in_current_pose_var.block<3,1>(0,3);
        // qFirst = Eigen::Quaterniond(in_current_pose_var.block<3,3>(0,0));
        first << in_current_pose_var.position.x, in_current_pose_var.position.y, in_current_pose_var.position.z;
        qFirst.x() = in_current_pose_var.orientation.x;
        qFirst.y() = in_current_pose_var.orientation.y;
        qFirst.z() = in_current_pose_var.orientation.z;
        qFirst.w() = in_current_pose_var.orientation.w;

        first_iter = false;
    }

    in_pose_pt_flow = in_pose_pt_port.read(in_pose_pt_var);
    if (in_pose_pt_flow == RTT::NewData)
    {
        goal << in_pose_pt_var.position.x, in_pose_pt_var.position.y, in_pose_pt_var.position.z;
        qGoal.x() = in_pose_pt_var.orientation.x;
        qGoal.y() = in_pose_pt_var.orientation.y;
        qGoal.z() = in_pose_pt_var.orientation.z;
        qGoal.w() = in_pose_pt_var.orientation.w;

        ignore_traj = true;

        if (!once)
        {
            once = true;
        }
    }
    else
    {
        in_pose_flow = in_pose_port.read(in_pose_var);
        if (in_pose_flow == RTT::NewData)
        {
            // start = in_current_pose_var.block<3,1>(0,3);
            // qStart = Eigen::Quaterniond(in_current_pose_var.block<3,3>(0,0));
            start << in_current_pose_var.position.x, in_current_pose_var.position.y, in_current_pose_var.position.z;
            qStart.x() = in_current_pose_var.orientation.x;
            qStart.y() = in_current_pose_var.orientation.y;
            qStart.z() = in_current_pose_var.orientation.z;
            qStart.w() = in_current_pose_var.orientation.w;

            goal << in_pose_var.position.x, in_pose_var.position.y, in_pose_var.position.z;
            qGoal.x() = in_pose_var.orientation.x;
            qGoal.y() = in_pose_var.orientation.y;
            qGoal.z() = in_pose_var.orientation.z;
            qGoal.w() = in_pose_var.orientation.w;

            diff = goal - start;

            trap_gen = KDL::VelocityProfile_Trap(traj_max_vel, traj_max_acc);
            trap_gen.SetProfileDuration(0.0, 1.0, traj_time);

            st = this->getOrocosTime();

            if (!once)
            {
                once = true;
            }

            ignore_traj = false;
        }
    }
    
    if (once)
    {
        if (ignore_traj)
        {
            out_pose_matrix_var.block<3,1>(0,3) = goal;
            out_pose_matrix_var.block<3,3>(0,0) = qGoal.toRotationMatrix();

            out_pose_var.position.x = goal(0);
            out_pose_var.position.y = goal(1);
            out_pose_var.position.z = goal(2);
            out_pose_var.orientation.w = qGoal.w();
            out_pose_var.orientation.x = qGoal.x();
            out_pose_var.orientation.y = qGoal.y();
            out_pose_var.orientation.z = qGoal.z();
        }
        else
        {
            double t = (this->getOrocosTime() - st);
            double timeIndex = 1.0;

            if (t <= trap_gen.Duration())
            {
                timeIndex = trap_gen.Pos(t);
            }

            out_pose_matrix_var.block<3,1>(0,3) = start + timeIndex * diff;
            out_pose_matrix_var.block<3,3>(0,0) = qStart.slerp(timeIndex, qGoal).toRotationMatrix();

            out_pose_var.position.x = goal(0);
            out_pose_var.position.y = goal(1);
            out_pose_var.position.z = goal(2);
            out_pose_var.orientation.w = qGoal.w();
            out_pose_var.orientation.x = qGoal.x();
            out_pose_var.orientation.y = qGoal.y();
            out_pose_var.orientation.z = qGoal.z();
        }

        out_pose_matrix_var(3,3) = 1;
    }
    else
    {
        // Always set the first starting pose
        out_pose_matrix_var.block<3,1>(0,3) = first;
        out_pose_matrix_var.block<3,3>(0,0) = qFirst.toRotationMatrix();

        out_pose_var.position.x = goal(0);
        out_pose_var.position.y = goal(1);
        out_pose_var.position.z = goal(2);
        out_pose_var.orientation.w = qGoal.w();
        out_pose_var.orientation.x = qGoal.x();
        out_pose_var.orientation.y = qGoal.y();
        out_pose_var.orientation.z = qGoal.z();

        out_pose_matrix_var(3,3) = 1;
    }

    // TODO perhaps put the write into the if above to not send all the time.
    // Downside would be if one message would be lost...
    if (!this->out_pose_matrix_var.isZero())
    {
        // RTT::log(RTT::Error) << out_pose_matrix_var << RTT::endlog();
        this->out_pose_matrix_port.write(this->out_pose_matrix_var);
        this->out_pose_port.write(this->out_pose_var);
    }
}

void PoseInterpolator::stopHookInternal()
{
}

void PoseInterpolator::cleanupHookInternal()
{
    if (this->ports()->getPort("in_pose_port"))
    {
        this->ports()->removePort("in_pose_port");
    }

    if (this->ports()->getPort("in_current_pose_port"))
    {
        this->ports()->removePort("in_current_pose_port");
    }

    if (this->ports()->getPort("out_pose_matrix_port"))
    {
        this->ports()->removePort("out_pose_matrix_port");
    }

    if (this->ports()->getPort("out_pose_port"))
    {
        this->ports()->removePort("out_pose_port");
    }
}

//this macro should appear only once per library
// ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::trajectories::PoseInterpolator)
