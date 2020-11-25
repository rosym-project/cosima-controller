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

PoseInterpolator::PoseInterpolator(std::string const &name) : RTT::TaskContext(name)
{
    traj_max_vel = 2.0;
    traj_max_acc = 0.1;
    traj_time = 1.5;

    this->addProperty("traj_max_vel", traj_max_vel);
    this->addProperty("traj_max_acc", traj_max_acc);
    this->addProperty("traj_time", traj_time);
}

double PoseInterpolator::getOrocosTime()
{
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

bool PoseInterpolator::configureHook()
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

    in_pose_var = geometry_msgs::Pose();
    in_pose_port.setName("in_pose_port");
    in_pose_port.doc("InputPort to receive the initial pose command");
    ports()->addPort(in_pose_port);
    in_pose_flow = RTT::NoData;

    in_current_pose_var = Eigen::MatrixXd::Zero(4,4);
    in_current_pose_port.setName("in_current_pose_port");
    in_current_pose_port.doc("InputPort to receive the current pose matrix");
    ports()->addPort(in_current_pose_port);
    in_current_pose_flow = RTT::NoData;

    out_pose_matrix_var = Eigen::MatrixXd::Zero(4,4);
    out_pose_matrix_port.setName("out_pose_matrix_port");
    out_pose_matrix_port.doc("OutputPort for the interpolated pose matrix commands");
    out_pose_matrix_port.setDataSample(out_pose_matrix_var);
    ports()->addPort(out_pose_matrix_port);

    return true;
}

bool PoseInterpolator::startHook()
{
    st = this->getOrocosTime();
    once = false;
    return true;
}

void PoseInterpolator::updateHook()
{
    in_current_pose_flow = in_current_pose_port.read(in_current_pose_var);
    if (in_current_pose_flow == RTT::NoData)
    {
        return;
    }

    Eigen::Vector3d start = in_current_pose_var.block<3,1>(0,3);
    Eigen::Vector3d goal;
    Eigen::Vector3d diff;
    Eigen::Quaterniond qStart(in_current_pose_var.block<3,3>(0,0));
    Eigen::Quaterniond qGoal;

    in_pose_flow = in_pose_port.read(in_pose_var);
    if (in_pose_flow == RTT::NewData)
    {
        goal << in_pose_var.position.x, in_pose_var.position.y, in_pose_var.position.z;

        diff = goal - start;

        qGoal.x() = in_pose_var.orientation.x;
        qGoal.y() = in_pose_var.orientation.y;
        qGoal.z() = in_pose_var.orientation.z;
        qGoal.w() = in_pose_var.orientation.w;

        trap_gen = KDL::VelocityProfile_Trap(traj_max_vel, traj_max_acc);
        trap_gen.SetProfileDuration(0.0, 1.0, traj_time);

        st = this->getOrocosTime();

        if (!once)
        {
            once = true;
        }
    }
    
    if (once)
    {
        double t = (this->getOrocosTime() - st);
        double timeIndex = 1.0;
        if (t <= trap_gen.Duration())
        {
            timeIndex = trap_gen.Pos(t);
        }

        out_pose_matrix_var.block<3,1>(0,3) = start + timeIndex * diff;

        Eigen::Quaterniond qres = qStart.slerp(timeIndex, qGoal);

        out_pose_matrix_var.block<3,3>(0,0) = qres.toRotationMatrix();

        out_pose_matrix_var(3,3) = 1;
    }

    // TODO perhaps put the write into the if above to not send all the time.
    // Downside would be if one message would be lost...
    if (!this->out_pose_matrix_var.isZero())
    {
        this->out_pose_matrix_port.write(this->out_pose_matrix_var);
    }
}

void PoseInterpolator::stopHook()
{
}

void PoseInterpolator::cleanupHook()
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
}

//this macro should appear only once per library
// ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::trajectories::PoseInterpolator)
