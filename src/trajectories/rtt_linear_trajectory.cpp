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

LinearTrajectory::LinearTrajectory(std::string const &name) : RTT::TaskContext(name), once(false)
{
    
}

double LinearTrajectory::getOrocosTime()
{
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

bool LinearTrajectory::configureHook()
{
    return true;
}

bool LinearTrajectory::startHook()
{
    st = this->getOrocosTime();
    once = false;
    return true;
}

void LinearTrajectory::updateHook()
{
    
}

void LinearTrajectory::stopHook()
{
}

void LinearTrajectory::cleanupHook()
{
    if (this->ports()->getPort("in_pose_port"))
    {
        this->ports()->removePort("in_pose_port");
    }
}

//this macro should appear only once per library
// ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::trajectories::LinearTrajectory)
