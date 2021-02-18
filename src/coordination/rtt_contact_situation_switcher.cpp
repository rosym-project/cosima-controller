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

#include "../../include/cosima-controller/coordination/rtt_contact_situation_switcher.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

using namespace cosima;
using namespace coordination;

ContactSituationSwitcher::ContactSituationSwitcher(std::string const &name) : RTT::TaskContext(name)
{
    // this->addProperty("traj_max_vel", traj_max_vel);
}

double ContactSituationSwitcher::getOrocosTime()
{
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

void ContactSituationSwitcher::addWrenchMonitor(std::string const &name)
{
    map_monitors_wrench[name] = std::shared_ptr<MonitorWrench>(new MonitorWrench(name, this));
}

void ContactSituationSwitcher::setWrenchMonitorBounds(std::string const &name, unsigned int const &dimension, double lower, double upper)
{
    map_monitors_wrench[name]->activateBounds(dimension, lower, upper);
}

bool ContactSituationSwitcher::configureHook()
{
    return true;
}

bool ContactSituationSwitcher::startHook()
{
    // st = this->getOrocosTime();
    return true;
}

void ContactSituationSwitcher::updateHook()
{
    for (auto const& x : map_monitors_wrench)
    {
        x.second->eval();
    }
}

void ContactSituationSwitcher::stopHook()
{
}

void ContactSituationSwitcher::cleanupHook()
{
    // map_monitors_wrench.clear();
}

//this macro should appear only once per library
// ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::coordination::ContactSituationSwitcher)


// https://docs.orocos.org/rtt/orocos-rtt-scripting.html#orocos-state-descriptions
// https://docs.orocos.org/rtt/orocos-rtt-scripting.html#program-syntax
// https://docs.orocos.org/rtt/tutorials/hello_6.html#writing-a-program