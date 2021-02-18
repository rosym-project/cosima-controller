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

#include "../../include/cosima-controller/coordination/monitor_if.hpp"

using namespace cosima;
using namespace coordination;

MonitorIF::MonitorIF(const std::string &name, RTT::TaskContext *tc)
{
    this->tc = tc;
    this->monitor_name = name;

    this->port_name = "out_monitor_" + this->monitor_name + "_event_port";

    if (this->tc->getPort(this->port_name))
    {
        this->tc->ports()->removePort(this->port_name);
    }

    this->out_event_var = false;
    this->out_event_port.setName(this->port_name);
    this->out_event_port.doc("Output port for sending the event of the monitor: " + this->monitor_name);
    this->out_event_port.setDataSample(this->out_event_var);
    this->tc->ports()->addPort(this->out_event_port);
}