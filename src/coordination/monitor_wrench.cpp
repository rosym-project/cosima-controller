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

#include "../../include/cosima-controller/coordination/monitor_wrench.hpp"

#include <limits>
#include <cstddef>
#include <iostream>

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->monitor_name << "] ")

using namespace cosima;
using namespace coordination;

MonitorWrench::MonitorWrench(const std::string &name, RTT::TaskContext *tc) : MonitorIF(name, tc), 
    bounds_active_for_fx(false),
    bounds_active_for_fy(false),
    bounds_active_for_fz(false),
    bounds_active_for_tx(false),
    bounds_active_for_ty(false),
    bounds_active_for_tz(false),
    outside_bounds_fx(1),
    outside_bounds_fy(1),
    outside_bounds_fz(1),
    outside_bounds_tx(1),
    outside_bounds_ty(1),
    outside_bounds_tz(1),
    lower_bounds_fx(std::numeric_limits<double>::min()), upper_bounds_fx(std::numeric_limits<double>::max()),
    lower_bounds_fy(std::numeric_limits<double>::min()), upper_bounds_fy(std::numeric_limits<double>::max()),
    lower_bounds_fz(std::numeric_limits<double>::min()), upper_bounds_fz(std::numeric_limits<double>::max()),
    lower_bounds_tx(std::numeric_limits<double>::min()), upper_bounds_tx(std::numeric_limits<double>::max()),
    lower_bounds_ty(std::numeric_limits<double>::min()), upper_bounds_ty(std::numeric_limits<double>::max()),
    lower_bounds_tz(std::numeric_limits<double>::min()), upper_bounds_tz(std::numeric_limits<double>::max()),
    for_min_secs(0),
    mode_and(true)
{
    std::string wrench_port_name = "in_monitor_" + this->monitor_name + "_wrench_port";

    if (this->tc->getPort(wrench_port_name))
    {
        this->tc->ports()->removePort(wrench_port_name);
    }

    this->in_wrench_var = geometry_msgs::Wrench();
    this->in_wrench_flow = RTT::NoData;
    this->in_wrench_port.setName(wrench_port_name);
    this->in_wrench_port.doc("Input port for receiving the wrench data for monitor: " + this->monitor_name);
    this->tc->ports()->addPort(this->in_wrench_port);
}

void MonitorWrench::activateBounds(unsigned int id, double lower_bounds, double upper_bounds)
{
    if (id == 0)
    {
        bounds_active_for_fx = true;
        lower_bounds_fx = lower_bounds;
        upper_bounds_fx = upper_bounds;
    }
    else if (id == 1)
    {
        bounds_active_for_fy = true;
        lower_bounds_fy = lower_bounds;
        upper_bounds_fy = upper_bounds;
    }
    else if (id == 2)
    {
        bounds_active_for_fz = true;
        lower_bounds_fz = lower_bounds;
        upper_bounds_fz = upper_bounds;
    }
    else if (id == 3)
    {
        bounds_active_for_tx = true;
        lower_bounds_tx = lower_bounds;
        upper_bounds_tx = upper_bounds;
    }
    else if (id == 4)
    {
        bounds_active_for_ty = true;
        lower_bounds_ty = lower_bounds;
        upper_bounds_ty = upper_bounds;
    }
    else if (id == 5)
    {
        bounds_active_for_tz = true;
        lower_bounds_tz = lower_bounds;
        upper_bounds_tz = upper_bounds;
    }
}

void MonitorWrench::deactivateBounds(unsigned int id)
{
    if (id == 0)
    {
        bounds_active_for_fx = false;
    }
    else if (id == 1)
    {
        bounds_active_for_fy = false;
    }
    else if (id == 2)
    {
        bounds_active_for_fz = false;
    }
    else if (id == 3)
    {
        bounds_active_for_tx = false;
    }
    else if (id == 4)
    {
        bounds_active_for_ty = false;
    }
    else if (id == 5)
    {
        bounds_active_for_tz = false;
    }
}

bool MonitorWrench::eval()
{
    // reset
    this->outside_bounds_fx = 1;
    this->outside_bounds_fy = 1;
    this->outside_bounds_fz = 1;
    this->outside_bounds_tx = 1;
    this->outside_bounds_ty = 1;
    this->outside_bounds_tz = 1;

    this->in_wrench_flow = this->in_wrench_port.read(this->in_wrench_var);
    if (this->in_wrench_flow != RTT::NewData)
    {
        return false;
    }

    int count = 0;

    // mode? AND vs OR

    if (bounds_active_for_fx)
    {
        if ((this->in_wrench_var.force.x < this->lower_bounds_fx) || (this->in_wrench_var.force.x > this->upper_bounds_fx))
        {
            // outside of bounds -> trigger
            this->outside_bounds_fx = 1;
        }
        else
        {
            this->outside_bounds_fx = 0;
        }
        count++;
    }

    if (bounds_active_for_fy)
    {
        if ((this->in_wrench_var.force.y < this->lower_bounds_fy) || (this->in_wrench_var.force.y > this->upper_bounds_fy))
        {
            // outside of bounds -> trigger
            this->outside_bounds_fy = 1;
        }
        else
        {
            this->outside_bounds_fy = 0;
        }
        count++;
    }

    if (bounds_active_for_fz)
    {
        if ((this->in_wrench_var.force.z < this->lower_bounds_fz) || (this->in_wrench_var.force.z > this->upper_bounds_fz))
        {
            // outside of bounds -> trigger
            this->outside_bounds_fz = 1;
        }
        else
        {
            this->outside_bounds_fz = 0;
        }
        count++;
    }


    if (bounds_active_for_tx)
    {
        if ((this->in_wrench_var.torque.x < this->lower_bounds_tx) || (this->in_wrench_var.torque.x > this->upper_bounds_tx))
        {
            // outside of bounds -> trigger
            this->outside_bounds_tx = 1;
        }
        else
        {
            this->outside_bounds_tx = 0;
        }
        count++;
    }

    if (bounds_active_for_ty)
    {
        if ((this->in_wrench_var.torque.y < this->lower_bounds_ty) || (this->in_wrench_var.torque.y > this->upper_bounds_ty))
        {
            // outside of bounds -> trigger
            this->outside_bounds_ty = 1;
        }
        else
        {
            this->outside_bounds_ty = 0;
        }
        count++;
    }

    if (bounds_active_for_tz)
    {
        if ((this->in_wrench_var.torque.z < this->lower_bounds_tz) || (this->in_wrench_var.torque.z > this->upper_bounds_tz))
        {
            // outside of bounds -> trigger
            this->outside_bounds_tz = 1;
        }
        else
        {
            this->outside_bounds_tz = 0;
        }
        count++;
    }

    int sum = this->outside_bounds_fx + this->outside_bounds_fy + this->outside_bounds_fz + this->outside_bounds_tx + this->outside_bounds_ty + this->outside_bounds_tz;

    if (this->mode_and)
    {
        if (sum == 6)
        {
            // trigger
            this->out_event_var = true;
            this->out_event_port.write(true);
        }
    }
    else
    {
        if (sum > (6-count))
        {
            // trigger
            this->out_event_var = true;
            this->out_event_port.write(true);
        }
    }
   
}