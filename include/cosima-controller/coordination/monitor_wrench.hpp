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

#pragma once

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <string>

#include <Eigen/Dense>
#include <iostream>

#include "monitor_if.hpp"

#include <geometry_msgs/Wrench.h>

namespace cosima
{

  namespace coordination
  {

    class MonitorWrench : public MonitorIF
    {
    public:
      MonitorWrench(const std::string &name, RTT::TaskContext *tc);

      bool eval();

      void activateBounds(unsigned int id, double lower_bounds, double upper_bounds);
      void deactivateBounds(unsigned int id);

    protected:
      RTT::InputPort<geometry_msgs::Wrench> in_wrench_port;
      geometry_msgs::Wrench in_wrench_var;
      RTT::FlowStatus in_wrench_flow;

      double lower_bounds_fx, upper_bounds_fx;
      bool bounds_active_for_fx;
      int outside_bounds_fx;
      double lower_bounds_fy, upper_bounds_fy;
      bool bounds_active_for_fy;
      int outside_bounds_fy;
      double lower_bounds_fz, upper_bounds_fz;
      bool bounds_active_for_fz;
      int outside_bounds_fz;
      double lower_bounds_tx, upper_bounds_tx;
      bool bounds_active_for_tx;
      int outside_bounds_tx;
      double lower_bounds_ty, upper_bounds_ty;
      bool bounds_active_for_ty;
      int outside_bounds_ty;
      double lower_bounds_tz, upper_bounds_tz;
      bool bounds_active_for_tz;
      int outside_bounds_tz;

      double for_min_secs;

      // and vs or mode
      bool mode_and;
    };

  } // namespace coordination

} // namespace cosima