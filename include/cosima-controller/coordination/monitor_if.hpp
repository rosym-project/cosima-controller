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

namespace cosima
{

  namespace coordination
  {
    // template <typename T>
    class MonitorIF
    {
    public:
      MonitorIF(const std::string &name, RTT::TaskContext *tc);

      virtual bool eval() = 0;

    protected:
      RTT::TaskContext *tc;
      std::string monitor_name;
      std::string port_name;

      RTT::OutputPort<bool> out_event_port;
      bool out_event_var;
    };

  } // namespace coordination

} // namespace cosima