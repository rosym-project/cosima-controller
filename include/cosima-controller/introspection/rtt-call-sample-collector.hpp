/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2021 by Dennis Leroy Wigand <dwigand at cor-lab dot uni-bielefeld dot de>
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
 *   European Community’s Horizon 2020 robotics program ICT-23-2014
 *     under grant agreement 644727 - CogIMon
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */
#pragma once

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>

#include <Eigen/Dense>

#include <vector>

#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Logger.hpp>

#include <boost/shared_ptr.hpp>

#include <thread>
#include <memory>
#include <map>

#include <cosima_msgs/CallTraceSample.h>
#include <cosima_msgs/CallTraceSampleCollection.h>

#include <rtt/os/TimeService.hpp>

#include <fstream>
#include <streambuf>
#include <limits>

#include <iostream>

namespace cosima
{

class RTTCallSampleCollector : public RTT::TaskContext
{
  public:
	RTTCallSampleCollector(std::string const &name);

	enum CTSenum {CALL_UNIVERSAL, CALL_START, CALL_END, CALL_INSTANTANEOUS, CALL_PORT_WRITE, CALL_PORT_READ_NODATA, CALL_PORT_READ_NEWDATA, CALL_PORT_READ_OLDDATA, CALL_START_WITH_DURATION};

	struct CtsPorts
    {
        RTT::InputPort<cosima_msgs::CallTraceSample> port;
		RTT::FlowStatus flow;
		cosima_msgs::CallTraceSample data;
		bool is_core_scheduler;
		double max_height;
    };

	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	void addCallPort(const std::string &name, bool is_core_scheduler = false);

  private:
	std::map<std::string, std::shared_ptr<CtsPorts>> map_cts_ports;

	RTT::OutputPort<cosima_msgs::CallTraceSampleCollection> out_port;
	cosima_msgs::CallTraceSampleCollection out_port_var;

	std::ofstream myfile;

	bool is_last;
};

} // namespace cosima