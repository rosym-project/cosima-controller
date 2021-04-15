/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2017 by Dennis Leroy Wigand <dwigand at cor-lab dot uni-bielefeld dot de>
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
#include "../../include/cosima-controller/introspection/rtt-call-sample-collector.hpp"

#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>
#include <limits>

#include <iostream>

using namespace cosima;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

RTTCallSampleCollector::RTTCallSampleCollector(const std::string &name) : TaskContext(name)
{
	this->addOperation("addCallPort", &RTTCallSampleCollector::addCallPort, this);

	this->out_port_var = cosima_msgs::CallTraceSampleCollection();
}

void RTTCallSampleCollector::addCallPort(const std::string &name, bool is_core_scheduler)
{
	std::shared_ptr<RTTCallSampleCollector::CtsPorts> port_struct = std::shared_ptr<RTTCallSampleCollector::CtsPorts>(new RTTCallSampleCollector::CtsPorts());
	port_struct->port.setName("in_" + name + "_port");
	// 
	port_struct->flow = RTT::NoData;
	// 
	port_struct->data = cosima_msgs::CallTraceSample();
	port_struct->data.call_duration = 0.0;
	port_struct->data.call_time = 0.0;
	port_struct->data.call_type = CALL_UNIVERSAL;
	port_struct->data.callName = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
	port_struct->data.containerName = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
	// 
	port_struct->is_core_scheduler = is_core_scheduler;
	// 
	this->addPort(port_struct->port);
	// 
	this->map_cts_ports[name] = port_struct;


	this->out_port_var.samples.push_back(port_struct->data);
	this->out_port_var.names.push_back(port_struct->data.callName);

	double d = 1.0;
	if (is_core_scheduler)
	{
		d = 1.5;
	}
	port_struct->max_height = d;
	// 
	this->out_port_var.heights.push_back(0.0);
}

bool RTTCallSampleCollector::configureHook()
{
	if (this->map_cts_ports.size() == 0)
	{
		return false;
	}

	this->out_port.setName("out_csc_port");
	this->out_port.setDataSample(this->out_port_var);

	this->addPort(this->out_port);

	return true;
}

void RTTCallSampleCollector::updateHook()
{
	for (auto const& port : this->map_cts_ports)
	{
		// std::cout << port.first  // string (key)
		// 		<< ':' 
		// 		<< port.second // string's value 
		// 		<< std::endl;

		port.second->flow = port.second->port.read(port.second->data);
		if (port.second->flow == RTT::NewData)
		{
			// myfile << ",\n" << cts;

			myfile << "{\"call_name\":\"" << port.second->data.callName << "\""
				   << ",\"container_name\":\"" << port.second->data.containerName << "\""
				   << ",\"call_time\":\"" << port.second->data.call_time << "\""
				   << ",\"call_duration\":\"" << port.second->data.call_duration << "\""
				   << ",\"call_type\":\"" << port.second->data.call_type << "\"}";

			myfile << ",";
			myfile << "\n";

			// if (port.second->data.containerName.compare("updateHook()") == 0)
			// {
			// 	this->out_port_var.samples = port.second->data;
			// }
		}
	}
}

bool RTTCallSampleCollector::startHook()
{
	is_last = false;
	myfile.open(this->getName() + "_" + std::to_string(RTT::os::TimeService::Instance()->getNSecs()) + ".csv");
	myfile << "{\"root\":[\n";
	return true;
}

void RTTCallSampleCollector::stopHook()
{
	myfile << "]}";
	myfile.close();
}

void RTTCallSampleCollector::cleanupHook()
{
	
}

//ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(cosima::RTTCallSampleCollector)
