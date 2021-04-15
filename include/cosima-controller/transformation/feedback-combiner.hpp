/* Author: Niels Dehio
 * Date:   16 June 2016
 *
 * Description: 
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <sensor_msgs/JointState.h>
#include <boost/lexical_cast.hpp>

// header for introspection
#include "../introspection/rtt-introspection-base.hpp"

namespace cosima
{

class FeedbackCombiner : public cogimon::RTTIntrospectionBase
{
  public:
    FeedbackCombiner(std::string const &name);

    ///////////////////////////////////////////
    // Internal mirrors of the default Orocos
    // life cycle from the introspection base.
    ///////////////////////////////////////////
    bool configureHookInternal();
    bool startHookInternal();
    void updateHookInternal();
    void stopHookInternal();
    void cleanupHookInternal();
    ///////////////////////////////////////////

    void setDOFsize(unsigned int DOFsize);
    void addChainDOFsize(unsigned int ChainDOFsize);
    void preparePorts(std::string prefix);
    void displayCurrentState();

  private:
    // Declare input ports and their datatypes
    std::vector<boost::shared_ptr<RTT::InputPort<sensor_msgs::JointState>>> in_robotstatus_ports;

    // Declare output ports and their datatypes
    RTT::OutputPort<sensor_msgs::JointState> out_robotstatus_port;

    // Data flow:
    std::vector<RTT::FlowStatus> in_robotstatus_flow;

    // variables
    std::vector<sensor_msgs::JointState> in_robotstatus_var;
    sensor_msgs::JointState out_robotstatus_var;
    unsigned int DOFsize;
    unsigned int numInputPorts;
    std::vector<unsigned int> numChainDOFs;
    bool portsArePrepared;
};

} // namespace cosima