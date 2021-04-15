/* Author: Niels Dehio
 * Date:   16 June 2016
 *
 * Description: 
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/lexical_cast.hpp>

// header for introspection
#include "../introspection/rtt-introspection-base.hpp"

namespace cosima
{

class TorqueCommandSeparator : public cogimon::RTTIntrospectionBase
{
public:
  TorqueCommandSeparator(std::string const &name);

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
  RTT::InputPort<Eigen::VectorXd> in_torques_port;

  // Declare output ports and their datatypes
  std::vector<boost::shared_ptr<RTT::OutputPort<Eigen::VectorXd>>> out_torques_ports;

  // Data flow:
  RTT::FlowStatus in_torques_flow;

  // variables
  Eigen::VectorXd in_torques_var;
  std::vector<Eigen::VectorXd> out_torques_var;
  unsigned int DOFsize;
  unsigned int numOutputPorts;
  std::vector<unsigned int> numChainDOFs;
  bool portsArePrepared;
};

} // namespace cosima