/* Author: Niels Dehio, Dennis Leroy Wigand
 * Date:            16.06.2016
 * Latest Update:   13.12.2017
 *
 * Description:
 */

#include "../../include/cosima-controller/transformation/torque-command-separator.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

using namespace cosima;

TorqueCommandSeparator::TorqueCommandSeparator(std::string const &name) : cogimon::RTTIntrospectionBase(name), in_torques_flow(RTT::NoData)
{
    //prepare operations
    addOperation("setDOFsize", &TorqueCommandSeparator::setDOFsize, this).doc("set DOF size");
    addOperation("addChainDOFsize", &TorqueCommandSeparator::addChainDOFsize, this).doc("add Chain DOFsize");
    addOperation("preparePorts", &TorqueCommandSeparator::preparePorts, this).doc("prepare ports");
    addOperation("displayCurrentState", &TorqueCommandSeparator::displayCurrentState, this).doc("displayCurrentState");

    //other stuff
    numOutputPorts = 0;
    portsArePrepared = false;
}

bool TorqueCommandSeparator::configureHookInternal()
{
    // intializations and object creations go here. Each component should run this before being able to run

    //check conncetion
    // if (!in_torques_port.connected())
    // {
    //     return false;
    // }

    //    for(unsigned int portNr=0; portNr<numOutputPorts; portNr++){
    //        if (!out_torques_ports[portNr]->connected()){
    //            RTT::log(RTT::Error) << "out_torques_port " << portNr << " is not connected" << RTT::endlog();
    //            return false;
    //        }
    //    }
    return true;
}

bool TorqueCommandSeparator::startHookInternal()
{
    // this method starts the component
    unsigned int totalDOF = 0;
    for (unsigned int i = 0; i < numOutputPorts; i++)
    {
        totalDOF += this->numChainDOFs.at(i);
    }
    if (totalDOF != DOFsize)
    {
        RTT::log(RTT::Error) << "DOFsize is wrong..." << RTT::endlog();
        return false;
    }
    return true;
}
// #include <chrono>
void TorqueCommandSeparator::updateHookInternal()
{
    // auto start = std::chrono::high_resolution_clock::now();
    // this is the actual body of a component. it is called on each cycle
    in_torques_flow = in_torques_port.readNewest(in_torques_var);

    // you can handle cases when there is no new data. RETURN! WE DO NOT WANT TO SEND ZERO DATA!
    // if (in_torques_flow == RTT::NoData)
    // {
    //     return;
    // }

    unsigned int portNr, counter;
    counter = 0;
    for (portNr = 0; portNr < numOutputPorts; portNr++)
    {
        for (unsigned int i = 0; i < numChainDOFs.at(portNr); i++)
        {
            out_torques_var[portNr](i) = in_torques_var(counter + i);
        }
        counter = counter + numChainDOFs.at(portNr);
    }
    assert(counter == DOFsize);

    for (portNr = 0; portNr < numOutputPorts; portNr++)
    {
        out_torques_ports[portNr]->write(out_torques_var[portNr]);
    }
    // auto end = std::chrono::high_resolution_clock::now();
    // // if (in_robotstatus_flow == RTT::NewData) {
    // std::chrono::duration<double, std::milli> fp_ms = end - start;
    // RTT::log(RTT::Debug) << fp_ms.count() << " milliseconds [" << this->getName() << "]" << RTT::endlog();
    // RTT::log(RTT::Info).logflush();
    // }
}

void TorqueCommandSeparator::stopHookInternal()
{
    // stops the component (update hookInternal wont be  called anymore)
}

void TorqueCommandSeparator::cleanupHookInternal()
{
    // cleaning the component data
    this->numChainDOFs.clear();
    portsArePrepared = false;
}

void TorqueCommandSeparator::setDOFsize(unsigned int DOFsize)
{
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
}

void TorqueCommandSeparator::addChainDOFsize(unsigned int ChainDOFsize)
{
    assert(ChainDOFsize > 0);
    unsigned int totalDOF = 0;
    for (unsigned int i = 0; i < numOutputPorts; i++)
    {
        totalDOF += this->numChainDOFs.at(i);
    }
    if (totalDOF + ChainDOFsize > DOFsize)
    {
        RTT::log(RTT::Error) << "TorqueCommandSeparator: too many DOF used..." << RTT::endlog();
        assert(false);
    }

    this->numOutputPorts++;
    this->numChainDOFs.push_back(ChainDOFsize);
}

void TorqueCommandSeparator::preparePorts(std::string prefix)
{
    if (portsArePrepared)
    {
        ports()->removePort("in_torques" + prefix + "_port");
        for (unsigned int portNr = 0; portNr < numOutputPorts; portNr++)
        {
            ports()->removePort("out_torques" + prefix + "_port_" + boost::lexical_cast<std::string>(portNr));
        }
    }

    //prepare input
    in_torques_var = Eigen::VectorXd(DOFsize);
    in_torques_port.setName("in_torques" + prefix + "_port");
    in_torques_port.doc("Input port for reading torques values");
    ports()->addPort(in_torques_port);
    in_torques_flow = RTT::NoData;

    //prepare output
    out_torques_var.clear();
    out_torques_ports.clear();
    for (unsigned int portNr = 0; portNr < numOutputPorts; portNr++)
    {
        out_torques_var.push_back(Eigen::VectorXd(numChainDOFs.at(portNr)));
        out_torques_var[portNr].setZero();

        boost::shared_ptr<RTT::OutputPort<Eigen::VectorXd>> tmpPort(new RTT::OutputPort<Eigen::VectorXd>());
        tmpPort->setName("out_torques" + prefix + "_port_" + boost::lexical_cast<std::string>(portNr));
        tmpPort->doc("Output port for sending torques values");
        tmpPort->setDataSample(out_torques_var[portNr]);
        ports()->addPort(*tmpPort);
        out_torques_ports.push_back(tmpPort);
    }

    portsArePrepared = true;
}

void TorqueCommandSeparator::displayCurrentState()
{
    // std::cout << "############## TorqueCommandSeparator State begin " << std::endl;
    // std::cout << " in_torques_var \n"
    //           << in_torques_var << std::endl;
    // for (unsigned int portNr = 0; portNr < numOutputPorts; portNr++)
    // {
    //     std::cout << " out_torques_var[" << portNr << "] \n"
    //               << out_torques_var[portNr] << std::endl;
    // }
    // std::cout << "############## TorqueCommandSeparator State end " << std::endl;
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::TorqueCommandSeparator)
