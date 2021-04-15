/* Author: Niels Dehio, Dennis Leroy Wigand
 * Date:            16.06.2016
 * Latest Update:   14.12.2017
 *
 * Description:
 */

#include "../../include/cosima-controller/transformation/feedback-combiner.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

using namespace cosima;

FeedbackCombiner::FeedbackCombiner(std::string const &name) : cogimon::RTTIntrospectionBase(name)
{
    //prepare operations
    addOperation("setDOFsize", &FeedbackCombiner::setDOFsize, this).doc("set DOF size");
    addOperation("addChainDOFsize", &FeedbackCombiner::addChainDOFsize, this).doc("add Chain DOFsize");
    addOperation("preparePorts", &FeedbackCombiner::preparePorts, this).doc("prepare ports");
    addOperation("displayCurrentState", &FeedbackCombiner::displayCurrentState, this).doc("displayCurrentState");

    //other stuff
    numInputPorts = 0;
    portsArePrepared = false;
}

bool FeedbackCombiner::configureHookInternal()
{
    // intializations and object creations go here. Each component should run this before being able to run

    // //check conncetion
    // for (unsigned int portNr = 0; portNr < numInputPorts; portNr++) {
    //     if (!in_robotstatus_ports[portNr]->connected()) {
    //         RTT::log(RTT::Error) << "in_robotstatus_port_" << portNr << " is not connected" << RTT::endlog();
    //         return false;
    //     }
    // }

    // if (!out_robotstatus_port.connected()) {
    //     RTT::log(RTT::Error) << "out_robotstatus_port is not connected" << std::endl;
    //     return false;
    // }
    // else
    return true;
}

bool FeedbackCombiner::startHookInternal()
{
    // this method starts the component
    unsigned int totalDOF = 0;
    for (unsigned int i = 0; i < numInputPorts; i++)
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
void FeedbackCombiner::updateHookInternal()
{
    bool needToSend = false;
    // auto start = std::chrono::high_resolution_clock::now();

    // this is the actual body of a component. it is called on each cycle
    // out_robotstatus_var.position.setZero();
    // out_robotstatus_var.velocity.setZero();
    // out_robotstatus_var.effort.setZero();
    unsigned int counter = 0;
    for (unsigned int portNr = 0; portNr < numInputPorts; portNr++)
    {
        // if (in_robotstatus_ports[portNr]->connected())
        // {
        // read data and save state of data into "Flow", which can be "NewData", "OldData" or "NoData".
        in_robotstatus_flow[portNr] = in_robotstatus_ports[portNr]->read(in_robotstatus_var[portNr]);
        // }
        // else
        // {
        //     //RTT::log(RTT::Warning) << "[" << this->getName() << "::updateHookInternal] Port in_robotstatus_port_" << portNr << " not connected! Skipping sending data!" << RTT::endlog();
        //     // return;
        //     in_robotstatus_flow[portNr] = RTT::OldData; // TODO DLW HACK!
        // }

        // you can handle cases when there is no new data.
        // if (in_robotstatus_flow[portNr] != RTT::NoData)
        // {
        // if one at least has new data we need to send!
        if (in_robotstatus_flow[portNr] == RTT::NewData)
        {
            needToSend = true;
        }
        for (unsigned int jointID = 0; jointID < in_robotstatus_var[portNr].effort.size(); jointID++)
        {
            out_robotstatus_var.position[counter] += in_robotstatus_var[portNr].position[jointID];
            out_robotstatus_var.velocity[counter] += in_robotstatus_var[portNr].velocity[jointID];
            out_robotstatus_var.effort[counter] += in_robotstatus_var[portNr].effort[jointID];
            counter++;
        }
        // }
        // else if (in_robotstatus_flow[portNr] == RTT::NoData)
        // {
        //     //RTT::log(RTT::Warning) << "[" << this->getName() << "::updateHookInternal] Not getting any data from port in_robotstatus_port_" << portNr << ". Skipping sending data!" << RTT::endlog();
        //     return;
        //     //assert(false); //TODO
        // }
        // else
        // {
        //     // there should be something really wrong!
        // }
    }
    // assert(counter == DOFsize);
    if (needToSend)
    {
        out_robotstatus_port.write(out_robotstatus_var);
    }
}

void FeedbackCombiner::stopHookInternal()
{
    // stops the component (update hookInternal wont be  called anymore)
}

void FeedbackCombiner::cleanupHookInternal()
{
    // cleaning the component data
    this->numChainDOFs.clear();
    portsArePrepared = false;
}

void FeedbackCombiner::setDOFsize(unsigned int DOFsize)
{
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
}

void FeedbackCombiner::addChainDOFsize(unsigned int ChainDOFsize)
{
    assert(ChainDOFsize > 0);
    unsigned int totalDOF = 0;
    for (unsigned int i = 0; i < numInputPorts; i++)
    {
        totalDOF += this->numChainDOFs.at(i);
    }
    if (totalDOF + ChainDOFsize > DOFsize)
    {
        RTT::log(RTT::Error) << "FeedbackCombiner: too many DOF used..." << RTT::endlog();
        assert(false);
    }

    this->numInputPorts++;
    this->numChainDOFs.push_back(ChainDOFsize);
}

void FeedbackCombiner::preparePorts(std::string prefix)
{
    if (portsArePrepared)
    {
        for (unsigned int portNr = 0; portNr < numInputPorts; portNr++)
        {
            ports()->removePort("in_robotstatus" + prefix + "_port_" + boost::lexical_cast<std::string>(portNr));
        }
        ports()->removePort("out_robotstatus" + prefix + "_port");
    }

    //prepare input
    in_robotstatus_var.clear();
    in_robotstatus_flow.clear();
    in_robotstatus_ports.clear();
    for (unsigned int portNr = 0; portNr < numInputPorts; portNr++)
    {
        sensor_msgs::JointState tmp = sensor_msgs::JointState();
        for (unsigned int i = 0; i < numChainDOFs.at(portNr); i++)
        {
            tmp.position.push_back(0.0);
            tmp.velocity.push_back(0.0);
            tmp.effort.push_back(0.0);
        }
        in_robotstatus_var.push_back(tmp);

        boost::shared_ptr<RTT::InputPort<sensor_msgs::JointState>> tmpPort(new RTT::InputPort<sensor_msgs::JointState>());
        tmpPort->setName("in_robotstatus" + prefix + "_port_" + boost::lexical_cast<std::string>(portNr));
        tmpPort->doc("Input port for torque values");
        ports()->addPort(*tmpPort);
        in_robotstatus_flow.push_back(RTT::NoData);
        in_robotstatus_ports.push_back(tmpPort);
    }

    //prepare output
    out_robotstatus_var = sensor_msgs::JointState();
    for (unsigned int i = 0; i < DOFsize; i++)
    {
        out_robotstatus_var.position.push_back(0.0);
        out_robotstatus_var.velocity.push_back(0.0);
        out_robotstatus_var.effort.push_back(0.0);
    }
    out_robotstatus_port.setName("out_robotstatus" + prefix + "_port");
    out_robotstatus_port.doc("Output port for sending cost value");
    out_robotstatus_port.setDataSample(out_robotstatus_var);
    ports()->addPort(out_robotstatus_port);

    portsArePrepared = true;
}

void FeedbackCombiner::displayCurrentState()
{
    // std::cout << "############## FeedbackCombiner State begin " << std::endl;
    // for (unsigned int portNr = 0; portNr < numInputPorts; portNr++)
    // {
    //     std::cout << " in_robotstatus_var[" << portNr << "].position \n"
    //               << in_robotstatus_var[portNr].position << std::endl;
    //     std::cout << " in_robotstatus_var[" << portNr << "].velocity \n"
    //               << in_robotstatus_var[portNr].velocity << std::endl;
    //     std::cout << " in_robotstatus_var[" << portNr << "].effort \n"
    //               << in_robotstatus_var[portNr].effort << std::endl;
    // }
    // std::cout << " out_robotstatus_var.position \n"
    //           << out_robotstatus_var.position << std::endl;
    // std::cout << " out_robotstatus_var.velocity \n"
    //           << out_robotstatus_var.velocity << std::endl;
    // std::cout << " out_robotstatus_var.effort \n"
    //           << out_robotstatus_var.effort << std::endl;
    // std::cout << "############## FeedbackCombiner State end " << std::endl;
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::FeedbackCombiner)
