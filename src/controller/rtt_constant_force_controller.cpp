/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2017 by Niels Dehio,
 *                       Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
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

#include "../../include/cosima-controller/controller/rtt_constant_force_controller.hpp"

#include <rtt/Component.hpp> // needed for the macro at the end of this file

using namespace cosima;
using namespace controller;

ConstantForceController::ConstantForceController(std::string const &name)
    : RTT::TaskContext(name)
{
    //prepare operations
    addOperation("setDOFsize", &ConstantForceController::setDOFsize, this).doc("set DOF size");
    addOperation("setConstantForce", &ConstantForceController::setConstantForce, this).doc("set force");
    addOperation("setConstantForceVector", &ConstantForceController::setConstantForceVector, this).doc("set force vector");
    addOperation("displayStatus", &ConstantForceController::displayStatus, this).doc("print status");
    addOperation("preparePorts", &ConstantForceController::preparePorts, this).doc("preparePorts");
    addOperation("setTaskSpaceDimension", &ConstantForceController::setTaskSpaceDimension, this, RTT::ClientThread).doc("set TaskSpaceDimension");

    include_compensation = false;
    addProperty("include_compensation", include_compensation);
    include_gravity = true;
    addProperty("include_gravity", include_gravity);
    include_projection = true;
    addProperty("include_projection", include_projection);
    make_orthogonal_projection = false;
    addProperty("make_orthogonal_projection", make_orthogonal_projection);

    //other stuff
    portsArePrepared = false;
    this->setConstantForce(0);
}

bool ConstantForceController::configureHook()
{
    return true;
}

bool ConstantForceController::startHook()
{
    //    if (!in_force_port.connected()) {
    //        RTT::log(RTT::Error) << "in_force_port not connected" << RTT::endlog();
    //        return false;
    //    }
    //    if (!in_direction_port.connected()) {
    //        RTT::log(RTT::Error) << "in_direction_port not connected" << RTT::endlog();
    //        return false;
    //    }
    if (!in_jacobian_port.connected())
    {
        RTT::log(RTT::Error) << "in_jacobian_port not connected" << RTT::endlog();
        return false;
    }
    if (!out_torques_port.connected())
    {
        RTT::log(RTT::Error) << "out_torques_port not connected" << RTT::endlog();
        return false;
    }
    //    if (!out_force_port.connected()) {
    //        RTT::log(RTT::Error) << "out_force_port not connected" << RTT::endlog();
    //        return false;
    //    }
    return true;
}

void ConstantForceController::updateHook()
{
#ifdef TIMING
    RTT::os::TimeService::nsecs start = RTT::os::TimeService::Instance()->getNSecs();
#endif
    //update desired force only when new force-vector received
    if (in_force_port.connected())
    {
        in_force_flow = in_force_port.read(in_force_var);
        if (in_force_flow == RTT::NewData || in_force_flow == RTT::OldData)
        {
            current_force = in_force_var; //TODO: implemented not in a nice way
        }
    }

    if (in_direction_port.connected())
    {
        in_direction_flow = in_direction_port.read(in_direction_var);
    }

    if (in_jacobian_port.connected())
    {
        in_jacobian_flow = in_jacobian_port.read(in_jacobian_var);
    }

    if (in_jacobian_flow == RTT::NoData)
    {
        return;
    }

    in_P_flow = in_P_port.read(in_P_var);
    in_Pdot_flow = in_Pdot_port.read(in_Pdot_var);
    in_M_flow = in_M_port.read(in_M_var);
    in_Mc_flow = in_Mc_port.read(in_Mc_var);
    in_h_flow = in_h_port.read(in_h_var);
    in_tauM_flow = in_tauM_port.read(in_tauM_var);
    in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);

    this->compute(in_direction_var,
                  in_jacobian_var,
                  current_force,
                  out_torques_var,
                  out_force_var);
    out_torques_port.write(out_torques_var);
    out_force_port.write(out_force_var);
#ifdef TIMING
    RTT::os::TimeService::nsecs end = RTT::os::TimeService::Instance()->getNSecs(start);
    RTT::log(RTT::Debug) << 1e-6 * end << " milliseconds [" << this->getName() << "]" << RTT::endlog();
#endif
}

void ConstantForceController::stopHook()
{
}

void ConstantForceController::cleanupHook()
{
    portsArePrepared = false;
}

void ConstantForceController::compute(
    Eigen::VectorXd &in_direction,
    Eigen::MatrixXd &in_jacobian,
    double current_force,
    Eigen::VectorXd &out_torques,
    Eigen::VectorXd &out_force)
{
    // in_direction(0) = 0;
    // in_direction(1) = 0;
    // in_direction(2) = -1;
    // in_direction(3) = 0;
    // in_direction(4) = 0;
    // in_direction(5) = 0;

    // in_direction(6) = 0;
    // in_direction(7) = 0;
    // in_direction(8) = -1;
    // in_direction(9) = 0;
    // in_direction(10) = 0;
    // in_direction(11) = 0;

    Eigen::VectorXd in_robotstatus_vel = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(in_robotstatus_var.velocity.data(), in_robotstatus_var.velocity.size());

    out_force = current_force * in_direction;

    out_torques = in_jacobian.transpose() * out_force;
    if (include_gravity)
    {
        out_torques += in_P_var * in_h_var;
    }

    if (include_compensation)
    {
        // TODO perhaps this was the problem all the time, why was there ZERO? I guess there has to be IDENTITY all the time!
        out_torques = (out_torques + in_h_var + in_M_var * in_Mc_var.inverse() * (in_P_var * in_tauM_var - in_P_var * in_h_var + in_Pdot_var * in_robotstatus_vel));
    }

    if (include_projection)
    {
        if (make_orthogonal_projection)
        {
            out_torques = (Eigen::MatrixXd::Identity(DOFsize, DOFsize) - in_P_var) * out_torques;
        }
        else
        {
            out_torques = in_P_var * out_torques;
        }
    }

    out_force = in_jacobian * out_torques;
}

void ConstantForceController::setDOFsize(unsigned int DOFsize)
{
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
}

void ConstantForceController::setTaskSpaceDimension(const unsigned int TaskSpaceDimension)
{
    assert(TaskSpaceDimension > 0);
    this->TaskSpaceDimension = TaskSpaceDimension;
}

void ConstantForceController::setConstantForce(double new_force)
{
    this->current_force = new_force;
}

void ConstantForceController::setConstantForceVector(const Eigen::VectorXd &new_force)
{
    in_direction_var = new_force;
    this->current_force = 1;
}

void ConstantForceController::preparePorts()
{
    assert(DOFsize > 0);
    assert(TaskSpaceDimension > 0);

    if (portsArePrepared)
    {
        ports()->removePort("in_force_port");
        ports()->removePort("in_direction_port");
        ports()->removePort("in_jacobian_port");
        ports()->removePort("in_P_port");
        ports()->removePort("in_Pdot_port");
        ports()->removePort("in_M_port");
        ports()->removePort("in_Mc_port");
        ports()->removePort("in_h_port");
        ports()->removePort("in_tauM_port");
        ports()->removePort("in_robotstatus_port");
    }

    in_P_var = Eigen::MatrixXd::Zero(DOFsize, DOFsize);
    in_P_port.setName("in_P_port");
    in_P_port.doc("Input port for reading P values");
    ports()->addPort(in_P_port);
    in_P_flow = RTT::NoData;

    in_Pdot_var = Eigen::MatrixXd::Zero(DOFsize, DOFsize);
    in_Pdot_port.setName("in_Pdot_port");
    in_Pdot_port.doc("Input port for reading Pdot values");
    ports()->addPort(in_Pdot_port);
    in_Pdot_flow = RTT::NoData;

    in_M_var = Eigen::MatrixXd::Zero(DOFsize, DOFsize);
    in_M_port.setName("in_M_port");
    in_M_port.doc("Input port for reading M values");
    ports()->addPort(in_M_port);
    in_M_flow = RTT::NoData;

    in_Mc_var = Eigen::MatrixXd::Zero(DOFsize, DOFsize);
    in_Mc_port.setName("in_Mc_port");
    in_Mc_port.doc("Input port for reading Mc values");
    ports()->addPort(in_Mc_port);
    in_Mc_flow = RTT::NoData;

    in_h_var = Eigen::VectorXd::Zero(DOFsize);
    in_h_port.setName("in_h_port");
    in_h_port.doc("Input port for reading GC values");
    ports()->addPort(in_h_port);
    in_h_flow = RTT::NoData;

    in_tauM_var = Eigen::VectorXd(DOFsize);
    in_tauM_port.setName("in_tauM_port");
    in_tauM_port.doc("Input port for reading tauM values");
    ports()->addPort(in_tauM_port);
    in_tauM_flow = RTT::NoData;

    in_robotstatus_var = sensor_msgs::JointState();
    for (unsigned int i = 0; i < DOFsize; i++)
    {
        in_robotstatus_var.position.push_back(0.0);
        in_robotstatus_var.velocity.push_back(0.0);
        in_robotstatus_var.effort.push_back(0.0);
    }
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for reading robotstatus values");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    //prepare input
    in_force_var = double(0);
    in_force_port.setName("in_force_port");
    in_force_port.doc("Input port for reading desired forces");
    ports()->addPort(in_force_port);
    in_force_flow = RTT::NoData;

    in_direction_var = Eigen::VectorXd::Zero(TaskSpaceDimension);
    in_direction_port.setName("in_direction_port");
    in_direction_port.doc("Input port for reading desired forces");
    ports()->addPort(in_direction_port);
    in_direction_flow = RTT::NoData;

    in_jacobian_var = Eigen::MatrixXd::Zero(TaskSpaceDimension, DOFsize);
    in_jacobian_port.setName("in_jacobian_port");
    in_jacobian_port.doc("Input port for reading jacobian_c values");
    ports()->addPort(in_jacobian_port);
    in_jacobian_flow = RTT::NoData;

    //prepare output
    out_torques_var = Eigen::VectorXd(DOFsize);
    out_torques_var.setZero();
    out_torques_port.setName("out_torques_port");
    out_torques_port.doc("Output port for sending torque values");
    out_torques_port.setDataSample(out_torques_var);
    ports()->addPort(out_torques_port);

    out_force_var = Eigen::VectorXd::Zero(TaskSpaceDimension);
    out_force_port.setName("out_force_port");
    out_force_port.doc("Output port for sending force values");
    out_force_port.setDataSample(out_force_var);
    ports()->addPort(out_force_port);

    portsArePrepared = true;
}

void ConstantForceController::displayStatus()
{
    RTT::log(RTT::Error) << "in_force_var \n"
                         << in_force_var << RTT::endlog();
    RTT::log(RTT::Error) << "in_direction_var \n"
                         << in_direction_var << RTT::endlog();
    RTT::log(RTT::Error) << "in_jacobian_var \n"
                         << in_jacobian_var << RTT::endlog();
    RTT::log(RTT::Error) << "out_torques_var \n"
                         << out_torques_var << RTT::endlog();
    RTT::log(RTT::Error) << "out_force_var \n"
                         << out_force_var << RTT::endlog();
    RTT::log(RTT::Error) << "current_force \n"
                         << current_force << RTT::endlog();

    RTT::log(RTT::Error) << "in_P_var \n"
                         << in_P_var << RTT::endlog();
    RTT::log(RTT::Error) << "in_M_var \n"
                         << in_M_var << RTT::endlog();
    RTT::log(RTT::Error) << "in_Mc_var \n"
                         << in_Mc_var << RTT::endlog();
    RTT::log(RTT::Error) << "in_h_var \n"
                         << in_h_var << RTT::endlog();
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::controller::ConstantForceController)
