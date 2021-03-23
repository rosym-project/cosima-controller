/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2020 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>,
 *                       Niels Dehio
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
#include "../../include/cosima-controller/kinematics_dynamics/rtt_kin_dyn_multi_arm.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

using namespace cosima;

RTTKinDynMultiArm::RTTKinDynMultiArm(std::string const &name) : RTT::TaskContext(name), portsArePrepared(false)
{
    addOperation("addChain", &RTTKinDynMultiArm::addChain, this).doc("add chain");
    addOperation("addChainWithCompliance", &RTTKinDynMultiArm::addChainWithCompliance, this).doc("add chain with compliance");
    // addOperation("addChainWithWorldOffset_to", &RTTKinDynMultiArm::addChainWithWorldOffset_to, this).doc("add chain");
    addOperation("addChainWithWorldOffset", &RTTKinDynMultiArm::addChainWithWorldOffset, this).doc("add chain");

    solver_manager = KinDynMultiArm();

    addProperty("ext_override", ext_override);
    this->ext_override = -1;
}

/**
 * This needs to be called before the component is configured!
 */
void RTTKinDynMultiArm::setComplianceFrame(const unsigned int &index, const geometry_msgs::Pose &offset)
{
    this->solver_manager.setComplianceFrame(index, offset);
}

bool RTTKinDynMultiArm::configureHook()
{
    this->preparePorts();
    return true;
}

bool RTTKinDynMultiArm::startHook()
{
    // if (!in_robotstatus_port.connected())
    // {
    //     RTT::log(RTT::Error) << "in_robotstatus_port not connected, so I won't start!" << RTT::endlog();
    //     return false;
    // }

    this->num_robots = this->solver_manager.getNumberOfRobots();

    return true;
}

void RTTKinDynMultiArm::updateHook()
{
    // read all the information that we need for the computations
    bool override = true;

    bool no_data_received = false;
    for (unsigned int arms = 0; arms < this->num_robots; arms++)
    {
        unsigned int e_index_begin = 0;
        if (arms > 0)
        {
            e_index_begin = this->solver_manager.getJointDofs(arms - 1);
        }
        unsigned int e_index_end = this->solver_manager.getJointDofs(arms);

        // Take care of externally provided gravity
        if (in_external_gravity_ports[arms]->connected())
        {
            in_external_gravity_flows[arms] = in_external_gravity_ports[arms]->read(in_external_gravity_vars[arms]);
            in_external_gravity_var_stacked.segment(e_index_begin, e_index_end) = in_external_gravity_vars[arms];
        }

        // Take care of externally provided inertia
        if (in_inertia_ports[arms]->connected())
        {
            in_inertia_flows[arms] = in_inertia_ports[arms]->read(in_inertia_vars[arms]);
            // RTT::log(RTT::Error) << "in_inertia_vars[arms] =\n" << in_inertia_vars[arms] << RTT::endlog();
            in_inertia_var_stacked.block(e_index_begin, e_index_begin, e_index_end, e_index_end) = in_inertia_vars[arms];
        }
        // RTT::log(RTT::Error) << "in_inertia_var_stacked =\n" << in_inertia_var_stacked << RTT::endlog();

        // Take care of externally provided torque
        if (in_external_torque_ports[arms]->connected())
        {
            in_external_torque_vars[arms].setZero(); // TODO good like this?
            in_external_torque_flows[arms] = in_external_torque_ports[arms]->read(in_external_torque_vars[arms]);
            in_external_torque_var_stacked.segment(e_index_begin, e_index_end) = in_external_torque_vars[arms];
        }

        // Take care of externally provided robot state
        in_robotstatus_flows[arms] = in_robotstatus_ports[arms]->read(in_robotstatus_vars[arms]);
        sensor_msgs::JointState _tmp_j_state = in_robotstatus_vars[arms];

        for (unsigned int entry_index = 0; entry_index < e_index_end; entry_index++)
        {
            out_robotstatus_var.position[entry_index + e_index_begin] = _tmp_j_state.position[entry_index];
            out_robotstatus_var.velocity[entry_index + e_index_begin] = _tmp_j_state.velocity[entry_index];
            out_robotstatus_var.effort[entry_index + e_index_begin] = _tmp_j_state.effort[entry_index];
        }

        // for (unsigned int lll = 0; lll < out_robotstatus_var.position.size(); lll++)
        // {
        //     RTT::log(RTT::Error) << "out_robotstatus_var.position[" << lll << "] = " << out_robotstatus_var.position[lll] << RTT::endlog();
        // }

        // for (unsigned int lll = 0; lll < _tmp_j_state.position.size(); lll++)
        // {
        //     RTT::log(RTT::Error) << "_tmp_j_state.position[" << lll << "] = " << _tmp_j_state.position[lll] << RTT::endlog();
        // }

        if ((in_external_gravity_flows[arms] == RTT::NoData) || (in_inertia_flows[arms] == RTT::NoData))
        {
            override = false;
        }

        if (in_robotstatus_flows[arms] == RTT::NoData)
        {
            no_data_received = true;
            // break;
        }
    }

    // you can handle cases when there is no new data.
    // if (in_robotstatus_flow != RTT::NoData)
    if (!no_data_received)
    {
        bool tmp_override = false;
        if (this->ext_override == 1)
        {
            tmp_override = true;
        }
        else if (this->ext_override == 0)
        {
            tmp_override = false;
        }
        else
        {
            tmp_override = override;
        }
        
        this->solver_manager.computeAllAsStack(
            out_robotstatus_var,
            out_inertia_var,
            out_inertiaInv_var,
            out_gravity_var,
            out_coriolis_var,
            out_coriolisAndGravity_var,
            out_cartPos_var,
            out_cartVel_var,
            out_cartAcc_var,
            out_jacobian_var,
            out_jacobianDot_var,
            in_external_gravity_var_stacked,
            out_external_wrench_var,
            in_external_gravity_var_stacked,
            in_inertia_var_stacked,
            tmp_override);

        // RTT::log(RTT::Error) << "out_inertia_var =\n" << out_inertia_var << RTT::endlog();
    }
    else // if (in_robotstatus_flow == RTT::NoData)
    {
        PRELOG(Warning) << ": in_robotstatus_flow == RTT::NoData. If this only happens at the beginning before the robot is sending feedback it's fine!" << RTT::endlog();
        for (unsigned int i = 0; i < this->out_robotstatus_var.position.size(); i++)
        {
            out_robotstatus_var.position[i] = 0.0;
            out_robotstatus_var.velocity[i] = 0.0;
            out_robotstatus_var.effort[i] = 0.0;
        }
        out_inertia_var.setZero();
        out_inertiaInv_var.setZero();
        out_gravity_var.setZero();
        out_coriolis_var.setZero();
        out_coriolisAndGravity_var.setZero();
        out_cartPos_var.setZero();
        out_cartVel_var.setZero();
        out_cartAcc_var.setZero();
        out_jacobian_var.setZero();
        out_jacobianDot_var.setZero();
        out_external_wrench_var.setZero();
    }

    // PRELOG(Error) << "DEBUG: KIN 2 out_coriolisAndGravity_var = " << out_coriolisAndGravity_var << RTT::endlog();

    // Write the data through the ports!

    // for (unsigned int lll = 0; lll < out_robotstatus_var.position.size(); lll++)
    // {
    //     RTT::log(RTT::Error) << "out_robotstatus_var.position[" << lll << "] = " << out_robotstatus_var.position[lll] << RTT::endlog();
    // }

    // separate cart pos and vel for publishing
    for (unsigned int arm = 0; arm < this->num_robots; arm++)
    {
        out_cartPos_vars[arm].position.x = out_cartPos_var((arm * 7) + 0);
        out_cartPos_vars[arm].position.y = out_cartPos_var((arm * 7) + 1);
        out_cartPos_vars[arm].position.z = out_cartPos_var((arm * 7) + 2);
        out_cartPos_vars[arm].orientation.w = out_cartPos_var((arm * 7) + 3);
        out_cartPos_vars[arm].orientation.x = out_cartPos_var((arm * 7) + 4);
        out_cartPos_vars[arm].orientation.y = out_cartPos_var((arm * 7) + 5);
        out_cartPos_vars[arm].orientation.z = out_cartPos_var((arm * 7) + 6);

        out_cartVel_vars[arm].linear.x = out_cartVel_var((arm * 6) + 0);
        out_cartVel_vars[arm].linear.x = out_cartVel_var((arm * 6) + 1);
        out_cartVel_vars[arm].linear.x = out_cartVel_var((arm * 6) + 2);
        out_cartVel_vars[arm].angular.x = out_cartVel_var((arm * 6) + 3);
        out_cartVel_vars[arm].angular.y = out_cartVel_var((arm * 6) + 4);
        out_cartVel_vars[arm].angular.z = out_cartVel_var((arm * 6) + 5);
    }

    out_robotstatus_port.write(out_robotstatus_var);
    out_inertia_port.write(out_inertia_var);
    out_inertiaInv_port.write(out_inertiaInv_var);
    out_gravity_port.write(out_gravity_var);
    out_coriolis_port.write(out_coriolis_var);
    out_coriolisAndGravity_port.write(out_coriolisAndGravity_var);
    out_cartPos_port.write(out_cartPos_var);
    out_cartVel_port.write(out_cartVel_var);
    out_cartAcc_port.write(out_cartAcc_var);
    out_jacobian_port.write(out_jacobian_var);
    out_jacobianDot_port.write(out_jacobianDot_var);
    out_external_wrench_port.write(out_external_wrench_var);

    for (unsigned int arm = 0; arm < this->num_robots; arm++)
    {
        out_cartPos_ports[arm]->write(out_cartPos_vars[arm]);
        out_cartVel_ports[arm]->write(out_cartVel_vars[arm]);
        out_external_wrench_ports[arm]->write(out_external_wrench_vars[arm]);
    }
}

void RTTKinDynMultiArm::stopHook()
{
}

void RTTKinDynMultiArm::cleanupHook()
{
    // TODO proper clean up of vars and also of vectors.
    this->portsArePrepared = false;
}

bool RTTKinDynMultiArm::addChain(const std::string &solver_type, const std::string &modelname, const std::string &chain_root_link_name, const std::string &chain_tip_link_name)
{
    return solver_manager.addRobotChain(solver_type, modelname, chain_root_link_name, chain_tip_link_name);
}

bool RTTKinDynMultiArm::addChainWithCompliance(const std::string &solver_type, const std::string &modelname, const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const geometry_msgs::Pose &compliance_frame)
{
    geometry_msgs::Pose _worldOffset;
    _worldOffset.position.x = 0;
    _worldOffset.position.y = 0;
    _worldOffset.position.z = 0;

    _worldOffset.orientation.w = 1;
    _worldOffset.orientation.x = 0;
    _worldOffset.orientation.y = 0;
    _worldOffset.orientation.z = 0;
    return solver_manager.addRobotChainWithWorldOffset(solver_type, modelname, chain_root_link_name, chain_tip_link_name, _worldOffset, compliance_frame);
}

bool RTTKinDynMultiArm::addChainWithWorldOffset_to(const std::string &solver_type, const std::string &modelname, const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const Eigen::VectorXd &worldOffsetTranslation, const Eigen::VectorXd &worldOffsetRotation)
{
    return solver_manager.addRobotChainWithWorldOffset(solver_type, modelname, chain_root_link_name, chain_tip_link_name, worldOffsetTranslation, worldOffsetRotation);
}

bool RTTKinDynMultiArm::addChainWithWorldOffset(const std::string &solver_type, const std::string &modelname, const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const geometry_msgs::Pose &worldOffset)
{
    geometry_msgs::Pose _cf;
    _cf.position.x = 0;
    _cf.position.y = 0;
    _cf.position.z = 0;

    _cf.orientation.w = 1;
    _cf.orientation.x = 0;
    _cf.orientation.y = 0;
    _cf.orientation.z = 0;
    return solver_manager.addRobotChainWithWorldOffset(solver_type, modelname, chain_root_link_name, chain_tip_link_name, worldOffset, _cf);
}

void RTTKinDynMultiArm::preparePorts()
{
    if (this->portsArePrepared)
    {
        ports()->removePort("in_robotstatus_port");
        ports()->removePort("out_robotstatus_port");
        ports()->removePort("out_inertia_port");
        ports()->removePort("out_inertiaInv_port");
        ports()->removePort("out_gravity_port");
        ports()->removePort("out_coriolis_port");
        ports()->removePort("out_coriolisAndGravity_port");
        ports()->removePort("out_cartPos_port");
        ports()->removePort("out_cartVel_port");
        ports()->removePort("out_cartAcc_port");
        ports()->removePort("out_jacobian_port");
        ports()->removePort("out_jacobianDot_port");
        ports()->removePort("out_external_wrench_port");

        for (int i = 0; i < this->solver_manager.getNumberOfRobots(); i++)
        {
            ports()->removePort("out_cartPos_" + std::to_string(i) + "_port");
            ports()->removePort("out_cartVel_" + std::to_string(i) + "_port");
            ports()->removePort("in_external_gravity_" + std::to_string(i) + "_port");
            ports()->removePort("in_external_torque_" + std::to_string(i) + "_port");
            ports()->removePort("out_external_wrench_" + std::to_string(i) + "_port");
        }
    }

    // prepare input
    out_robotstatus_var = sensor_msgs::JointState();
    unsigned int _total_dofs = this->solver_manager.getTotalNumberJointDofs();
    for (unsigned int i = 0; i < _total_dofs; i++)
    {
        out_robotstatus_var.position.push_back(0.0);
        out_robotstatus_var.velocity.push_back(0.0);
        out_robotstatus_var.effort.push_back(0.0);
    }

    in_external_gravity_var_stacked = Eigen::VectorXd::Zero(_total_dofs);
    in_inertia_var_stacked = Eigen::MatrixXd::Zero(_total_dofs, _total_dofs);
    in_external_torque_var_stacked = Eigen::VectorXd::Zero(_total_dofs);

    // prepare output
    out_robotstatus_port.setName("out_robotstatus_port");
    out_robotstatus_port.doc("Output port for robotstatus values");
    out_robotstatus_port.setDataSample(out_robotstatus_var);
    ports()->addPort(out_robotstatus_port);

    out_inertia_var = Eigen::MatrixXd::Zero(_total_dofs, _total_dofs);
    out_inertia_var.setZero();
    out_inertia_port.setName("out_inertia_port");
    out_inertia_port.doc("Output port for inertia matrix");
    out_inertia_port.setDataSample(out_inertia_var);
    ports()->addPort(out_inertia_port);

    out_inertiaInv_var = Eigen::MatrixXd::Zero(_total_dofs, _total_dofs);
    out_inertiaInv_var.setZero();
    out_inertiaInv_port.setName("out_inertiaInv_port");
    out_inertiaInv_port.doc("Output port for inertia matrix");
    out_inertiaInv_port.setDataSample(out_inertiaInv_var);
    ports()->addPort(out_inertiaInv_port);

    out_gravity_var = Eigen::VectorXd::Zero(_total_dofs);
    out_gravity_var.setZero();
    out_gravity_port.setName("out_gravity_port");
    out_gravity_port.doc("Output port for gravity vector");
    out_gravity_port.setDataSample(out_gravity_var);
    ports()->addPort(out_gravity_port);

    out_coriolis_var = Eigen::VectorXd::Zero(_total_dofs);
    out_coriolis_var.setZero();
    out_coriolis_port.setName("out_coriolis_port");
    out_coriolis_port.doc("Output port for coriolis vector");
    out_coriolis_port.setDataSample(out_coriolis_var);
    ports()->addPort(out_coriolis_port);

    out_coriolisAndGravity_var = Eigen::VectorXd::Zero(_total_dofs);
    out_coriolisAndGravity_var.setZero();
    out_coriolisAndGravity_port.setName("out_coriolisAndGravity_port");
    out_coriolisAndGravity_port.doc("Output port for combined coriolis and gravity vector");
    out_coriolisAndGravity_port.setDataSample(out_coriolisAndGravity_var);
    ports()->addPort(out_coriolisAndGravity_port);

    out_cartPos_var = Eigen::VectorXd::Zero(7 * this->solver_manager.getNumberOfRobots());
    out_cartPos_var.setZero();
    out_cartPos_port.setName("out_cartPos_port");
    out_cartPos_port.doc("Output port for cartesian position vector");
    out_cartPos_port.setDataSample(out_cartPos_var);
    ports()->addPort(out_cartPos_port);

    out_cartVel_var = Eigen::VectorXd::Zero(6 * this->solver_manager.getNumberOfRobots());
    out_cartVel_var.setZero();
    out_cartVel_port.setName("out_cartVel_port");
    out_cartVel_port.doc("Output port for cartesian velocity vector");
    out_cartVel_port.setDataSample(out_cartVel_var);
    ports()->addPort(out_cartVel_port);

    out_cartAcc_var = Eigen::VectorXd::Zero(6 * this->solver_manager.getNumberOfRobots());
    out_cartAcc_var.setZero();
    out_cartAcc_port.setName("out_cartAcc_port");
    out_cartAcc_port.doc("Output port for cartesian acceleration vector");
    out_cartAcc_port.setDataSample(out_cartAcc_var);
    ports()->addPort(out_cartAcc_port);

    out_jacobian_var = Eigen::MatrixXd::Zero(6 * this->solver_manager.getNumberOfRobots(), _total_dofs);
    out_jacobian_var.setZero();
    out_jacobian_port.setName("out_jacobian_port");
    out_jacobian_port.doc("Output port for jacobian matrix");
    out_jacobian_port.setDataSample(out_jacobian_var);
    ports()->addPort(out_jacobian_port);

    out_jacobianDot_var = Eigen::MatrixXd::Zero(6 * this->solver_manager.getNumberOfRobots(), _total_dofs);
    out_jacobianDot_var.setZero();
    out_jacobianDot_port.setName("out_jacobianDot_port");
    out_jacobianDot_port.doc("Output port for jacobianDot matrix");
    out_jacobianDot_port.setDataSample(out_jacobianDot_var);
    ports()->addPort(out_jacobianDot_port);

    out_external_wrench_var = Eigen::VectorXd::Zero(6 * this->solver_manager.getNumberOfRobots());
    out_external_wrench_var.setZero();
    out_external_wrench_port.setName("out_external_wrench_port");
    out_external_wrench_port.doc("Output port for combined external wrench");
    out_external_wrench_port.setDataSample(out_external_wrench_var);
    ports()->addPort(out_external_wrench_port);

    for (int i = 0; i < this->solver_manager.getNumberOfRobots(); i++)
    {
        sensor_msgs::JointState _tmp_robotstatus;
        for (int jjj = 0; jjj < this->solver_manager.getJointDofs(i); jjj++)
        {
            _tmp_robotstatus.position.push_back(0.0);
            _tmp_robotstatus.velocity.push_back(0.0);
            _tmp_robotstatus.effort.push_back(0.0);
        }
        in_robotstatus_vars.push_back(_tmp_robotstatus);

        in_robotstatus_ports.push_back(std::unique_ptr<RTT::InputPort<sensor_msgs::JointState>>(new RTT::InputPort<sensor_msgs::JointState>));
        in_robotstatus_flows.push_back(RTT::NoData);
        in_robotstatus_ports.at(i)->setName("in_robotstatus_" + std::to_string(i) + "_port");
        in_robotstatus_ports.at(i)->doc("Input port for receiving the robot feedback.");
        ports()->addPort(*(in_robotstatus_ports.at(i).get()));

        geometry_msgs::Pose _tmp_pose;
        _tmp_pose.position.x = 0.0;
        _tmp_pose.position.y = 0.0;
        _tmp_pose.position.z = 0.0;
        _tmp_pose.orientation.w = 0.0;
        _tmp_pose.orientation.x = 0.0;
        _tmp_pose.orientation.y = 0.0;
        _tmp_pose.orientation.z = 0.0;
        out_cartPos_vars.push_back(_tmp_pose);

        geometry_msgs::Twist _tmp_twist;
        _tmp_twist.linear.x = 0.0;
        _tmp_twist.linear.y = 0.0;
        _tmp_twist.linear.z = 0.0;
        _tmp_twist.angular.x = 0.0;
        _tmp_twist.angular.y = 0.0;
        _tmp_twist.angular.z = 0.0;
        out_cartVel_vars.push_back(_tmp_twist);

        out_cartPos_ports.push_back(std::unique_ptr<RTT::OutputPort<geometry_msgs::Pose>>(new RTT::OutputPort<geometry_msgs::Pose>()));
        out_cartPos_ports.at(i)->setName("out_cartPos_" + std::to_string(i) + "_port");
        out_cartPos_ports.at(i)->doc("Output port for sending the cartesian end-effector position");
        out_cartPos_ports.at(i)->setDataSample(_tmp_pose);
        ports()->addPort(*(out_cartPos_ports.at(i).get()));

        out_cartVel_ports.push_back(std::unique_ptr<RTT::OutputPort<geometry_msgs::Twist>>(new RTT::OutputPort<geometry_msgs::Twist>));
        out_cartVel_ports.at(i)->setName("out_cartVel_" + std::to_string(i) + "_port");
        out_cartVel_ports.at(i)->doc("Output port for sending the cartesian end effector velocity");
        out_cartVel_ports.at(i)->setDataSample(_tmp_twist);
        ports()->addPort(*(out_cartVel_ports.at(i).get()));

        // TODO include input below

        in_external_gravity_ports.push_back(std::unique_ptr<RTT::InputPort<Eigen::VectorXd>>(new RTT::InputPort<Eigen::VectorXd>));
        in_external_gravity_flows.push_back(RTT::NoData);
        in_external_gravity_vars.push_back(Eigen::VectorXd::Zero(_total_dofs / this->solver_manager.getNumberOfRobots())); // TODO make this variable for the added chain.
        in_external_gravity_ports.at(i)->setName("in_external_gravity_" + std::to_string(i) + "_port");
        in_external_gravity_ports.at(i)->doc("Input port for receiving an external G vector.");
        ports()->addPort(*(in_external_gravity_ports.at(i).get()));

        in_inertia_ports.push_back(std::unique_ptr<RTT::InputPort<Eigen::MatrixXd>>(new RTT::InputPort<Eigen::MatrixXd>));
        in_inertia_flows.push_back(RTT::NoData);
        in_inertia_vars.push_back(Eigen::MatrixXd::Zero(_total_dofs / this->solver_manager.getNumberOfRobots(), _total_dofs / this->solver_manager.getNumberOfRobots())); // TODO make this variable for the added chain.
        in_inertia_ports.at(i)->setName("in_inertia_" + std::to_string(i) + "_port");
        in_inertia_ports.at(i)->doc("Input port for receiving an external inertia matrix.");
        ports()->addPort(*(in_inertia_ports.at(i).get()));

        in_external_torque_ports.push_back(std::unique_ptr<RTT::InputPort<Eigen::VectorXd>>(new RTT::InputPort<Eigen::VectorXd>));
        in_external_torque_flows.push_back(RTT::NoData);
        in_external_torque_vars.push_back(Eigen::VectorXd::Zero(_total_dofs / this->solver_manager.getNumberOfRobots())); // TODO make this variable for the added chain.
        in_external_torque_ports.at(i)->setName("in_external_torque_" + std::to_string(i) + "_port");
        in_external_torque_ports.at(i)->doc("Input port for receiving an external torque vector.");
        ports()->addPort(*(in_external_torque_ports.at(i).get()));

        geometry_msgs::Wrench _tmp_wrench;
        _tmp_wrench.force.x = 0.0;
        _tmp_wrench.force.y = 0.0;
        _tmp_wrench.force.z = 0.0;
        _tmp_wrench.torque.x = 0.0;
        _tmp_wrench.torque.y = 0.0;
        _tmp_wrench.torque.z = 0.0;
        out_external_wrench_vars.push_back(_tmp_wrench);

        out_external_wrench_ports.push_back(std::unique_ptr<RTT::OutputPort<geometry_msgs::Wrench>>(new RTT::OutputPort<geometry_msgs::Wrench>));
        out_external_wrench_ports.at(i)->setName("out_external_wrench_" + std::to_string(i) + "_port");
        out_external_wrench_ports.at(i)->doc("Output port for sending the external wrench");
        out_external_wrench_ports.at(i)->setDataSample(_tmp_wrench);
        ports()->addPort(*(out_external_wrench_ports.at(i).get()));
    }

    this->portsArePrepared = true;

    // PRELOG(Error) << "[RTT] out_inertia_var(" << out_inertia_var.rows() << ", " << out_inertia_var.cols() << ")" << RTT::endlog();
    // PRELOG(Error) << "[RTT] out_inertiaInv_var(" << out_inertiaInv_var.rows() << ", " << out_inertiaInv_var.cols() << ")" << RTT::endlog();
    // PRELOG(Error) << "[RTT] out_gravity_var(" << out_gravity_var.rows() << ", " << out_gravity_var.cols() << ")" << RTT::endlog();
    // PRELOG(Error) << "[RTT] out_coriolis_var(" << out_coriolis_var.rows() << ", " << out_coriolis_var.cols() << ")" << RTT::endlog();
    // PRELOG(Error) << "[RTT] out_coriolisAndGravity_var(" << out_coriolisAndGravity_var.rows() << ", " << out_coriolisAndGravity_var.cols() << ")" << RTT::endlog();
    // PRELOG(Error) << "[RTT] out_cartPos_var(" << out_cartPos_var.rows() << ", " << out_cartPos_var.cols() << ")" << RTT::endlog();
    // PRELOG(Error) << "[RTT] out_cartVel_var(" << out_cartVel_var.rows() << ", " << out_cartVel_var.cols() << ")" << RTT::endlog();
    // PRELOG(Error) << "[RTT] out_cartAcc_var(" << out_cartAcc_var.rows() << ", " << out_cartAcc_var.cols() << ")" << RTT::endlog();
    // PRELOG(Error) << "[RTT] out_jacobian_var(" << out_jacobian_var.rows() << ", " << out_jacobian_var.cols() << ")" << RTT::endlog();
    // PRELOG(Error) << "[RTT] out_jacobianDot_var(" << out_jacobianDot_var.rows() << ", " << out_jacobianDot_var.cols() << ")" << RTT::endlog();
}

// void RTTKinDynMultiArm::compute(
//     const sensor_msgs::JointState &int_robotstatus,
//     Eigen::MatrixXd &out_inertia,
//     Eigen::MatrixXd &out_inertiaInv,
//     Eigen::VectorXd &out_gravity,
//     Eigen::VectorXd &out_coriolis,
//     Eigen::VectorXd &out_coriolisAndGravity,
//     geometry_msgs::Pose &out_cartPos,
//     Eigen::VectorXd &out_cartVel,
//     Eigen::VectorXd &out_cartAcc,
//     Eigen::MatrixXd &out_jacobian,
//     Eigen::MatrixXd &out_jacobianDot)
// {
//     //////////////////////////
//     // Initialize Variables //
//     //////////////////////////
//     out_inertia.setZero();
//     out_gravity.setZero();
//     out_coriolis.setZero();
//     out_cartPos.position.x = 0;
//     out_cartPos.position.y = 0;
//     out_cartPos.position.z = 0;
//     out_cartPos.orientation.w = 0;
//     out_cartPos.orientation.x = 0;
//     out_cartPos.orientation.y = 0;
//     out_cartPos.orientation.z = 0;
//     out_cartVel.setZero();
//     out_jacobian.setZero();
//     out_jacobianDot.setZero();

//     // initialize KDL fields
//     G_.data.setZero();
//     M_.data.setZero();
//     C_.data.setZero();

//     unsigned int numPreviousJoints = 0;
//     for (unsigned int manipulator = 0; manipulator < this->numRobotArms + this->numObjects; manipulator++)
//     {
//         jntPosConfigPlusJntVelConfig_q_i.at(manipulator).q.data.setZero();
//         jntPosConfigPlusJntVelConfig_q_i.at(manipulator).qdot.data.setZero();

//         G_i.at(manipulator).data.setZero();
//         M_i.at(manipulator).data.setZero();
//         C_i.at(manipulator).data.setZero();
//         jac_i.at(manipulator).data.setZero();
//         jac_dot_i.at(manipulator).data.setZero();

//         //set current joint positions and velocities
//         /* initialize strange stuff for solvers */
//         this->castEigenVectorFtoD(robotstatus.angles.segment(numPreviousJoints, kdl_chain_i.at(manipulator).getNrOfJoints()), jntPosConfigPlusJntVelConfig_q_i.at(manipulator).q.data);
//         this->castEigenVectorFtoD(robotstatus.velocities.segment(numPreviousJoints, kdl_chain_i.at(manipulator).getNrOfJoints()), jntPosConfigPlusJntVelConfig_q_i.at(manipulator).qdot.data);

//         /* execute solvers for inv.Dynamics */
//         // calculate matrices G(gravitation), M(inertia) and C(coriolis)
//         std::transform(this->DynamicsType.begin(), this->DynamicsType.end(), this->DynamicsType.begin(), [](unsigned char c) {
//             return std::toupper(c);
//         });
//         if (this->DynamicsType == "KDL")
//         {
//             id_dyn_solver_i.at(manipulator)->JntToGravity(jntPosConfigPlusJntVelConfig_q_i.at(manipulator).q, G_i.at(manipulator));
//             id_dyn_solver_i.at(manipulator)->JntToMass(jntPosConfigPlusJntVelConfig_q_i.at(manipulator).q, M_i.at(manipulator));
//             id_dyn_solver_i.at(manipulator)->JntToCoriolis(jntPosConfigPlusJntVelConfig_q_i.at(manipulator).q, jntPosConfigPlusJntVelConfig_q_i.at(manipulator).qdot, C_i.at(manipulator));
//         }
//         else if (this->DynamicsType == "KUKACLOSEDFORM")
//         {
//             kukaClosedForm.calc_inertia_matrix(tempInertia, jntPosConfigPlusJntVelConfig_q_i.at(manipulator).q.data.data());
//             M_i.at(manipulator).data = Eigen::Map<Eigen::MatrixXd>(*tempInertia, 7, 7);

//             kukaClosedForm.calc_coriolis_matrix(tempCorilois, jntPosConfigPlusJntVelConfig_q_i.at(manipulator).q.data.data(), jntPosConfigPlusJntVelConfig_q_i.at(manipulator).qdot.data.data());
//             C_i.at(manipulator).data = Eigen::Map<Eigen::MatrixXd>(*tempCorilois, 7, 7) * jntPosConfigPlusJntVelConfig_q_i.at(manipulator).qdot.data;

//             kukaClosedForm.calc_gravity_torque_vector(tempGravity, jntPosConfigPlusJntVelConfig_q_i.at(manipulator).q.data.data());
//             G_i.at(manipulator).data = Eigen::Map<Eigen::VectorXd>(tempGravity, 7);
//         }
//         /* execute solver for Jacobian based on velocities */
//         //calculate jacobian and jacobian dot

//         //TODO DLW for each manipulator I need to calculate the elbow and so on by adjusting kdl_chain_i.at(manipulator).getNrOfSegments()!
//         jnt_to_jac_solver_i.at(manipulator)->JntToJac(jntPosConfigPlusJntVelConfig_q_i.at(manipulator).q, jac_i.at(manipulator), kdl_chain_i.at(manipulator).getNrOfSegments());
//         jnt_to_jac_dot_solver_i.at(manipulator)->JntToJacDot(jntPosConfigPlusJntVelConfig_q_i.at(manipulator), jac_dot_i.at(manipulator), kdl_chain_i.at(manipulator).getNrOfSegments());

//         // forward kinematics: calculate cartesian position based on joint angles
//         jnt_to_cart_pos_solver_i.at(manipulator)->JntToCart(jntPosConfigPlusJntVelConfig_q_i.at(manipulator).q, cartPosFrame_i.at(manipulator), kdl_chain_i.at(manipulator).getNrOfSegments());
//         jnt_to_cart_vel_solver_i.at(manipulator)->JntToCart(jntPosConfigPlusJntVelConfig_q_i.at(manipulator), cartVelFrame_i.at(manipulator), kdl_chain_i.at(manipulator).getNrOfSegments());

//         // M_.data.block(numPreviousJoints, numPreviousJoints, kdl_chain_i.at(manipulator).getNrOfJoints(), kdl_chain_i.at(manipulator).getNrOfJoints()) = M_i.at(manipulator).data;
//         // G_.data.segment(numPreviousJoints, kdl_chain_i.at(manipulator).getNrOfJoints()) = G_i.at(manipulator).data;
//         // C_.data.segment(numPreviousJoints, kdl_chain_i.at(manipulator).getNrOfJoints()) = C_i.at(manipulator).data;
//         if (this->include_object_dynamics || manipulator < this->numRobotArms)
//         {
//             M_.data.block(numPreviousJoints, numPreviousJoints, kdl_chain_i.at(manipulator).getNrOfJoints(), kdl_chain_i.at(manipulator).getNrOfJoints()) = M_i.at(manipulator).data;
//             if (this->include_gravity || manipulator >= this->numRobotArms) // Which means it is an object...
//             {
//                 G_.data.segment(numPreviousJoints, kdl_chain_i.at(manipulator).getNrOfJoints()) = G_i.at(manipulator).data;
//             }
//             else
//             {
//                 G_.data.segment(numPreviousJoints, kdl_chain_i.at(manipulator).getNrOfJoints()).setZero();
//             }
//             C_.data.segment(numPreviousJoints, kdl_chain_i.at(manipulator).getNrOfJoints()) = C_i.at(manipulator).data;
//         }
//         else
//         {
//             M_.data.block(numPreviousJoints, numPreviousJoints, kdl_chain_i.at(manipulator).getNrOfJoints(), kdl_chain_i.at(manipulator).getNrOfJoints()) = M_i.at(manipulator).data; //TODO fix for low rank when zero
//             G_.data.segment(numPreviousJoints, kdl_chain_i.at(manipulator).getNrOfJoints()).setZero();
//             C_.data.segment(numPreviousJoints, kdl_chain_i.at(manipulator).getNrOfJoints()).setZero();
//         }

//         this->castEigenMatrixDtoF(jac_i.at(manipulator).data, jacobian_i.at(manipulator));
//         this->castEigenMatrixDtoF(jac_dot_i.at(manipulator).data, jacobianDot_i.at(manipulator));

//         jacobian.block(manipulator * 6, numPreviousJoints, 6, kdl_chain_i.at(manipulator).getNrOfJoints()) = jacobian_i.at(manipulator);
//         jacobianDot.block(manipulator * 6, numPreviousJoints, 6, kdl_chain_i.at(manipulator).getNrOfJoints()) = jacobianDot_i.at(manipulator);

//         double x, y, z, w;
//         cartPosFrame_i.at(manipulator).M.GetQuaternion(x, y, z, w);
//         cartPos(manipulator * 7 + 0) = cartPosFrame_i.at(manipulator).p.x();
//         cartPos(manipulator * 7 + 1) = cartPosFrame_i.at(manipulator).p.y();
//         cartPos(manipulator * 7 + 2) = cartPosFrame_i.at(manipulator).p.z();
//         cartPos(manipulator * 7 + 3) = w;
//         cartPos(manipulator * 7 + 4) = x;
//         cartPos(manipulator * 7 + 5) = y;
//         cartPos(manipulator * 7 + 6) = z;

//         cartVel(manipulator * 6 + 0) = cartVelFrame_i.at(manipulator).GetTwist().vel.x(); //similar to velFrame.p.v.x();
//         cartVel(manipulator * 6 + 1) = cartVelFrame_i.at(manipulator).GetTwist().vel.y(); //similar to velFrame.p.v.y();
//         cartVel(manipulator * 6 + 2) = cartVelFrame_i.at(manipulator).GetTwist().vel.z(); //similar to velFrame.p.v.z();
//         cartVel(manipulator * 6 + 3) = cartVelFrame_i.at(manipulator).GetTwist().rot.x();
//         cartVel(manipulator * 6 + 4) = cartVelFrame_i.at(manipulator).GetTwist().rot.y();
//         cartVel(manipulator * 6 + 5) = cartVelFrame_i.at(manipulator).GetTwist().rot.z();

//         //TODO: compare with: cartVel = jacobian * robotstatus.angles

//         numPreviousJoints += kdl_chain_i.at(manipulator).getNrOfJoints();

//         // out_cartPos_vars.at(manipulator).translation.translation = cartPos.segment<3>(manipulator * 7);
//     }
//     //assert(numPreviousJoints == this->DOFsizeActive);
//     assert(numPreviousJoints == this->DOFsize);
//     this->castEigenMatrixDtoF(M_.data, inertia);
//     this->castEigenVectorDtoF(G_.data, gravity);
//     this->castEigenVectorDtoF(C_.data, coriolis);

//     for (unsigned int object = 0; object < this->numObjects; object++)
//     {
//         jacobian.block<6, 6>(this->numRobotArms * 6 + object * 6, this->DOFsizeActive + object * 6) = this->identity66;
//         jacobianDot.block<6, 6>(this->numRobotArms * 6 + object * 6, this->DOFsizeActive + object * 6) = this->zero66;
//     }

//     // NO_OBJECT_COMPENSATION:
//     float alpha = 0.01;
//     // FULL_OBJECT_COMPENSATION:
//     alpha = 1;
//     if (this->numObjects == 1)
//     {
//         inertia.block<6, 6>(this->DOFsize - 6, this->DOFsize - 6) = inertia.block<6, 6>(this->DOFsize - 6, this->DOFsize - 6) * alpha;
//         gravity.tail<6>() = gravity.tail<6>() * alpha;
//         coriolis.tail<6>() = coriolis.tail<6>() * alpha;
//     }

//     inertiaInv = inertia.inverse();
//     if (!this->include_gravity)
//     {
//         gravity.setZero();
//         bool oneGdataReceived = false;
//         for (unsigned int arms = 0; arms < this->numRobotArms; arms++)
//         {
//             if (in_external_gravity_ports[arms]->connected())
//             {
//                 if (in_external_gravity_flows[arms] != RTT::NoData)
//                 {
//                     oneGdataReceived = true;
//                     int slot = this->DOFsize / this->numRobotArms;
//                     // why plus? this doesn't make any sense...
//                     gravity.segment(arms * slot, slot).noalias() += in_external_gravity_vars[arms].torques;
//                 }
//             }
//         }
//         if (!oneGdataReceived) // first line make this irrelevant now...
//         {
//             gravity.setZero();
//         }
//     }

//     coriolisAndGravity = gravity + coriolis;
//     cartAcc = jacobianDot * robotstatus.velocities; //TODO: add out_jacobian_var * in_robotstatus_var.accelerations
// }

//this macro should appear only once per library
ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::RTTKinDynMultiArm)