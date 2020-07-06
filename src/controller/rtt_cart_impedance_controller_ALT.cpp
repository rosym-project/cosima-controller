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

#include "../../include/cosima-controller/controller/rtt_cart_impedance_controller.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

using namespace cosima;
using namespace controller;

RTTCartImpCtrl::RTTCartImpCtrl(std::string const &name) : RTTStackedCtrl(name), include_gravity(true)
{
    addProperty("include_gravity", this->include_gravity)
        .doc("Include the gravity term in the commanded torques (default: true).");

    addOperation("setGains", &RTTCartImpCtrl::setGains, this);

    this->kp = 100;
    this->kd = 3;

    addProperty("kp", this->kp);
    addProperty("kd", this->kd);
}

void RTTCartImpCtrl::setGains(const double &kp, const double &kd)
{
    this->kp = kp;
    this->kd = kd;
}

double RTTCartImpCtrl::getOrocosTime()
{
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

bool RTTCartImpCtrl::configureHook()
{
    if (this->vec_stacked_robot_info.size() <= 0)
    {
        PRELOG(Warning) << "Please add at least one robot, before configuring this component!" << RTT::endlog();
        return false;
    }

    // count the total DoF size of the added robots
    this->calculateStackedVariables();

    in_robotstatus_var = sensor_msgs::JointState();
    for (unsigned int i = 0; i < this->vec_stacked_robot_info.size(); i++)
    {
        for (unsigned int j = 0; j < this->vec_stacked_robot_info[i].jDoF; j++)
        {
            in_robotstatus_var.position.push_back(0.0);
            in_robotstatus_var.velocity.push_back(0.0);
            in_robotstatus_var.effort.push_back(0.0);
        }
    }

    // IDEA FOR CONVERSION OF ROS etc...
    // std::vector<double> a = {1, 2, 3, 4};
    // Eigen::VectorXd b = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(a.data(), a.size());

    in_cart_pos_cmd_var = Eigen::VectorXd::Zero(this->total_tDoF_quat);
    in_cart_pos_cmd_port.setName("in_cart_pos_cmd_port");
    in_cart_pos_cmd_port.doc("InputPort for reading cartesian-space position commands (concatenated 3 dof pos + 4 dof orn (quat))");
    ports()->addPort(in_cart_pos_cmd_port);
    in_cart_pos_cmd_flow = RTT::NoData;

    in_cart_vel_cmd_var = Eigen::VectorXd::Zero(this->total_tDoF_euler);
    in_cart_vel_cmd_port.setName("in_cart_vel_cmd_port");
    in_cart_vel_cmd_port.doc("InputPort for reading cartesian-space velocity commands (concatenated 3 dof pos + 3 dof orn (euler))");
    ports()->addPort(in_cart_vel_cmd_port);
    in_cart_vel_cmd_flow = RTT::NoData;

    in_cart_acc_cmd_var = Eigen::VectorXd::Zero(this->total_tDoF_euler);
    in_cart_acc_cmd_port.setName("in_cart_acc_cmd_port");
    in_cart_acc_cmd_port.doc("InputPort for reading cartesian-space acceleration commands (concatenated 3 dof pos + 3 dof orn (euler))");
    ports()->addPort(in_cart_acc_cmd_port);
    in_cart_acc_cmd_flow = RTT::NoData;

    in_cart_pos_fdb_var = Eigen::VectorXd::Zero(this->total_tDoF_quat);
    in_cart_pos_fdb_port.setName("in_cart_pos_fdb_port");
    in_cart_pos_fdb_port.doc("InputPort for reading cartesian-space position feedback (concatenated 3 dof pos + 4 dof orn (quat))");
    ports()->addPort(in_cart_pos_fdb_port);
    in_cart_pos_fdb_flow = RTT::NoData;

    in_cart_vel_fdb_var = Eigen::VectorXd::Zero(this->total_tDoF_euler);
    in_cart_vel_fdb_port.setName("in_cart_vel_fdb_port");
    in_cart_vel_fdb_port.doc("InputPort for reading cartesian-space velocity feedback (concatenated 3 dof pos + 3 dof orn (euler))");
    ports()->addPort(in_cart_vel_fdb_port);
    in_cart_vel_fdb_flow = RTT::NoData;

    in_cart_acc_fdb_var = Eigen::VectorXd::Zero(this->total_tDoF_euler);
    in_cart_acc_fdb_port.setName("in_cart_acc_fdb_port");
    in_cart_acc_fdb_port.doc("InputPort for reading cartesian-space acceleration feedback (concatenated 3 dof pos + 3 dof orn (euler))");
    ports()->addPort(in_cart_acc_fdb_port);
    in_cart_acc_fdb_flow = RTT::NoData;

    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("InputPort for reading the current robot joint-space status");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    in_coriolisAndGravity_var = Eigen::VectorXd::Zero(this->total_jDoF);
    in_coriolisAndGravity_port.setName("in_coriolisAndGravity_port");
    in_coriolisAndGravity_port.doc("InputPort for reading coriolisAndGravity vector");
    ports()->addPort(in_coriolisAndGravity_port);
    in_coriolisAndGravity_flow = RTT::NoData;

    in_inertia_var = Eigen::MatrixXd::Zero(this->total_jDoF, this->total_jDoF);
    in_inertia_port.setName("in_inertia_port");
    in_inertia_port.doc("InputPort for reading joint-space inertia");
    ports()->addPort(in_inertia_port);
    in_inertia_flow = RTT::NoData;

    out_torques_var = Eigen::VectorXd::Zero(this->total_jDoF);
    out_torques_port.setName("out_torques_port");
    out_torques_port.doc("OutputPort for sending the command torques");
    out_torques_port.setDataSample(out_torques_var);
    ports()->addPort(out_torques_port);

    in_jacobian_var = Eigen::MatrixXd::Zero(this->total_tDoF_euler, this->total_jDoF);
    in_jacobian_port.setName("in_jacobian_port");
    in_jacobian_port.doc("InputPort for reading jacobian");
    ports()->addPort(in_jacobian_port);
    in_jacobian_flow = RTT::NoData;

    in_jacobianDot_var = Eigen::MatrixXd::Zero(this->total_tDoF_euler, this->total_jDoF);
    in_jacobianDot_port.setName("in_jacobianDot_port");
    in_jacobianDot_port.doc("InputPort for reading jacobian dot");
    ports()->addPort(in_jacobianDot_port);
    in_jacobianDot_flow = RTT::NoData;

    in_projection_var = Eigen::MatrixXd::Zero(this->total_jDoF, this->total_jDoF);
    in_projection_port.setName("in_projection_port");
    in_projection_port.doc("InputPort for reading the projection matrix (if not used needs to be one)");
    ports()->addPort(in_projection_port);
    in_projection_flow = RTT::NoData;

    in_projectionDot_var = Eigen::MatrixXd::Zero(this->total_jDoF, this->total_jDoF);
    in_projectionDot_port.setName("in_projectionDot_port");
    in_projectionDot_port.doc("InputPort for reading the projection matrix (if not used needs to be zero)");
    ports()->addPort(in_projectionDot_port);
    in_projectionDot_flow = RTT::NoData;

    return true;
}

bool RTTCartImpCtrl::startHook()
{
    return true;
}

void RTTCartImpCtrl::updateHook()
{
    out_torques_var.setZero();

    //////////////////////////
    // READ DATA FROM PORTS //
    //////////////////////////

    // Get cmd input
    in_cart_pos_cmd_flow = in_cart_pos_cmd_port.read(in_cart_pos_cmd_var);
    // if (in_cart_cmd_flow != RTT::NewData)
    // {
    //     if (virtual_joint_cmd_flow == RTT::NewData)
    //     {
    //         in_cart_cmd_var = virtual_joint_cmd_var;
    //         virtual_joint_cmd_flow = RTT::OldData;
    //     }
    // }
    in_cart_vel_cmd_flow = in_cart_vel_cmd_port.read(in_cart_vel_cmd_var);
    if (in_cart_vel_cmd_flow != RTT::NewData)
    {
        in_cart_vel_cmd_var.setZero();
    }
    in_cart_acc_cmd_flow = in_cart_acc_cmd_port.read(in_cart_acc_cmd_var);
    if (in_cart_acc_cmd_flow != RTT::NewData)
    {
        in_cart_acc_cmd_var.setZero();
    }

    in_cart_pos_fdb_flow = in_cart_pos_fdb_port.read(in_cart_pos_fdb_var);
    in_cart_vel_fdb_flow = in_cart_vel_fdb_port.read(in_cart_vel_fdb_var);
    in_cart_acc_fdb_flow = in_cart_acc_fdb_port.read(in_cart_acc_fdb_var);

    in_jacobian_flow = in_jacobian_port.read(in_jacobian_var);
    in_jacobianDot_flow = in_jacobianDot_port.read(in_jacobianDot_var);

    if (in_projection_port.connected())
    {
        in_projection_flow = in_projection_port.read(in_projection_var);
    }
    if (in_projectionDot_port.connected())
    {
        in_projectionDot_flow = in_projectionDot_port.read(in_projectionDot_var);
    }
    // If the projection P (optional data) is missing, set it to identity.
    if (in_projection_flow == RTT::NoData)
    {
        in_projection_var.setIdentity();
    }
    else
    {
        // If we have received the projection P at least once, we also need to receive Pd.
        if (in_projectionDot_flow == RTT::NoData)
        {
            RTT::log(RTT::Error) << "Error in_projectionDot_flow == RTT::NoData" << RTT::endlog();
            return;
        }
    }
    // If the projection dot P_dot (optional data) is missing, set it to Zero.
    if (in_projectionDot_flow == RTT::NoData)
    {
        in_projectionDot_var.setZero();
    }

    in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    if (in_robotstatus_flow == RTT::NoData)
    {
        PRELOG(Error) << "Houston we have a problem! No data on in_robotstatus_port." << RTT::endlog();
        return; // Return is better, because in this case we don't send a command at all!
    }

    in_inertia_flow = in_inertia_port.read(in_inertia_var);
    if (in_inertia_flow == RTT::NoData)
    {
        PRELOG(Error) << "Houston we have a problem! No data on in_inertia_port." << RTT::endlog();
        return; // Return is better, because in this case we don't send a command at all!
    }

    if (this->include_gravity)
    {
        in_coriolisAndGravity_flow = in_coriolisAndGravity_port.read(in_coriolisAndGravity_var);
        if (in_coriolisAndGravity_flow == RTT::NoData)
        {
            PRELOG(Error) << "Houston we have a problem! No data on in_coriolisAndGravity_port." << RTT::endlog();
            return; // Return is better, because in this case we don't send a command at all!
        }

        out_torques_var = in_coriolisAndGravity_var;
    }

    double timeStep = this->getPeriod();

    //////////////////////////
    // SANITY CHECK OF DATA //
    //////////////////////////

    // TODO check also others for NAN!
    if (in_cart_pos_fdb_var.hasNaN())
    {
        PRELOG(Error) << "in_cart_pos_fdb_var NAN!=\n"
                             << in_cart_pos_fdb_var << RTT::endlog();
    }

    Eigen::MatrixXf in_inertiaInv = in_inertia_var.inverse();

    // TODO don't send anything.
    if (in_inertiaInv.isZero())
    {
        RTT::log(RTT::Error) << this->getName() << ": in_inertiaInv is zero..." << RTT::endlog();
        out_torques_var.setZero();
        return;
    }

    // TODO don't send anything.
    if (in_projection_var.isZero())
    {
        RTT::log(RTT::Error) << this->getName() << ": in_projection is zero..." << RTT::endlog();
        out_torques_var.setZero();
        return;
    }

    ///////////////////////////
    // CALCULATE ERROR TERMS //
    ///////////////////////////

    //     # if position is being controlled
    //     if np.sum(self.ctrlr_dof[:3]) > 0:
    //         xyz = np.array(pos)
    //         u_task[:3] = xyz - desiredPositions[:3]

    //     # if orientation is being controlled
    //     if np.sum(self.ctrlr_dof[3:]) > 0:
    //         u_task[3:] = self._calc_orientation_forces(desiredPositions[3:], q, non_bullet_orn)

    // # task space integrated error term
    // if self.ki != 0:
    //     self.integrated_error += u_task
    //     u_task += self.ki * self.integrated_error


    // if self.vmax is not None:
    //     # if max task space velocities specified, apply velocity limiting
    //     u_task = self._velocity_limiting(u_task)
    // else:
    //     # otherwise apply specified gains
    //     u_task *= self.task_space_gains



    // for (unsigned int i = 0; i < this->total_dof_size; i++)
    // {
    //     qError(i) = this->in_cart_cmd_var.positions[i] - in_robotstatus_var.position[i];
    //     qdError(i) = this->in_cart_cmd_var.velocities[i] - in_robotstatus_var.velocity[i];
    // }

    // Calculate positional error

    // Calculate orientational error

    // Weight the error

    // self.kp = kp
    // self.ko = kp if ko is None else ko
    // # TODO: find the appropriate default critical damping value
    // # when using different position and orientation gains
    // self.kv = np.sqrt(self.kp + self.ko) if kv is None else kv
    // self.ki = ki
    // self.null_controllers = null_controllers
    // self.use_g = use_g
    // self.use_C = use_C
    // self.orientation_algorithm = orientation_algorithm

    // if self.ki != 0:
    //     self.integrated_error = np.zeros(6)

    // self.task_space_gains = np.array([self.kp] * 3 + [self.ko] * 3)

    // self.vmax = vmax
    // if vmax is not None:
    //     # precalculate gains used in velocity limiting
    //     self.sat_gain_xyz = vmax[0] / self.kp * self.kv
    //     self.sat_gain_abg = vmax[1] / self.ko * self.kv
    //     self.scale_xyz = vmax[0] / self.kp * self.kv
    //     self.scale_abg = vmax[1] / self.ko * self.kv


    //////////////////////
    // CONTROL EQUATION //
    //////////////////////

    // Calculate task-space inertia
    // Mx, M_inv = self._Mx(M=M, J=J)  # inertia matrix in task space


    // # compensate for velocity
    // if np.all(desiredVelocities == 0):
    //     # if there's no target velocity in task space,
    //     # compensate for velocity in joint space (more accurate)
    //     u = -1 * self.kv * np.dot(M, qdot)
    // else:
    //     dx = np.zeros(6)
    //     # dx[self.ctrlr_dof] = np.dot(J, qdot)
    //     dx = np.dot(J, qdot)
    //     u_task += self.kv * (dx - desiredVelocities)

    // # isolate task space forces corresponding to controlled DOF
    // # u_task = u_task[self.ctrlr_dof]

    // # transform task space control signal into joint space ----------------
    // u -= np.dot(J.T, np.dot(Mx, u_task))

    // # add in estimation of full centrifugal and Coriolis effects ----------
    // # if self.use_C:
    // #     u -= np.dot(self.robot_config.C(q=q, dq=qdot), qdot)


    // # add in gravity term in joint space ----------------------------------
    //     # if self.use_g:
    //     u -= np.array(self._pb.calculateInverseDynamics(bodyUniqueId, q1, qdot1, zeroAccelerations))




    //////////////////////////
    // GRAVITY COMPENSATION //
    //////////////////////////


    //////////////////////////
    // SAFETY-RELATED STUFF //
    //////////////////////////

    // // NULLSPACE>????????
    // # add in secondary control signals ------------------------------------
    //     if self.null_controllers is not None:
    //         for null_controller in self.null_controllers:
    //             # generate control signal to apply in null space
    //             u_null = null_controller.generate(q, qdot)
    //             # calculate null space filter
    //             Jbar = np.dot(M_inv, np.dot(J.T, Mx))
    //             null_filter = self.IDENTITY_N_JOINTS - np.dot(J.T, Jbar.T)
    //             # add in filtered null space control signal
    //             u += np.dot(null_filter, u_null)

    out_torques_port.write(out_torques_var);
}

void RTTCartImpCtrl::stopHook()
{
}

void RTTCartImpCtrl::cleanupHook()
{
    if (this->ports()->getPort("in_robotstatus_port"))
    {
        this->ports()->removePort("in_robotstatus_port");
    }

    if (this->ports()->getPort("in_coriolisAndGravity_port"))
    {
        this->ports()->removePort("in_coriolisAndGravity_port");
    }

    if (this->ports()->getPort("out_torques_port"))
    {
        this->ports()->removePort("out_torques_port");
    }

    if (this->ports()->getPort("in_inertia_port"))
    {
        this->ports()->removePort("in_inertia_port");
    }

    if (this->ports()->getPort("in_jacobian_port"))
    {
        this->ports()->removePort("in_jacobian_port");
    }

    if (this->ports()->getPort("in_jacobianDot_port"))
    {
        this->ports()->removePort("in_jacobianDot_port");
    }

    if (this->ports()->getPort("in_projection_port"))
    {
        this->ports()->removePort("in_projection_port");
    }

    if (this->ports()->getPort("in_projectionDot_port"))
    {
        this->ports()->removePort("in_projectionDot_port");
    }

    if (this->ports()->getPort("in_cart_pos_cmd_port"))
    {
        this->ports()->removePort("in_cart_pos_cmd_port");
    }

    if (this->ports()->getPort("in_cart_vel_cmd_port"))
    {
        this->ports()->removePort("in_cart_vel_cmd_port");
    }

    if (this->ports()->getPort("in_cart_acc_cmd_port"))
    {
        this->ports()->removePort("in_cart_acc_cmd_port");
    }

    if (this->ports()->getPort("in_cart_pos_fdb_port"))
    {
        this->ports()->removePort("in_cart_pos_fdb_port");
    }

    if (this->ports()->getPort("in_cart_vel_fdb_port"))
    {
        this->ports()->removePort("in_cart_vel_fdb_port");
    }

    if (this->ports()->getPort("in_cart_acc_fdb_port"))
    {
        this->ports()->removePort("in_cart_acc_fdb_port");
    }
}

//this macro should appear only once per library
// ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::controller::RTTCartImpCtrl)
