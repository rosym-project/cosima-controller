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

#include "../../include/cosima-controller/coordination/rtt_contact_situation_switcher.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

using namespace cosima;
using namespace coordination;

ContactSituationSwitcher::ContactSituationSwitcher(std::string const &name) : RTT::TaskContext(name), skill_type(0)
{
    converged_th = 0.001;
    this->addProperty("converged_th", converged_th);

    move_speed_trans = 0.0001;
    // this->addProperty("move_speed_trans", move_speed_trans);

    slerp_time_ = 0.0;
    move_time_ = 0.0;

    move_speed_rot_max = 5;
    this->addProperty("move_speed_rot_max", move_speed_rot_max);

    move_speed_trans_max = 15.0;
    this->addProperty("move_speed_trans_max", move_speed_trans_max);
    
    // RTT::OutputPort<bool> port;
    // this->ports()->addPort("my_port", port);
    // port.createStream(rtt_roscomm::topic("my_ros_topic"));
    // rtt_rosservice::ROSService

    this->addOperation("assemble_srv",&ContactSituationSwitcher::assemble_srv,this);
    this->addOperation("move_srv",&ContactSituationSwitcher::move_srv,this);

}

bool ContactSituationSwitcher::assemble_srv(cosima_msgs::AssembleRequest& req,cosima_msgs::AssembleResponse& resp)
{

}

bool ContactSituationSwitcher::move_srv(cosima_msgs::MoveRequest& req,cosima_msgs::MoveResponse& resp)
{
    
    //
    //
    if (skill_type == 0)
    {
        _pose_var_trans(0) = req.i_pose.position.x;
        _pose_var_trans(1) = req.i_pose.position.y;
        _pose_var_trans(2) = req.i_pose.position.z;
        //
        _pose_var_orn.w() = req.i_pose.orientation.w;
        _pose_var_orn.x() = req.i_pose.orientation.x;
        _pose_var_orn.y() = req.i_pose.orientation.y;
        _pose_var_orn.z() = req.i_pose.orientation.z;

        move_speed_rot_max = req.i_max_rot_sec;
        move_speed_trans_max = req.i_max_trans_sec;

        // IDLE -> Do It
        PRELOG(Error) << "Received command " << _pose_var_trans << "with rot = " << move_speed_rot_max << " and trans = " << move_speed_trans_max << " in IDLE" << RTT::endlog();
        new_move_command = true;
        // Wait until main() sends data
        std::unique_lock<std::mutex> lck(m);
        cv.wait(lck);
    }
    else
    {
        PRELOG(Error) << "Error: Already processing!" << RTT::endlog();
        return false;
    }
    // else if (skill_type == 1)
    // {
    //     // MOVE -> Do It
    //     PRELOG(Error) << "Received command " << _pose_var_trans << " in MOVE" << RTT::endlog();
    //     new_move_command = true;
    //     // Wait until main() sends data
    //     std::unique_lock<std::mutex> lck(m);
    //     cv.wait(lck);
    // }
    // else if (skill_type == 2)
    // {
    //     // ASSEMBLY -> Do Nothing
    // }

    return true;
}


double ContactSituationSwitcher::getOrocosTime()
{
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

// void ContactSituationSwitcher::addWrenchMonitor(std::string const &name)
// {
//     map_monitors_wrench[name] = std::shared_ptr<MonitorWrench>(new MonitorWrench(name, this));
// }

// void ContactSituationSwitcher::setWrenchMonitorBounds(std::string const &name, unsigned int const &dimension, double lower, double upper)
// {
//     map_monitors_wrench[name]->activateBounds(dimension, lower, upper);
// }

bool ContactSituationSwitcher::configureHook()
{
    // Params
    new_move_command = false;
    new_assemble_command = false;
    skill_type = 0;


    // Ports

    // InputPort: current cart pose
    in_pose_var = geometry_msgs::Pose();
    in_pose_var.position.x = 0.0;
    in_pose_var.position.y = 0.0;
    in_pose_var.position.z = 0.0;
    //
    in_pose_var.orientation.w = 1.0;
    in_pose_var.orientation.x = 0.0;
    in_pose_var.orientation.y = 0.0;
    in_pose_var.orientation.z = 0.0;
    //
    in_pose_port.setName("in_pose_port");
    ports()->addPort(in_pose_port);
    in_pose_flow = RTT::NoData;

    // InputPort: current cart wrench
    in_wrench_var = geometry_msgs::Wrench();
    in_wrench_var.force.x = 0.0;
    in_wrench_var.force.y = 0.0;
    in_wrench_var.force.z = 0.0;
    //
    in_wrench_var.torque.x = 0.0;
    in_wrench_var.torque.y = 0.0;
    in_wrench_var.torque.z = 0.0;
    //
    in_wrench_port.setName("in_wrench_port");
    ports()->addPort(in_wrench_port);
    in_wrench_flow = RTT::NoData;

    // OutputPort: converged
    out_converged_var = std_msgs::Bool();
    out_converged_var.data = true; // converged
    out_converged_port.setName("out_converged_port");
    out_converged_port.setDataSample(out_converged_var);
    ports()->addPort(out_converged_port);

    // OutputPort: command gripper
    out_gripper_var = std_msgs::Bool();
    out_gripper_var.data = true; // open
    out_gripper_port.setName("out_gripper_port");
    out_gripper_port.setDataSample(out_gripper_var);
    ports()->addPort(out_gripper_port);

    // OutputPort: command pose of arm
    out_desiredTaskSpace_var = trajectory_msgs::MultiDOFJointTrajectoryPoint();
    out_desiredTaskSpace_var.transforms.push_back(geometry_msgs::Transform());
    //
    out_desiredTaskSpace_var.transforms[0].translation.x = 0.000158222;
    out_desiredTaskSpace_var.transforms[0].translation.y = -0.675439;
    out_desiredTaskSpace_var.transforms[0].translation.z = 0.285982;
    out_desiredTaskSpace_var.transforms[0].rotation.w = 0.0150682;
    out_desiredTaskSpace_var.transforms[0].rotation.x = 0.707013;
    out_desiredTaskSpace_var.transforms[0].rotation.y = -0.706876;
    out_desiredTaskSpace_var.transforms[0].rotation.z = 0.0152298;
    //
    // out_desiredTaskSpace_var.transforms[0].translation.x = 0.0;
    // out_desiredTaskSpace_var.transforms[0].translation.y = 0.0;
    // out_desiredTaskSpace_var.transforms[0].translation.z = 0.0;
    // out_desiredTaskSpace_var.transforms[0].rotation.w = 1.0;
    // out_desiredTaskSpace_var.transforms[0].rotation.x = 0.0;
    // out_desiredTaskSpace_var.transforms[0].rotation.y = 0.0;
    // out_desiredTaskSpace_var.transforms[0].rotation.z = 0.0;
    //
    out_desiredTaskSpace_var.velocities.push_back(geometry_msgs::Twist());
    out_desiredTaskSpace_var.accelerations.push_back(geometry_msgs::Twist());
    //
    out_desiredTaskSpace_port.setName("out_desiredTaskSpace_port");
    out_desiredTaskSpace_port.setDataSample(out_desiredTaskSpace_var);
    ports()->addPort(out_desiredTaskSpace_port);

    //
    _pose_var_trans = Eigen::Vector3d::Zero();
    _pose_var_orn = Eigen::Quaterniond::Identity();

    out_trans_ = Eigen::Vector3d::Zero();
    des_trans_ = Eigen::Vector3d::Zero();
    cur_trans_ = Eigen::Vector3d::Zero();

    out_orn_ = Eigen::Quaterniond::Identity();
    des_orn_ = Eigen::Quaterniond::Identity();
    cur_orn_ = Eigen::Quaterniond::Identity();

    this->skill_stack_ptr = this->getPeer("ctrl");
    if (this->skill_stack_ptr)
    {
        PRELOG(Error) << "ctrl peered!" << RTT::endlog();
        // Link to operations
        this->getOperation_updateContactSituationBlocking = this->skill_stack_ptr->getOperation("updateContactSituationBlocking");
        this->addOperation("updateContactSituationBlocking_srv",&ContactSituationSwitcher::updateContactSituationBlocking_srv,this);

        this->getOperation_setFFVec = this->skill_stack_ptr->getOperation("setFFVec");
        this->addOperation("setFFVec_srv",&ContactSituationSwitcher::setFFVec_srv,this);
    }
    else
    {
        PRELOG(Fatal) << "ctrl NOT peered!" << RTT::endlog();
        return false;
    }
    
    // ROS services
    boost::shared_ptr<rtt_rosservice::ROSService> rosservice = this->getProvider<rtt_rosservice::ROSService>("rosservice");
    if(rosservice)
    {
        rosservice->connect("assemble_srv",this->getName()+"/assemble_srv","cosima_msgs/Assemble");
        rosservice->connect("move_srv",this->getName()+"/move_srv","cosima_msgs/Move");
        if (this->skill_stack_ptr)
        {
            rosservice->connect("updateContactSituationBlocking_srv",this->getName()+"/updateContactSituationBlocking_srv","cosima_msgs/ContactSituation");

            rosservice->connect("setFFVec_srv",this->getName()+"/setFFVec_srv","cosima_msgs/ContactForce");
        }
    }
    else
    {
        PRELOG(Error) << "ROSService not available" << RTT::endlog();
    }
    return true;
}

bool ContactSituationSwitcher::updateContactSituationBlocking_srv(cosima_msgs::ContactSituationRequest& req,cosima_msgs::ContactSituationResponse& resp)
{
    Eigen::VectorXd kp = Eigen::VectorXd::Zero(6);
    kp(0) = req.kp_trans.x;
    kp(1) = req.kp_trans.y;
    kp(2) = req.kp_trans.z;
    kp(3) = req.kp_rot.x;
    kp(4) = req.kp_rot.y;
    kp(5) = req.kp_rot.z;

    Eigen::VectorXd kd = Eigen::VectorXd::Zero(6);
    kd(0) = req.kd_trans.x;
    kd(1) = req.kd_trans.y;
    kd(2) = req.kd_trans.z;
    kd(3) = req.kd_rot.x;
    kd(4) = req.kd_rot.y;
    kd(5) = req.kd_rot.z;

    Eigen::VectorXd fdir = Eigen::VectorXd::Zero(6);
    fdir(0) = req.fdir_trans.x;
    fdir(1) = req.fdir_trans.y;
    fdir(2) = req.fdir_trans.z;
    fdir(3) = req.fdir_rot.x;
    fdir(4) = req.fdir_rot.y;
    fdir(5) = req.fdir_rot.z;

    Eigen::VectorXd force = Eigen::VectorXd::Zero(6);
    force(0) = req.force_trans.x;
    force(1) = req.force_trans.y;
    force(2) = req.force_trans.z;
    force(3) = req.force_rot.x;
    force(4) = req.force_rot.y;
    force(5) = req.force_rot.z;

    this->getOperation_updateContactSituationBlocking(kp, kd, fdir, force, req.time);
    return true;
}

bool ContactSituationSwitcher::setFFVec_srv(cosima_msgs::ContactForceRequest& req,cosima_msgs::ContactForceResponse& resp)
{
    Eigen::VectorXd force = Eigen::VectorXd::Zero(6);
    force(0) = req.wrench.force.x;
    force(1) = req.wrench.force.y;
    force(2) = req.wrench.force.z;
    force(3) = req.wrench.torque.x;
    force(4) = req.wrench.torque.y;
    force(5) = req.wrench.torque.z;

    this->getOperation_setFFVec(force);
    return true;
}

bool ContactSituationSwitcher::startHook()
{
    // st = this->getOrocosTime();
    return true;
}

void ContactSituationSwitcher::updateHook()
{
    // for (auto const& x : map_monitors_wrench)
    // {
    //     x.second->eval();
    // }

    in_pose_flow = in_pose_port.read(in_pose_var);
    if (in_pose_flow == RTT::NoData)
    {
        return;
    }

    if (skill_type == 0) // IDLE
    {
        if (new_assemble_command)
        {
            skill_type = 2;
        }

        if (new_move_command)
        {
            skill_type = 1;
        }
    }

    ////////////////////////////////////////
    
    if (skill_type == 1) // MOVE
    {
        if (new_move_command)
        {
            // TODO
            new_move_command = false;
            skill_type = 1;
            out_converged_var.data = false;
            out_converged_port.write(out_converged_var);

            des_trans_(0) = _pose_var_trans(0);
            des_trans_(1) = _pose_var_trans(1);
            des_trans_(2) = _pose_var_trans(2);

            des_orn_.w() = _pose_var_orn.w();
            des_orn_.x() = _pose_var_orn.x();
            des_orn_.y() = _pose_var_orn.y();
            des_orn_.z() = _pose_var_orn.z();

            slerp_time_ = 0.0;
            move_time_ = 0.0;

            PRELOG(Error) << "Received:\n" << des_trans_ << RTT::endlog();

            // Find the difference quaternion: qd = inverse(q1)*q2
            Eigen::Quaterniond qd = cur_orn_.inverse()*des_orn_;
            // Then find the angle between q1 and q2:
            double angle = 2.0 * atan2(qd.vec().norm(), qd.w()); // NOTE: signed

            // PRELOG(Error) << "des_orn_:\n" << " w = " << des_orn_.w() << "\n x = " << des_orn_.x() << "\n y = " << des_orn_.y() << "\n z = " << des_orn_.z() << RTT::endlog();

            PRELOG(Error) << "cur_orn_:\n" << " w = " << cur_orn_.w() << "\n x = " << cur_orn_.x() << "\n y = " << cur_orn_.y() << "\n z = " << cur_orn_.z() << RTT::endlog();

            if (angle > 3.14159)
            {
                angle = 3.14159 - angle;
            }

            // PRELOG(Error) << "Dist (rad) " << angle << RTT::endlog();
            PRELOG(Error) << "Dist (deg) " << angle * 180.0 / M_PI << RTT::endlog();

            // move_speed_orn = 0.001 / fabs(angle * 180.0 / M_PI);
            // 180 grad can be rot in 5 sec
            move_speed_orn = 180.0/move_speed_rot_max*0.001/fabs(angle * 180.0 / M_PI);
 
            // PRELOG(Error) << "Speed ROT " << move_speed_orn << RTT::endlog();


            cur_trans_(0) = in_pose_var.position.x;
            cur_trans_(1) = in_pose_var.position.y;
            cur_trans_(2) = in_pose_var.position.z;
            //
            cur_orn_.w() = in_pose_var.orientation.w;
            cur_orn_.x() = in_pose_var.orientation.x;
            cur_orn_.y() = in_pose_var.orientation.y;
            cur_orn_.z() = in_pose_var.orientation.z;


            move_speed_trans = 1.0/move_speed_trans_max*0.001/(des_trans_ - cur_trans_).norm();
            // PRELOG(Error) << "Speed TRANS " << move_speed_trans << RTT::endlog();
            

            des_orn_.normalize();
            cur_orn_.normalize();

            out_orn_ = cur_orn_;
        }

        // 1) Calculate Error and Normalized Direction
        // out_trans_ = cur_trans_ + (des_trans_ - cur_trans_).normalized() * move_speed_trans;
        move_time_ += move_speed_trans;
        if (move_time_ >= 1.0)
        {
            move_time_ = 1.0;
        }
        out_trans_ = cur_trans_ + (des_trans_ - cur_trans_) * move_time_;

        // 2) Slerp
        slerp_time_ += move_speed_orn;
        if (slerp_time_ >= 1.0)
        {
            slerp_time_ = 1.0;
        }
        out_orn_ = cur_orn_.slerp(slerp_time_, des_orn_);
        out_orn_.normalize();

        // out_orn_ = des_orn_;
        // out_orn_.normalize();

        // PRELOG(Error) << "move_time_ " << move_time_ << RTT::endlog();
        // PRELOG(Error) << "slerp_time_ " << slerp_time_ << RTT::endlog();

        // if ((des_trans_ - out_trans_).norm() <= converged_th)
        if (move_time_ >= 1.0)
        {
            out_desiredTaskSpace_var.transforms[0].translation.x = des_trans_(0);
            out_desiredTaskSpace_var.transforms[0].translation.y = des_trans_(1);
            out_desiredTaskSpace_var.transforms[0].translation.z = des_trans_(2);
            out_desiredTaskSpace_var.transforms[0].rotation.w = out_orn_.w();
            out_desiredTaskSpace_var.transforms[0].rotation.x = out_orn_.x();
            out_desiredTaskSpace_var.transforms[0].rotation.y = out_orn_.y();
            out_desiredTaskSpace_var.transforms[0].rotation.z = out_orn_.z();

            if (slerp_time_ >= 1.0)
            {
                out_desiredTaskSpace_var.transforms[0].rotation.w = des_orn_.w();
                out_desiredTaskSpace_var.transforms[0].rotation.x = des_orn_.x();
                out_desiredTaskSpace_var.transforms[0].rotation.y = des_orn_.y();
                out_desiredTaskSpace_var.transforms[0].rotation.z = des_orn_.z();
                PRELOG(Error) << "Converged!" << RTT::endlog();

                PRELOG(Error) << "cur_orn_: CONV \n" << " w = " << cur_orn_.w() << "\n x = " << cur_orn_.x() << "\n y = " << cur_orn_.y() << "\n z = " << cur_orn_.z() << RTT::endlog();
                cur_orn_ = des_orn_;

                // publish converged
                out_converged_var.data = true;
                out_converged_port.write(out_converged_var);

                skill_type = 0;

                cv.notify_one();
            }
        }
        else
        {
            out_desiredTaskSpace_var.transforms[0].translation.x = out_trans_(0);
            out_desiredTaskSpace_var.transforms[0].translation.y = out_trans_(1);
            out_desiredTaskSpace_var.transforms[0].translation.z = out_trans_(2);
            out_desiredTaskSpace_var.transforms[0].rotation.w = out_orn_.w();
            out_desiredTaskSpace_var.transforms[0].rotation.x = out_orn_.x();
            out_desiredTaskSpace_var.transforms[0].rotation.y = out_orn_.y();
            out_desiredTaskSpace_var.transforms[0].rotation.z = out_orn_.z();
        }

        // publish command
        out_desiredTaskSpace_port.write(out_desiredTaskSpace_var);
    }
    else if (skill_type == 2) // ASSEMBLY
    {
        // TODO) Wait for skill_type == 1 to complete?
        

        // 1) 
        
    }

    
}

void ContactSituationSwitcher::stopHook()
{
}

void ContactSituationSwitcher::cleanupHook()
{
    // map_monitors_wrench.clear();
}

//this macro should appear only once per library
// ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::coordination::ContactSituationSwitcher)


// https://docs.orocos.org/rtt/orocos-rtt-scripting.html#orocos-state-descriptions
// https://docs.orocos.org/rtt/orocos-rtt-scripting.html#program-syntax
// https://docs.orocos.org/rtt/tutorials/hello_6.html#writing-a-program