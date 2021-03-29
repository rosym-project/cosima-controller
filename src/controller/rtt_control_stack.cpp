#include "../../include/cosima-controller/controller/rtt_control_stack.hpp"

#include <rtt/Component.hpp> // needed for the macro at the end of this file

using namespace cosima;
using namespace controller;

RTTControlStack::RTTControlStack(std::string const & name) : RTT::TaskContext(name)
{
    schedule_setting_commanded_pose_feedback = false;

    model_configured = false;

    notify_update_contacts_from_m = false;
    notify_update_contacts_from_f = false;

    finished_update_gains = true;
    gains_update_duration = 4.0;
    gains_update_time = 0.0;
    triggered_update_gains = false;
    command_gains_update_duration = gains_update_duration;
    addProperty("gains_update_duration", gains_update_duration);
    //
    finished_update_force = true;
    force_update_duration = 4.0;
    force_update_time = 0.0;
    triggered_update_force = false;
    command_force_update_duration = force_update_duration;
    addProperty("force_update_duration", force_update_duration);

    finished_update_contacts = true;
    // contacts_update_duration = 1.0;
    // contacts_update_time = 0.0;
    triggered_update_contacts = false;
    command_update_contacts_duration = force_update_duration;
    addProperty("force_update_duration", force_update_duration);

    // TODO: make the following automatic rather than hard-coded
    out_torques_data.setZero(7);
    q.setZero(7);
    qd.setZero(7);
    qdd.setZero(7);
    tau.setZero(7);
    out_torques_port.setName("out_torques_port");
    out_torques_port.setDataSample(out_torques_data);
    ports()->addPort(out_torques_port);

    in_coriolisAndGravity_data.setZero(7);
    in_coriolisAndGravity_port.setName("in_coriolisAndGravity_port");
    ports()->addPort(in_coriolisAndGravity_port);


    in_robotstatus_flow = RTT::NoData;
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_data.position.resize(7);
    in_robotstatus_data.velocity.resize(7);
    in_robotstatus_data.effort.resize(7);
    ports()->addPort(in_robotstatus_port);

    kv = 0.0;
    addProperty("gravCompDamp", kv);

    init_ports();

    // addProperty("model_config_path", path_to_model_config);
    addOperation("loadConfig", &RTTControlStack::load_config, this, RTT::ClientThread);
    addOperation("printinfo", &RTTControlStack::printInfo, this, RTT::ClientThread);
    // TODO: initilalize robot model for xbot

    ff_out_data = Eigen::VectorXd::Zero(6);
    t_ff_out_data = Eigen::VectorXd::Zero(6);
    s_ff_out_data = Eigen::VectorXd::Zero(6);
    e_ff_out_data = Eigen::VectorXd::Zero(6);
    addOperation("setFF", &RTTControlStack::setFF, this, RTT::ClientThread);
    addOperation("setFFRot", &RTTControlStack::setFFRot, this, RTT::ClientThread);

    s_cart_stiff_out_data = Eigen::MatrixXd::Identity(6, 6) * 100;
    e_cart_stiff_out_data = Eigen::MatrixXd::Zero(6, 6);
    t_cart_stiff_out_data = Eigen::MatrixXd::Identity(6, 6) * 100;

    s_cart_damp_out_data = Eigen::MatrixXd::Identity(6, 6) * 1;
    e_cart_damp_out_data = Eigen::MatrixXd::Zero(6, 6);
    t_cart_damp_out_data = Eigen::MatrixXd::Identity(6, 6) * 1;

    backup_cart_stiff_out_data = Eigen::MatrixXd::Identity(6, 6) * 100;
    backup_cart_damp_out_data = Eigen::MatrixXd::Identity(6, 6) * 1;

    // Target directions for contact constraint
    t_dir_data = Eigen::VectorXd::Zero(6);


    cart_stiff_out_data = Eigen::MatrixXd::Identity(6, 6) * 100;
    addOperation("setCartStiffness", &RTTControlStack::setCartStiffness, this, RTT::ClientThread);
    cart_damp_out_data   = Eigen::MatrixXd::Identity(6, 6) * 1;
    addOperation("setCartDamping", &RTTControlStack::setCartDamping, this, RTT::ClientThread);
    //
    jnt_stiff_out_data   = Eigen::MatrixXd::Identity(7,7) * 5.0;
    addOperation("setJointStiffness", &RTTControlStack::setJointStiffness, this, RTT::ClientThread);
    jnt_damp_out_data    = Eigen::MatrixXd::Identity(7,7) * 1.0;
    addOperation("setJointDamping", &RTTControlStack::setJointDamping, this, RTT::ClientThread);
    //
    des_posture_out_data = Eigen::VectorXd::Zero(7);
    addOperation("setJntPosture", &RTTControlStack::setJntPosture, this, RTT::ClientThread);

    // Deprecated
    addOperation("setJointStiffnessE", &RTTControlStack::setJointStiffnessE, this, RTT::ClientThread);
    addOperation("setJointDampingE", &RTTControlStack::setJointDampingE, this, RTT::ClientThread);
    addOperation("setCartStiffnessE", &RTTControlStack::setCartStiffnessE, this, RTT::ClientThread);
    addOperation("setCartDampingE", &RTTControlStack::setCartDampingE, this, RTT::ClientThread);

    // Use this instead!
    addOperation("setMassSpringDamper", &RTTControlStack::setMassSpringDamper, this, RTT::ClientThread);
    addOperation("setContactConstraintForce", &RTTControlStack::setContactConstraintForce, this, RTT::ClientThread);

    addOperation("updateContactSituationBlocking", &RTTControlStack::updateContactSituationBlocking, this, RTT::ClientThread);
    addOperation("updateContactSituation", &RTTControlStack::updateContactSituation, this, RTT::ClientThread);

    addOperation("setFFVec", &RTTControlStack::setFFVec, this, RTT::ClientThread);

    addOperation("updatePose", &RTTControlStack::updatePose, this, RTT::ClientThread);

    pose_out_data = Eigen::MatrixXd::Identity(4,4);
    rotation = Eigen::Quaterniond();
}

bool RTTControlStack::configureHook()
{

    RTT::log(RTT::Warning) <<"Enable check_ports_connectivity in the source code..."<<RTT::endlog();
    // TODO if (check_ports_connectivity() && model_configured) {
    if (model_configured) {
        // qp_sot instantiation. FIXME: add for other solver and removed the hard-coded part
        // q(0) = 1.5166;
        // q(1) = -0.279104;
        // q(2) = 0.0;
        // q(3) = 1.24357;
        // q(4) = 0.0;
        // q(5) = -1.57083;
        // q(6) = 0.0;

        q(0) = 1.56845;
        q(1) = -0.448372;
        q(2) = -0.070882;
        q(3) = 1.36423;
        q(4) = -0.0121202;
        q(5) = -1.25336;
        q(6) = 0.785342;


        
        this->model->setJointPosition(q);
        this->model->update();
        iHQP_SoT.reset(new qp_problem(model, q));
        return true;
    } else {
        RTT::log(RTT::Error) <<"Some ports are not connected or the model is not configured."<<RTT::endlog();
        return false;
    }
}

bool RTTControlStack::startHook()
{
    first_no_command = true;

    RTT::log(RTT::Info) <<"Starting..."<<RTT::endlog();
    return true;
}

void RTTControlStack::updateHook()
{
    /*----------------------------input port from robot---------------------------*/
    in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_data);
    if (in_robotstatus_flow == RTT::NoData)
    {
        return;
    }

    in_coriolisAndGravity_port.read(in_coriolisAndGravity_data);

    for (unsigned int i = 0; i < 7; i++)
    {
        q[i] = in_robotstatus_data.position[i];
        qd[i] = in_robotstatus_data.velocity[i];
        tau[i] = in_robotstatus_data.effort[i];
    }
    
    in_desiredTaskSpace_flow = in_desiredTaskSpace_port.read(in_desiredTaskSpace_var);

    if (schedule_setting_commanded_pose_feedback)
    {
        schedule_setting_commanded_pose_feedback = false;
        first_no_command = true;
        in_desiredTaskSpace_flow = RTT::NoData;
    }

    this->model->setJointPosition(q);
    this->model->setJointVelocity(qd);
    this->model->update();

    // Handle external pose command

    if (first_no_command && in_desiredTaskSpace_flow == RTT::NoData)
    {
        // If no command received yet and also its the first time, feed the current pose back from the internal model.
        first_no_command = false;
        Eigen::Affine3d affine_tmp;
        this->model->getPose(iHQP_SoT->cart_imped_high->getDistalLink(),affine_tmp);
        
        in_desiredTaskSpace_var.transforms[0].translation.x = (affine_tmp.matrix().block<3,1>(0,3))(0);
        in_desiredTaskSpace_var.transforms[0].translation.y = (affine_tmp.matrix().block<3,1>(0,3))(1);
        in_desiredTaskSpace_var.transforms[0].translation.z = (affine_tmp.matrix().block<3,1>(0,3))(2);
        // pose_out_data.block<3,1>(0,3) = tmp.block<3,1>(0,3);

        rotation = affine_tmp.matrix().block<3,3>(0,0);
        in_desiredTaskSpace_var.transforms[0].rotation.w = rotation.w();
        in_desiredTaskSpace_var.transforms[0].rotation.x = rotation.x();
        in_desiredTaskSpace_var.transforms[0].rotation.y = rotation.y();
        in_desiredTaskSpace_var.transforms[0].rotation.z = rotation.z();
        // pose_out_data.block<3,3>(0,0) = tmp.block<3,3>(0,0);

        in_desiredTaskSpace_flow = RTT::NewData;

        RTT::log(RTT::Error) <<"No command received, taking: "
            << "\n x: " << in_desiredTaskSpace_var.transforms[0].translation.x
            << "\n y: " << in_desiredTaskSpace_var.transforms[0].translation.y
            << "\n z: " << in_desiredTaskSpace_var.transforms[0].translation.z
            << "\n w: " << in_desiredTaskSpace_var.transforms[0].rotation.w
            << "\n x: " << in_desiredTaskSpace_var.transforms[0].rotation.x
            << "\n y: " << in_desiredTaskSpace_var.transforms[0].rotation.y
            << "\n z: " << in_desiredTaskSpace_var.transforms[0].rotation.z
            << RTT::endlog();
    }

    if (in_desiredTaskSpace_flow == RTT::NewData)
    {
        pose_out_data.block<3,1>(0,3) << in_desiredTaskSpace_var.transforms[0].translation.x,in_desiredTaskSpace_var.transforms[0].translation.y,in_desiredTaskSpace_var.transforms[0].translation.z;

        rotation.vec() << in_desiredTaskSpace_var.transforms[0].rotation.x,in_desiredTaskSpace_var.transforms[0].rotation.y,in_desiredTaskSpace_var.transforms[0].rotation.z;
        rotation.w() = in_desiredTaskSpace_var.transforms[0].rotation.w;
        pose_out_data.block<3,3>(0,0) = rotation.toRotationMatrix();
    }

    /////////////////////////////////////////////////
    // Update contact constraints
    if (triggered_update_contacts)
    {
        triggered_update_contacts = false;
        // stiff & damp
        s_cart_stiff_out_data = cart_stiff_out_data;
        s_cart_damp_out_data = cart_damp_out_data;
        // 
        s_ff_out_data = ff_out_data;

        for (unsigned int i = 0; i < 6; i++)
        {
            if (t_dir_data(i,i) == 0.0)
            {
                e_cart_stiff_out_data(i,i) = t_cart_stiff_out_data(i,i) - s_cart_stiff_out_data(i,i);
                e_cart_damp_out_data(i,i) = t_cart_damp_out_data(i,i) - s_cart_damp_out_data(i,i);
                e_ff_out_data(i,i) = 0.0 - s_ff_out_data(i,i);
            }
            else
            {
                e_cart_stiff_out_data(i,i) = 0.0 - s_cart_stiff_out_data(i,i);
                e_cart_damp_out_data(i,i) = 0.0 - s_cart_damp_out_data(i,i);

                e_ff_out_data(i,i) = t_ff_out_data(i,i) - s_ff_out_data(i,i);
            }
        }
        // time
        gains_update_duration = command_update_contacts_duration;
        force_update_duration = command_update_contacts_duration;
        // 
        gains_update_time = 0;
        force_update_time = 0;
        finished_update_contacts = false;
        finished_update_gains = false;
        finished_update_force = false;
    }

    /////////////////////////////////////////////////

    // Update target gains
    if (triggered_update_gains)
    {
        triggered_update_gains = false;
        s_cart_stiff_out_data = cart_stiff_out_data;
        s_cart_damp_out_data = cart_damp_out_data;
        gains_update_duration = command_gains_update_duration;

        e_cart_stiff_out_data = t_cart_stiff_out_data - s_cart_stiff_out_data;
        e_cart_damp_out_data = t_cart_damp_out_data - s_cart_damp_out_data;
        gains_update_time = 0;
        finished_update_gains = false;
    }
    
    if (gains_update_duration == 0.0)
    {
        gains_update_time = 1.0;
    }
    else
    {
        gains_update_time += 0.001 /* this->getPeriod() */ / gains_update_duration /* in sec */;
    }

    if ((gains_update_time >= 1.0) && (!finished_update_gains))
    {
        gains_update_time = 1.0;

        cart_stiff_out_data = s_cart_stiff_out_data + e_cart_stiff_out_data * gains_update_time;
        cart_damp_out_data = s_cart_damp_out_data + e_cart_damp_out_data * gains_update_time; 

        RTT::log(RTT::Error) << "Gains converged!" << RTT::endlog();

        finished_update_gains = true;
        cv.notify_one();

        if (!finished_update_contacts)
        {
            notify_update_contacts_from_m = true;
        }
    }
    
    if (!finished_update_gains)
    {
        cart_stiff_out_data = s_cart_stiff_out_data + e_cart_stiff_out_data * gains_update_time;
        cart_damp_out_data = s_cart_damp_out_data + e_cart_damp_out_data * gains_update_time; 

        // RTT::log(RTT::Error) << "cart_stiff_out_data(0,0) = " << cart_stiff_out_data(0,0) << RTT::endlog();
        // RTT::log(RTT::Error) << "> s_cart_stiff_out_data(0,0) = " << s_cart_stiff_out_data(0,0) << RTT::endlog();
        // RTT::log(RTT::Error) << "> gut = " << gains_update_time << RTT::endlog();
        // RTT::log(RTT::Error) << "> e_cart_damp_out_data * gut(0,0) = " << (e_cart_damp_out_data * gains_update_time)(0,0) << RTT::endlog();
        // RTT::log(RTT::Error) << "\n\n" << RTT::endlog();
    }

    // Update target force
    if (triggered_update_force)
    {
        triggered_update_force = false;
        s_ff_out_data = ff_out_data;
        force_update_duration = command_force_update_duration;

        e_ff_out_data = t_ff_out_data - s_ff_out_data;
        force_update_time = 0;
        finished_update_force = false;
    }
    
    if (force_update_duration == 0.0)
    {
        force_update_time = 1.0;
    }
    else
    {
        force_update_time += 0.001 /* this->getPeriod() */ / force_update_duration /* in sec */;
    }

    if ((force_update_time >= 1.0) && (!finished_update_force))
    {
        force_update_time = 1.0;

        ff_out_data = s_ff_out_data + e_ff_out_data * force_update_time;

        finished_update_force = true;
        cv_force.notify_one();

        if (!finished_update_contacts)
        {
            notify_update_contacts_from_f = true;
        }
    }

    if (!finished_update_force)
    {
        ff_out_data = s_ff_out_data + e_ff_out_data * force_update_time; 
    }

    if (notify_update_contacts_from_m && notify_update_contacts_from_f && !finished_update_contacts)
    {
        finished_update_contacts = true;
        notify_update_contacts_from_m = false;
        notify_update_contacts_from_f = false;
        cv_contact.notify_one();
    }
    

    iHQP_SoT->update(ff_out_data,
                     cart_stiff_out_data,
                     cart_damp_out_data,
                     pose_out_data,
                     jnt_stiff_out_data,
                     jnt_damp_out_data,
                     des_posture_out_data);

    iHQP_SoT->stack->update(q);

    if (!iHQP_SoT->qp_problem_solver->solve(out_torques_data)) {
        RTT::log(RTT::Error) << "Solution NOT found." << RTT::endlog();
        out_torques_data.setZero(7);
    }
    
    // out_torques_port.write(out_torques_data + in_coriolisAndGravity_data - kv * qd);
    out_torques_port.write(out_torques_data);
}

void RTTControlStack::stopHook()
{
    RTT::log(RTT::Info) <<"Stopping..."<<RTT::endlog();
}

void RTTControlStack::setJntPosture(int idx, double value)
{
    des_posture_out_data(idx) = value;
}

void RTTControlStack::setMassSpringDamper(const Eigen::VectorXd &KP, const Eigen::VectorXd &KD, double time, bool blocking)
{
    finished_update_gains = true;
    for (unsigned int i = 0; i < 6; i++)
    {
        t_cart_stiff_out_data(i,i) = KP(i);
        t_cart_damp_out_data(i,i) = KD(i);

        backup_cart_stiff_out_data(i,i) = KP(i);
        backup_cart_damp_out_data(i,i) = KD(i);
    }

    command_gains_update_duration = time;

    triggered_update_gains = true;

    if (blocking)
    {
        // Wait until main() sends data
        std::unique_lock<std::mutex> lck(m);
        cv.wait(lck);
    }
}

void RTTControlStack::setContactConstraintForce(const Eigen::VectorXd &dir, const Eigen::VectorXd &force, double time, bool blocking)
{
    finished_update_force = true;

    t_dir_data(0) = dir(0);
    t_dir_data(1) = dir(1);
    t_dir_data(2) = dir(2);
    t_dir_data(3) = dir(3);
    t_dir_data(4) = dir(4);
    t_dir_data(5) = dir(5);

    t_ff_out_data(0) = force(0);
    t_ff_out_data(1) = force(1);
    t_ff_out_data(2) = force(2);
    t_ff_out_data(3) = force(3);
    t_ff_out_data(4) = force(4);
    t_ff_out_data(5) = force(5);

    command_force_update_duration = time;

    triggered_update_force = true;

    if (blocking)
    {
        // Wait until main() sends data
        std::unique_lock<std::mutex> lck_force(m_force);
        cv_force.wait(lck_force);
    }
}

void RTTControlStack::updateContactSituationBlocking(const Eigen::VectorXd &kp, const Eigen::VectorXd &kd, const Eigen::VectorXd &fdir, const Eigen::VectorXd &force, double time)
{
    finished_update_contacts = true;

    t_dir_data(0) = fdir(0);
    t_dir_data(1) = fdir(1);
    t_dir_data(2) = fdir(2);
    t_dir_data(3) = fdir(3);
    t_dir_data(4) = fdir(4);
    t_dir_data(5) = fdir(5);

    t_ff_out_data(0) = force(0);
    t_ff_out_data(1) = force(1);
    t_ff_out_data(2) = force(2);
    t_ff_out_data(3) = force(3);
    t_ff_out_data(4) = force(4);
    t_ff_out_data(5) = force(5);

    for (unsigned int i = 0; i < 6; i++)
    {
        t_cart_stiff_out_data(i,i) = kp(i);
        t_cart_damp_out_data(i,i) = kd(i);
    }

    command_update_contacts_duration = time;

    triggered_update_contacts = true;

    // if (blocking)
    // Wait until main() sends data
    std::unique_lock<std::mutex> lck_contact(m_contact);
    cv_contact.wait(lck_contact);
}

void RTTControlStack::updateContactSituation(const Eigen::VectorXd &kp, const Eigen::VectorXd &kd, const Eigen::VectorXd &fdir, const Eigen::VectorXd &force, double time)
{
    finished_update_contacts = true;

    t_dir_data(0) = fdir(0);
    t_dir_data(1) = fdir(1);
    t_dir_data(2) = fdir(2);
    t_dir_data(3) = fdir(3);
    t_dir_data(4) = fdir(4);
    t_dir_data(5) = fdir(5);

    t_ff_out_data(0) = force(0);
    t_ff_out_data(1) = force(1);
    t_ff_out_data(2) = force(2);
    t_ff_out_data(3) = force(3);
    t_ff_out_data(4) = force(4);
    t_ff_out_data(5) = force(5);

    for (unsigned int i = 0; i < 6; i++)
    {
        t_cart_stiff_out_data(i,i) = kp(i);
        t_cart_damp_out_data(i,i) = kd(i);
    }

    command_update_contacts_duration = time;

    triggered_update_contacts = true;
}

void RTTControlStack::setCartStiffness(double KP)
{
    // cart_stiff_out_data = Eigen::MatrixXd::Identity(6, 6) * KP;
    finished_update_gains = true;

    cart_stiff_out_data(0,0) = KP;
    cart_stiff_out_data(1,1) = KP;
    cart_stiff_out_data(2,2) = KP;

    cart_stiff_out_data(3,3) = KP * 0.1;
    cart_stiff_out_data(4,4) = KP * 0.1;
    cart_stiff_out_data(5,5) = KP * 0.1;
}

void RTTControlStack::setCartStiffnessE(const Eigen::VectorXd &KP)
{
    // cart_stiff_out_data = Eigen::MatrixXd::Identity(6, 6);
    finished_update_gains = true;
    for (unsigned int i = 0; i < 6; i++)
    {
        cart_stiff_out_data(i,i) = KP(i);
    }
}

void RTTControlStack::setCartDamping(double KD)
{
    // cart_damp_out_data = Eigen::MatrixXd::Identity(6, 6) * KD;
    finished_update_gains = true;
    cart_damp_out_data(0,0) = KD;
    cart_damp_out_data(1,1) = KD;
    cart_damp_out_data(2,2) = KD;

    cart_damp_out_data(3,3) = KD * 0.1;
    cart_damp_out_data(4,4) = KD * 0.1;
    cart_damp_out_data(5,5) = KD * 0.1;
}

void RTTControlStack::setCartDampingE(const Eigen::VectorXd &KD)
{
    // cart_damp_out_data = Eigen::MatrixXd::Identity(6, 6);
    finished_update_gains = true;
    for (unsigned int i = 0; i < 6; i++)
    {
        cart_damp_out_data(i,i) = KD(i);
    }
}

void RTTControlStack::setJointStiffness(double KP)
{
    jnt_stiff_out_data = Eigen::MatrixXd::Identity(7, 7) * KP;
}

void RTTControlStack::setJointStiffnessE(const Eigen::VectorXd &KP)
{
    // jnt_stiff_out_data = Eigen::MatrixXd::Identity(7, 7);
    for (unsigned int i = 0; i < 7; i++)
    {
        jnt_stiff_out_data(i,i) = KP(i);
    }
}

void RTTControlStack::setJointDamping(double KD)
{
    jnt_damp_out_data = Eigen::MatrixXd::Identity(7, 7) * KD;
}

void RTTControlStack::setJointDampingE(const Eigen::VectorXd &KD)
{
    // jnt_damp_out_data = Eigen::MatrixXd::Identity(7, 7);
    for (unsigned int i = 0; i < 7; i++)
    {
        jnt_damp_out_data(i,i) = KD(i);
    }
}

void RTTControlStack::setFF(double x, double y, double z)
{
    ff_out_data(0) = x;
    ff_out_data(1) = y;
    ff_out_data(2) = z;
}

void RTTControlStack::setFFVec(const Eigen::VectorXd &force)
{
    ff_out_data(0) = force(0);
    ff_out_data(1) = force(1);
    ff_out_data(2) = force(2);
    ff_out_data(3) = force(3);
    ff_out_data(4) = force(4);
    ff_out_data(5) = force(5);
}

void RTTControlStack::updatePose()
{
    RTT::log(RTT::Warning) <<"Scheduling setting Command Pose to Feedback" << RTT::endlog();
    this->schedule_setting_commanded_pose_feedback = true;
}

void RTTControlStack::setFFRot(double x, double y, double z)
{
    ff_out_data(3) = x;
    ff_out_data(4) = y;
    ff_out_data(5) = z;
}

void RTTControlStack::cleanupHook()
{
    RTT::log(RTT::Info) <<"Cleaning..."<<RTT::endlog();
}

bool RTTControlStack::load_config(std::string config_path)
{
    // Create genContext parameter that decides about the backend
    // which is used in the next few lines for backend configuration
    try {
      model = XBot::ModelInterface::getModel(config_path);
      model_configured = true;
    } catch (const std::exception &exc) {
      RTT::log(RTT::Error) <<"Cannot configure the model: "<<exc.what()<<RTT::endlog();
      model_configured = false;
    }
    return model_configured;
}

void RTTControlStack::init_ports()
{
    /*-----------------------cart_imped_high_cartesian_pose-----------------------*/
    in_desiredTaskSpace_var = trajectory_msgs::MultiDOFJointTrajectoryPoint();
    in_desiredTaskSpace_var.transforms.push_back(geometry_msgs::Transform());
    //
    // // in_desiredTaskSpace_var.transforms[0].translation.x = -0.0287248;
    // // in_desiredTaskSpace_var.transforms[0].translation.y = -0.529656 ;
    // // in_desiredTaskSpace_var.transforms[0].translation.z = 0.467355  ;
    // // in_desiredTaskSpace_var.transforms[0].rotation.w = 0.0174455 ;
    // // in_desiredTaskSpace_var.transforms[0].rotation.x = 0.687507  ;
    // // in_desiredTaskSpace_var.transforms[0].rotation.y = -0.72578  ;
    // // in_desiredTaskSpace_var.transforms[0].rotation.z = 0.0165435 ;
    // in_desiredTaskSpace_var.transforms[0].translation.x = 0.0;
    // in_desiredTaskSpace_var.transforms[0].translation.y = -0.68835;
    // in_desiredTaskSpace_var.transforms[0].translation.z = 0.26309;
    // // in_desiredTaskSpace_var.transforms[0].rotation.w = 0.0150682 ;
    // // in_desiredTaskSpace_var.transforms[0].rotation.x = 0.707013  ;
    // // in_desiredTaskSpace_var.transforms[0].rotation.y = -0.706876  ;
    // // in_desiredTaskSpace_var.transforms[0].rotation.z = 0.0152298 ;
    // in_desiredTaskSpace_var.transforms[0].rotation.w = 0.0;
    // in_desiredTaskSpace_var.transforms[0].rotation.x = 0.0;
    // in_desiredTaskSpace_var.transforms[0].rotation.y = 1.0;
    // in_desiredTaskSpace_var.transforms[0].rotation.z = 0.0;

    in_desiredTaskSpace_var.transforms[0].translation.x = -0.0287248;
    in_desiredTaskSpace_var.transforms[0].translation.y = -0.529656;
    in_desiredTaskSpace_var.transforms[0].translation.z = 0.467355;
    in_desiredTaskSpace_var.transforms[0].rotation.w = 0.0174455;
    in_desiredTaskSpace_var.transforms[0].rotation.x = 0.687507;
    in_desiredTaskSpace_var.transforms[0].rotation.y = -0.72578;
    in_desiredTaskSpace_var.transforms[0].rotation.z = 0.0165435;



    in_desiredTaskSpace_var.velocities.push_back(geometry_msgs::Twist());
    in_desiredTaskSpace_var.accelerations.push_back(geometry_msgs::Twist());

    in_desiredTaskSpace_port.setName("in_desiredTaskSpace_port");
    in_desiredTaskSpace_flow = RTT::NoData;
    ports()->addPort(in_desiredTaskSpace_port);
}

bool RTTControlStack::check_ports_connectivity()
{
    return true;
}

void RTTControlStack::write_ports()
{
    // cart_imped_high_feedforward_forces_output_port.write(cart_imped_high_feedforward_forces_rosout_data);
    // cart_imped_high_cartesian_pose_output_port.write(cart_imped_high_cartesian_pose_rosout_data);
    // joint_space_redres_desired_joint_output_port.write(joint_space_redres_desired_joint_rosout_data);
}

void RTTControlStack::printInfo()
{
    Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(4,4);
    Eigen::Affine3d affine_tmp;

    iHQP_SoT->cart_imped_high->getActualPose(tmp);
    std::cout<<"cart_imped_high->getActualPose ->\n"<<tmp<<"\n----------------------"<<std::endl;
    std::cout<<"base: "<<iHQP_SoT->cart_imped_high->getBaseLink()<<std::endl;
    std::cout<<"ee: "<<iHQP_SoT->cart_imped_high->getDistalLink()<<std::endl;
    this->model->getPose(iHQP_SoT->cart_imped_high->getDistalLink(),affine_tmp);
    
    std::cout<<"model->getPose ->\n" << affine_tmp.matrix() << "\n----------------------"<<std::endl;
    std::cout<<"q: \n"<<q.transpose()<<"\nqd: \n"<<qd.transpose()<<"\ntau: \n"<<tau.transpose()<<std::endl;
    std::cout<<"DES INPUT: \n"<<in_desiredTaskSpace_var<<std::endl;
    
}

void RTTControlStack::prepare_monitors()
{
    // iHQP_SoT->cart_imped_high->getFeedForwardForces(cart_imped_high_feedforward_forces_output_data);
    // iHQP_SoT->cart_imped_high->getActualPose(cart_imped_high_cartesian_pose_output_data);
    // iHQP_SoT->joint_space_redres->getReference(joint_space_redres_desired_joint_output_data);
    // /*----------------converting eigen to ros via eigen_conversions---------------*/
    // Eigen::Affine3d tmp;
    // tmp.matrix() = cart_imped_high_cartesian_pose_output_data.matrix();
    // tf::poseEigenToMsg(tmp,cart_imped_high_cartesian_pose_rosout_data);
    // tf::matrixEigenToMsg(cart_imped_high_feedforward_forces_output_data,cart_imped_high_feedforward_forces_rosout_data);
    // tf::matrixEigenToMsg(joint_space_redres_desired_joint_output_data,joint_space_redres_desired_joint_rosout_data);
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::controller::RTTControlStack)
