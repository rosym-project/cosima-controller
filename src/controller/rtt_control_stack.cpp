#include "../../include/cosima-controller/controller/rtt_control_stack.hpp"

#include <rtt/Component.hpp> // needed for the macro at the end of this file

using namespace cosima;
using namespace controller;

RTTControlStack::RTTControlStack(std::string const & name) : RTT::TaskContext(name)
{
    model_configured = false;
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

    change_in_sec = 1.0;
    addProperty("change_in_sec", change_in_sec);

    simulation = false;
    addProperty("simulation", simulation);

    rate = 0.0;
    addProperty("rate", rate);

    in_wrench_fdb_data = Eigen::VectorXd::Zero(6);

    // addProperty("stack_type", stack_type);

    init_ports();

    // addProperty("model_config_path", path_to_model_config);
    addOperation("loadConfig", &RTTControlStack::load_config, this, RTT::ClientThread);
    // addOperation("printinfo", &RTTControlStack::printInfo, this, RTT::ClientThread);
    // TODO: initilalize robot model for xbot

    ff_out_data = Eigen::VectorXd::Zero(6);
    // addOperation("setFF", &RTTControlStack::setFF, this, RTT::ClientThread);
    // //
    // cart_stiff_out_data  = Eigen::MatrixXd::Identity(6, 6) * 100;
    // addOperation("setCartStiffness", &RTTControlStack::setCartStiffness, this, RTT::ClientThread);
    // cart_damp_out_data   = Eigen::MatrixXd::Identity(6, 6) * 1;
    // addOperation("setCartDamping", &RTTControlStack::setCartDamping, this, RTT::ClientThread);
    // //
    // jnt_stiff_out_data   = Eigen::MatrixXd::Identity(7,7) * 5.0;
    // addOperation("setJointStiffness", &RTTControlStack::setJointStiffness, this, RTT::ClientThread);
    // jnt_damp_out_data    = Eigen::MatrixXd::Identity(7,7) * 1.0;
    // addOperation("setJointDamping", &RTTControlStack::setJointDamping, this, RTT::ClientThread);
    // //
    des_posture_out_data = Eigen::VectorXd::Zero(7);
    // addOperation("setJntPosture", &RTTControlStack::setJntPosture, this, RTT::ClientThread);

    // addOperation("setJointStiffnessE", &RTTControlStack::setJointStiffnessE, this, RTT::ClientThread);
    // addOperation("setJointDampingE", &RTTControlStack::setJointDampingE, this, RTT::ClientThread);
    // addOperation("setCartStiffnessE", &RTTControlStack::setCartStiffnessE, this, RTT::ClientThread);
    // addOperation("setCartDampingE", &RTTControlStack::setCartDampingE, this, RTT::ClientThread);


    addOperation("addContact", &RTTControlStack::addContact, this, RTT::ClientThread);
    addOperation("addCompliance", &RTTControlStack::addCompliance, this, RTT::ClientThread);
    addOperation("addMSD", &RTTControlStack::addMSD, this, RTT::ClientThread);
    addOperation("addMSDdir", &RTTControlStack::addMSDdir, this, RTT::ClientThread);
    addOperation("addJSMsd", &RTTControlStack::addJSMsd, this, RTT::ClientThread);
    addOperation("useTransitionWrench", &RTTControlStack::useTransitionWrench, this, RTT::ClientThread);
    addOperation("addCS", &RTTControlStack::addCS, this, RTT::ClientThread);



    addProperty("cur_model_msd_gain_stiffness", cur_model_msd_gain_stiffness);
    addProperty("cur_model_msd_gain_damping", cur_model_msd_gain_damping);
    addProperty("cur_model_js_gain_stiffness", cur_model_js_gain_stiffness);
    addProperty("cur_model_js_gain_damping", cur_model_js_gain_damping);
}

bool RTTControlStack::configureHook()
{
    RTT::log(RTT::Warning) <<"Enable check_ports_connectivity in the source code..."<<RTT::endlog();
    if (model_configured) {
        // qp_sot instantiation. FIXME: add for other solver and removed the hard-coded part
        q(0) = 1.5166;
        q(1) = -0.279104;
        q(2) = 0.0;
        q(3) = 1.24357;
        q(4) = 0.0;
        q(5) = -1.57083;
        q(6) = 0.0;
        
        this->model->setJointPosition(q);
        this->model->update();

        // Update all CS
        this->old_cs = NULL;
        this->current_cs = NULL;

        for (auto const& x : model_css)
        {
            if (!current_cs)
            {
                RTT::log(RTT::Error) << "Choosing CS: " << x.first << RTT::endlog();
                current_cs = x.second;
            }
            x.second->configure(model, q);
        }

        // Get parameters from current CS
        cur_model_msd_gain_stiffness = this->current_cs->model_msd_gain_stiffness;
        cur_model_msd_gain_damping = this->current_cs->model_msd_gain_damping;
        cur_model_js_gain_stiffness = this->current_cs->model_js_gain_stiffness;
        cur_model_js_gain_damping = this->current_cs->model_js_gain_damping;

        old_msd_gain_stiffness = cur_model_msd_gain_stiffness;
        old_msd_gain_damping = cur_model_msd_gain_damping;
        old_js_gain_stiffness = cur_model_js_gain_stiffness;
        old_js_gain_damping = cur_model_js_gain_damping;

        return true;
    } else {
        RTT::log(RTT::Error) <<"Some ports are not connected or the model noot configured."<<RTT::endlog();
        return false;
    }
}

bool RTTControlStack::startHook()
{
    
	// cart_imped_high_feedforward_forces_output_port.createStream(rtt_roscomm::topic("/cart_imped_high/feedforward_forces"));
	// RTT::log(RTT::Info) <<"Created stream forcart_imped_high_feedforward_forces over the topic: /cart_imped_high/feedforward_forces..."<<RTT::endlog();

    
	// cart_imped_high_cartesian_pose_output_port.createStream(rtt_roscomm::topic("/cart_imped_high/cartesian_pose"));
	// RTT::log(RTT::Info) <<"Created stream forcart_imped_high_cartesian_pose over the topic: /cart_imped_high/cartesian_pose..."<<RTT::endlog();

    
	// joint_space_redres_desired_joint_output_port.createStream(rtt_roscomm::topic("/joint_space_redres/desired_joint"));
	// RTT::log(RTT::Info) <<"Created stream forjoint_space_redres_desired_joint over the topic: /joint_space_redres/desired_joint..."<<RTT::endlog();
    return true;
}

void RTTControlStack::updateHook()
{
    // read ports
    in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_data);
    if (in_robotstatus_flow == RTT::NoData)
    {
        return;
    }


    in_coriolisAndGravity_port.read(in_coriolisAndGravity_data);
    for(int i=0; i<7; ++i){
        q[i] = in_robotstatus_data.position[i];
        qd[i] = in_robotstatus_data.velocity[i];
        tau[i] = in_robotstatus_data.effort[i];
    }

    
    in_desiredTaskSpace_flow = in_desiredTaskSpace_port.read(in_desiredTaskSpace_var);
    // if (in_desiredTaskSpace_flow == RTT::NoData)
    // {
    //     return;
    // }

    in_desiredJointSpace_flow = in_desiredJointSpace_port.read(in_desiredJointSpace_var);
    in_contactWrench_flow = in_contactWrench_port.read(in_contactWrench_var);


    // update model
    this->model->setJointPosition(q);
	this->model->setJointVelocity(qd);
	// this->model->setJointAcceleration(qdd);
	// this->model->setJointEffort(tau);
	this->model->update();


    // set references
    Eigen::MatrixXd pose_out_data = Eigen::MatrixXd::Identity(4,4);
    pose_out_data.block<3,1>(0,3) << in_desiredTaskSpace_var.transforms[0].translation.x,in_desiredTaskSpace_var.transforms[0].translation.y,in_desiredTaskSpace_var.transforms[0].translation.z;

    Eigen::Quaterniond rotation;
    rotation.vec() << in_desiredTaskSpace_var.transforms[0].rotation.x,in_desiredTaskSpace_var.transforms[0].rotation.y,in_desiredTaskSpace_var.transforms[0].rotation.z;
    rotation.w() = in_desiredTaskSpace_var.transforms[0].rotation.w;
    pose_out_data.block<3,3>(0,0) = rotation.toRotationMatrix();

    // if (in_contactWrench_flow == RTT::NewData)
    // {
    //     ff_out_data(0) = in_contactWrench_var.force.x;
    //     ff_out_data(1) = in_contactWrench_var.force.y;
    //     ff_out_data(2) = in_contactWrench_var.force.z;
    //     ff_out_data(3) = in_contactWrench_var.torque.x;
    //     ff_out_data(4) = in_contactWrench_var.torque.y;
    //     ff_out_data(5) = in_contactWrench_var.torque.z;
    // }

    // if (in_desiredJointSpace_flow == RTT::NewData)
    // {
    //     for (unsigned int i = 0; i < 7; i++)
    //     {
    //         des_posture_out_data(i) = in_desiredJointSpace_var.positions[i];
    //     }
    // }

    // // Evaluate Transitions: FCFS Approach
    // in_wrench_fdb_flow = in_wrench_fdb_port.read(in_wrench_fdb_var);
    // if (in_wrench_fdb_flow == RTT::NewData)
    // {
    //     in_wrench_fdb_data(0) = in_wrench_fdb_var.force.x;
    //     in_wrench_fdb_data(1) = in_wrench_fdb_var.force.y;
    //     in_wrench_fdb_data(2) = in_wrench_fdb_var.force.z;
    //     in_wrench_fdb_data(3) = in_wrench_fdb_var.torque.x;
    //     in_wrench_fdb_data(4) = in_wrench_fdb_var.torque.y;
    //     in_wrench_fdb_data(5) = in_wrench_fdb_var.torque.z;

    //     for (unsigned int i = 0; i < this->current_cs->transitions.size(); i++)
    //     {
    //         if (this->current_cs->transitions[i]->eval(in_wrench_fdb_data)) // TODO DLW
    //         {
    //             old_msd_gain_stiffness = cur_model_msd_gain_stiffness;
    //             old_msd_gain_damping = cur_model_msd_gain_damping;
    //             old_js_gain_stiffness = cur_model_js_gain_stiffness;
    //             old_js_gain_damping = cur_model_js_gain_damping;
    //             // this->old_cs
    //             this->current_cs = this->model_css[this->current_cs->transitions[i]->t_target];

    //             rate = 0.0;
    //             break;
    //         }
    //     }
    // }

    rate += change_in_sec * this->getPeriod();
    if (rate >= 1.0)
    {
        rate = 1.0;
    }
    rate = 1.0; // TODO
    cur_model_msd_gain_stiffness = old_msd_gain_stiffness + (this->current_cs->model_msd_gain_stiffness - old_msd_gain_stiffness) * rate;
    cur_model_msd_gain_damping = old_msd_gain_damping + (this->current_cs->model_msd_gain_damping - old_msd_gain_damping) * rate;
    cur_model_js_gain_stiffness = old_js_gain_stiffness + (this->current_cs->model_js_gain_stiffness - old_js_gain_stiffness) * rate;
    cur_model_js_gain_damping = old_js_gain_damping + (this->current_cs->model_js_gain_damping - old_js_gain_damping) * rate;


    this->current_cs->update(ff_out_data,
                            pose_out_data,
                            des_posture_out_data,
                            cur_model_msd_gain_stiffness,
                            cur_model_msd_gain_damping,
                            cur_model_js_gain_stiffness,
                            cur_model_js_gain_damping);
    this->current_cs->stack->update(q); // TODO

    if (!this->current_cs->qp_problem_solver->solve(out_torques_data)) {
        RTT::log(RTT::Warning) <<"Solution NOT found."<<RTT::endlog();
        // Below is better to send previous solution instead of zero
        // or something that does not constitute a hidden assumption
        out_torques_data.setZero(7);
    }
    if (this->simulation)
    {
        out_torques_port.write(out_torques_data + in_coriolisAndGravity_data - kv * qd);
    }
    else
    {
        out_torques_port.write(out_torques_data);
    }
}

void RTTControlStack::stopHook()
{
    RTT::log(RTT::Info) <<"Stopping..."<<RTT::endlog();
}

// void RTTControlStack::setJntPosture(int idx, double value)
// {
//     des_posture_out_data(idx) = value;
// }

// void RTTControlStack::setCartStiffness(double KP)
// {
//     // cart_stiff_out_data = Eigen::MatrixXd::Identity(6, 6) * KP;

//     cart_stiff_out_data(0,0) = KP;
//     cart_stiff_out_data(1,1) = KP;
//     cart_stiff_out_data(2,2) = KP;

//     cart_stiff_out_data(3,3) = KP * 0.1;
//     cart_stiff_out_data(4,4) = KP * 0.1;
//     cart_stiff_out_data(5,5) = KP * 0.1;
// }

// void RTTControlStack::setCartStiffnessE(const Eigen::VectorXd &KP)
// {
//     // cart_stiff_out_data = Eigen::MatrixXd::Identity(6, 6);
//     for (unsigned int i = 0; i < 6; i++)
//     {
//         cart_stiff_out_data(i,i) = KP(i);
//     }
// }

// void RTTControlStack::setCartDamping(double KD)
// {
//     // cart_damp_out_data = Eigen::MatrixXd::Identity(6, 6) * KD;
//     cart_damp_out_data(0,0) = KD;
//     cart_damp_out_data(1,1) = KD;
//     cart_damp_out_data(2,2) = KD;

//     cart_damp_out_data(3,3) = KD * 0.1;
//     cart_damp_out_data(4,4) = KD * 0.1;
//     cart_damp_out_data(5,5) = KD * 0.1;
// }

// void RTTControlStack::setCartDampingE(const Eigen::VectorXd &KD)
// {
//     // cart_damp_out_data = Eigen::MatrixXd::Identity(6, 6);
//     for (unsigned int i = 0; i < 6; i++)
//     {
//         cart_damp_out_data(i,i) = KD(i);
//     }
// }

// void RTTControlStack::setJointStiffness(double KP)
// {
//     jnt_stiff_out_data = Eigen::MatrixXd::Identity(7, 7) * KP;
// }

// void RTTControlStack::setJointStiffnessE(const Eigen::VectorXd &KP)
// {
//     // jnt_stiff_out_data = Eigen::MatrixXd::Identity(7, 7);
//     for (unsigned int i = 0; i < 7; i++)
//     {
//         jnt_stiff_out_data(i,i) = KP(i);
//     }
// }

// void RTTControlStack::setJointDamping(double KD)
// {
//     jnt_damp_out_data = Eigen::MatrixXd::Identity(7, 7) * KD;
// }

// void RTTControlStack::setJointDampingE(const Eigen::VectorXd &KD)
// {
//     // jnt_damp_out_data = Eigen::MatrixXd::Identity(7, 7);
//     for (unsigned int i = 0; i < 7; i++)
//     {
//         jnt_damp_out_data(i,i) = KD(i);
//     }
// }

// void RTTControlStack::setFF(double x, double y, double z)
// {
//     ff_out_data(0) = x;
//     ff_out_data(1) = y;
//     ff_out_data(2) = z;
// }

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
    in_desiredTaskSpace_var.transforms[0].translation.x = 0.000158222;
    in_desiredTaskSpace_var.transforms[0].translation.y = -0.675439 ;
    in_desiredTaskSpace_var.transforms[0].translation.z = 0.285982  ;
    in_desiredTaskSpace_var.transforms[0].rotation.w = 0.0150682 ;
    in_desiredTaskSpace_var.transforms[0].rotation.x = 0.707013  ;
    in_desiredTaskSpace_var.transforms[0].rotation.y = -0.706876  ;
    in_desiredTaskSpace_var.transforms[0].rotation.z = 0.0152298 ;

    for (unsigned int j = 0; j < 7; j++)
    {
        in_desiredJointSpace_var.positions.push_back(0.0);
        in_desiredJointSpace_var.velocities.push_back(0.0);
        in_desiredJointSpace_var.accelerations.push_back(0.0);
        in_desiredJointSpace_var.effort.push_back(0.0);
    }

    in_contactWrench_var.force.x = 0.0;
    in_contactWrench_var.force.y = 0.0;
    in_contactWrench_var.force.z = 0.0;
    in_contactWrench_var.torque.x = 0.0;
    in_contactWrench_var.torque.y = 0.0;
    in_contactWrench_var.torque.z = 0.0;

    in_wrench_fdb_var.force.x = 0.0;
    in_wrench_fdb_var.force.y = 0.0;
    in_wrench_fdb_var.force.z = 0.0;
    in_wrench_fdb_var.torque.x = 0.0;
    in_wrench_fdb_var.torque.y = 0.0;
    in_wrench_fdb_var.torque.z = 0.0;

    in_desiredTaskSpace_var.velocities.push_back(geometry_msgs::Twist());
    in_desiredTaskSpace_var.accelerations.push_back(geometry_msgs::Twist());

    in_desiredTaskSpace_port.setName("in_desiredTaskSpace_port");
    in_desiredTaskSpace_flow = RTT::NoData;
    ports()->addPort(in_desiredTaskSpace_port);

    in_desiredJointSpace_port.setName("in_desiredJointSpace_port");
    in_desiredJointSpace_flow = RTT::NoData;
    ports()->addPort(in_desiredJointSpace_port);

    in_contactWrench_port.setName("in_contactWrench_port");
    in_contactWrench_flow = RTT::NoData;
    ports()->addPort(in_contactWrench_port);

    in_wrench_fdb_port.setName("in_wrench_fdb_port");
    in_wrench_fdb_flow = RTT::NoData;
    ports()->addPort(in_wrench_fdb_port);
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

// void RTTControlStack::printInfo()
// {
//     Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(4,4);
//     Eigen::Affine3d affine_tmp;
//     if (stack_type == 0)
//     {
//         iHQP_SoT->cart_imped_high->getActualPose(tmp);
//         std::cout<<"cart_imped_high->getActualPose ->\n"<<tmp<<"\n----------------------"<<std::endl;
//         std::cout<<"base: "<<iHQP_SoT->cart_imped_high->getBaseLink()<<std::endl;
//         std::cout<<"ee: "<<iHQP_SoT->cart_imped_high->getDistalLink()<<std::endl;
//         this->model->getPose(iHQP_SoT->cart_imped_high->getDistalLink(),affine_tmp);
//     }
//     else if (stack_type == 1)
//     {
//         iHQP_SoT_cart_wo_js->cart_imped_high->getActualPose(tmp);
//         std::cout<<"cart_imped_high->getActualPose ->\n"<<tmp<<"\n----------------------"<<std::endl;
//         std::cout<<"base: "<<iHQP_SoT_cart_wo_js->cart_imped_high->getBaseLink()<<std::endl;
//         std::cout<<"ee: "<<iHQP_SoT_cart_wo_js->cart_imped_high->getDistalLink()<<std::endl;
//         this->model->getPose(iHQP_SoT_cart_wo_js->cart_imped_high->getDistalLink(),affine_tmp);
//     }
    
//     std::cout<<"model->getPose ->\n" << affine_tmp.matrix() << "\n----------------------"<<std::endl;
//     std::cout<<"q: \n"<<q.transpose()<<"\nqd: \n"<<qd.transpose()<<"\ntau: \n"<<tau.transpose()<<std::endl;
//     std::cout<<"DES INPUT: \n"<<in_desiredTaskSpace_var<<std::endl;
    
// }

// void RTTControlStack::prepare_monitors()
// {
//     // iHQP_SoT->cart_imped_high->getFeedForwardForces(cart_imped_high_feedforward_forces_output_data);
//     // iHQP_SoT->cart_imped_high->getActualPose(cart_imped_high_cartesian_pose_output_data);
//     // iHQP_SoT->joint_space_redres->getReference(joint_space_redres_desired_joint_output_data);
//     // /*----------------converting eigen to ros via eigen_conversions---------------*/
//     // Eigen::Affine3d tmp;
//     // tmp.matrix() = cart_imped_high_cartesian_pose_output_data.matrix();
//     // tf::poseEigenToMsg(tmp,cart_imped_high_cartesian_pose_rosout_data);
//     // tf::matrixEigenToMsg(cart_imped_high_feedforward_forces_output_data,cart_imped_high_feedforward_forces_rosout_data);
//     // tf::matrixEigenToMsg(joint_space_redres_desired_joint_output_data,joint_space_redres_desired_joint_rosout_data);
// }

////////////// MODEL INPUT //////////////
void RTTControlStack::addContact(const std::string &cs, const unsigned int &dir)
{
    if ( model_css.find(cs) == model_css.end())
    {
        RTT::log(RTT::Error) << "CS name not found" << RTT::endlog();
        return;
    }
    model_css[cs]->model_contacts(dir) = 1;
}

void RTTControlStack::addCompliance(const std::string &cs, const unsigned int &dir)
{
    if ( model_css.find(cs) == model_css.end())
    {
        RTT::log(RTT::Error) << "CS name not found" << RTT::endlog();
        return;
    }
    model_css[cs]->model_compliance(dir) = 1;
}

void RTTControlStack::addMSD(const std::string &cs, const Eigen::VectorXd &stiffness, const Eigen::VectorXd &damping)
{
    if ( model_css.find(cs) == model_css.end())
    {
        RTT::log(RTT::Error) << "CS name not found" << RTT::endlog();
        return;
    }
    model_css[cs]->model_msd(0) = 1;
    model_css[cs]->model_msd(1) = 1;
    model_css[cs]->model_msd(2) = 1;
    model_css[cs]->model_msd(3) = 1;
    model_css[cs]->model_msd(4) = 1;
    model_css[cs]->model_msd(5) = 1;
    model_css[cs]->model_msd_gain_stiffness = stiffness;
    model_css[cs]->model_msd_gain_damping = damping;
}

void RTTControlStack::addJSMsd(const std::string &cs, const Eigen::VectorXd &stiffness, const Eigen::VectorXd &damping)
{
    if (model_css.find(cs) == model_css.end())
    {
        RTT::log(RTT::Error) << "CS name not found" << RTT::endlog();
        return;
    }
    model_css[cs]->enable_js_task = true;
    model_css[cs]->model_js_gain_stiffness = stiffness;
    model_css[cs]->model_js_gain_damping = damping;
}

void RTTControlStack::addMSDdir(const std::string &cs, const unsigned int &dir, const double &stiffness, const double &damping)
{
    if ( model_css.find(cs) == model_css.end())
    {
        RTT::log(RTT::Error) << "CS name not found" << RTT::endlog();
        return;
    }
    model_css[cs]->model_msd(dir) = 1;
    model_css[cs]->model_msd_gain_stiffness(dir) = stiffness;
    model_css[cs]->model_msd_gain_damping(dir) = damping;
}

void RTTControlStack::addCS(const std::string &cs)
{
    std::shared_ptr<CS> tmp = std::shared_ptr<CS>(new CS(cs));
    model_css[cs] = tmp;
}

void RTTControlStack::useTransitionWrench(const std::string &cs, const std::string &target, const unsigned int &dir, const bool &greater, const double &value)
{
    if ( model_css.find(cs) == model_css.end())
    {
        RTT::log(RTT::Error) << "CS name not found" << RTT::endlog();
        return;
    }
    std::shared_ptr<WrenchTransition> tmp = std::shared_ptr<WrenchTransition>(new WrenchTransition(target, dir, greater, value));
    model_css[cs]->transitions.push_back(tmp);
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::controller::RTTControlStack)
