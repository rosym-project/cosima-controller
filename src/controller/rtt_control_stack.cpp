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

    init_ports();

    // addProperty("model_config_path", path_to_model_config);
    addOperation("loadConfig", &RTTControlStack::load_config, this, RTT::ClientThread);
    addOperation("printinfo", &RTTControlStack::printInfo, this, RTT::ClientThread);
    // TODO: initilalize robot model for xbot

    ff_out_data = Eigen::VectorXd::Zero(6);
    addOperation("setFF", &RTTControlStack::setFF, this, RTT::ClientThread);
    //
    cart_stiff_out_data  = Eigen::MatrixXd::Identity(6, 6) * 100;
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

    addOperation("setJointStiffnessE", &RTTControlStack::setJointStiffnessE, this, RTT::ClientThread);
    addOperation("setJointDampingE", &RTTControlStack::setJointDampingE, this, RTT::ClientThread);
    addOperation("setCartStiffnessE", &RTTControlStack::setCartStiffnessE, this, RTT::ClientThread);
    addOperation("setCartDampingE", &RTTControlStack::setCartDampingE, this, RTT::ClientThread);
}

bool RTTControlStack::configureHook()
{

    RTT::log(RTT::Warning) <<"Enable check_ports_connectivity in the source code..."<<RTT::endlog();
    // TODO if (check_ports_connectivity() && model_configured) {
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
        iHQP_SoT.reset(new qp_problem(model, q));
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

    RTT::log(RTT::Info) <<"Staring..."<<RTT::endlog();
    return true;
}

void RTTControlStack::updateHook()
{
    read_ports();
    update_model();
    set_sot_references();
    iHQP_SoT->stack->update(q); //not sure if passing q make any sense...

    if (!iHQP_SoT->qp_problem_solver->solve(out_torques_data)) {
        RTT::log(RTT::Warning) <<"Solution NOT found."<<RTT::endlog();
        // Below is better to send previous solution instead of zero
        // or something that does not constitute a hidden assumption
        out_torques_data.setZero(7);
    }
    // out_torques_port.write(out_torques_data + in_coriolisAndGravity_data - kv * qd);
    out_torques_port.write(out_torques_data);
    // prepare_monitors();
    // write_ports();

}

void RTTControlStack::set_sot_references()
{
  Eigen::MatrixXd pose_out_data = Eigen::MatrixXd::Identity(4,4);
  pose_out_data.block<3,1>(0,3) << in_desiredTaskSpace_var.transforms[0].translation.x,in_desiredTaskSpace_var.transforms[0].translation.y,in_desiredTaskSpace_var.transforms[0].translation.z;

  Eigen::Quaterniond rotation;
  rotation.vec() << in_desiredTaskSpace_var.transforms[0].rotation.x,in_desiredTaskSpace_var.transforms[0].rotation.y,in_desiredTaskSpace_var.transforms[0].rotation.z;
  rotation.w() = in_desiredTaskSpace_var.transforms[0].rotation.w;
  pose_out_data.block<3,3>(0,0) = rotation.toRotationMatrix();
  // pass the set-points read from ports to the SoT
  iHQP_SoT->update(
               ff_out_data,
               cart_stiff_out_data,
               cart_damp_out_data,
               pose_out_data,
               jnt_stiff_out_data,
               jnt_damp_out_data,
               des_posture_out_data);
}

void RTTControlStack::stopHook()
{
    RTT::log(RTT::Info) <<"Stopping..."<<RTT::endlog();
}

void RTTControlStack::update_model()
{
    this->model->setJointPosition(q);
	this->model->setJointVelocity(qd);
	// this->model->setJointAcceleration(qdd);
	// this->model->setJointEffort(tau);
	this->model->update();

}

void RTTControlStack::setJntPosture(int idx, double value)
{
    des_posture_out_data(idx) = value;
}

void RTTControlStack::setCartStiffness(double KP)
{
    // cart_stiff_out_data = Eigen::MatrixXd::Identity(6, 6) * KP;

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
    for (unsigned int i = 0; i < 6; i++)
    {
        cart_stiff_out_data(i,i) = KP(i);
    }
}

void RTTControlStack::setCartDamping(double KD)
{
    // cart_damp_out_data = Eigen::MatrixXd::Identity(6, 6) * KD;
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
    in_desiredTaskSpace_var.transforms[0].translation.x = -0.0287248;
    in_desiredTaskSpace_var.transforms[0].translation.y = -0.529656 ;
    in_desiredTaskSpace_var.transforms[0].translation.z = 0.467355  ;
    in_desiredTaskSpace_var.transforms[0].rotation.w = 0.0174455 ;
    in_desiredTaskSpace_var.transforms[0].rotation.x = 0.687507  ;
    in_desiredTaskSpace_var.transforms[0].rotation.y = -0.72578  ;
    in_desiredTaskSpace_var.transforms[0].rotation.z = 0.0165435 ;











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

void RTTControlStack::read_ports()
{
    /*----------------------------input port from robot---------------------------*/
    in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_data);
	in_coriolisAndGravity_port.read(in_coriolisAndGravity_data);
	for(int i=0; i<7; ++i){
		q[i] = in_robotstatus_data.position[i];
		qd[i] = in_robotstatus_data.velocity[i];
		tau[i] = in_robotstatus_data.effort[i];
	}
    
    in_desiredTaskSpace_flow = in_desiredTaskSpace_port.read(in_desiredTaskSpace_var);
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
