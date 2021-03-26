#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <XBotInterface/ModelInterface.h>
#include <sensor_msgs/JointState.h> // for robot feedback
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <std_msgs/Float64MultiArray.h>
// #include <rtt_roscomm/rtt_rostopic.h> // deprecated. use the one below
#include <rtt_roscomm/rostopic.h>

#include <OpenSoT/tasks/torque/CartesianImpedanceCtrl.h>
#include <OpenSoT/tasks/torque/JointImpedanceCtrl.h>
#include <OpenSoT/constraints/torque/JointLimits.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <OpenSoT/utils/AutoStack.h>

#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>

#include "rtt_stack_cart.hpp"

#include <thread>
#include <mutex>
#include <condition_variable>

namespace cosima
{
  namespace controller
  {
    class RTTControlStack: public RTT::TaskContext {
    public:
        RTTControlStack(std::string const & name);

        bool configureHook();
        bool startHook();
        void updateHook();
        void stopHook();
        void cleanupHook();

        void setJointStiffnessE(const Eigen::VectorXd &KP);
        void setJointDampingE(const Eigen::VectorXd &KD);

        // Deprecated
        void setCartStiffnessE(const Eigen::VectorXd &KP);
        void setCartDampingE(const Eigen::VectorXd &KD);

        void setMassSpringDamper(const Eigen::VectorXd &KP, const Eigen::VectorXd &KD, double time, bool blocking);
        void setContactConstraintForce(const Eigen::VectorXd &force, double time, bool blocking);

    private:
        /*---------------------------------input ports--------------------------------*/
        // RTT::InputPort<Eigen::VectorXd> cart_imped_high_feedforward_forces_input_port;
        // RTT::InputPort<Eigen::MatrixXd> cart_imped_high_cartesian_stiffness_input_port;
        // RTT::InputPort<Eigen::MatrixXd> cart_imped_high_cartesian_damping_input_port;
        RTT::InputPort<trajectory_msgs::MultiDOFJointTrajectoryPoint> in_desiredTaskSpace_port;
        // RTT::InputPort<Eigen::VectorXd> joint_space_redres_desired_joint_input_port;
        // RTT::InputPort<Eigen::MatrixXd> joint_space_redres_joint_damping_input_port;
        // RTT::InputPort<Eigen::MatrixXd> joint_space_redres_joint_stiffness_input_port;

        /*-----------------------------ports' flow states-----------------------------*/
        // RTT::FlowStatus cart_imped_high_feedforward_forces_flow;
        // RTT::FlowStatus cart_imped_high_cartesian_stiffness_flow;
        // RTT::FlowStatus cart_imped_high_cartesian_damping_flow;
        RTT::FlowStatus in_desiredTaskSpace_flow;
        // RTT::FlowStatus joint_space_redres_desired_joint_flow;
        // RTT::FlowStatus joint_space_redres_joint_damping_flow;
        // RTT::FlowStatus joint_space_redres_joint_stiffness_flow;

        /*------------------------------input ports data------------------------------*/
        // Eigen::VectorXd cart_imped_high_feedforward_forces_input_data;
        // Eigen::MatrixXd cart_imped_high_cartesian_stiffness_input_data;
        // Eigen::MatrixXd cart_imped_high_cartesian_damping_input_data;
        trajectory_msgs::MultiDOFJointTrajectoryPoint in_desiredTaskSpace_var;
        // Eigen::VectorXd joint_space_redres_desired_joint_input_data;
        // Eigen::MatrixXd joint_space_redres_joint_damping_input_data;
        // Eigen::MatrixXd joint_space_redres_joint_stiffness_input_data;

        /*--------------------------------output ports--------------------------------*/
        // RTT::OutputPort<std_msgs::Float64MultiArray> cart_imped_high_feedforward_forces_output_port;
        // RTT::OutputPort<geometry_msgs::Pose> cart_imped_high_cartesian_pose_output_port;
        // RTT::OutputPort<std_msgs::Float64MultiArray> joint_space_redres_desired_joint_output_port;

        /*------------------------------output ports data-----------------------------*/
        // Eigen::VectorXd cart_imped_high_feedforward_forces_output_data;
        // Eigen::MatrixXd cart_imped_high_cartesian_pose_output_data;
        // Eigen::VectorXd joint_space_redres_desired_joint_output_data;
        // std_msgs::Float64MultiArray cart_imped_high_feedforward_forces_rosout_data;
        // geometry_msgs::Pose cart_imped_high_cartesian_pose_rosout_data;
        // std_msgs::Float64MultiArray joint_space_redres_desired_joint_rosout_data;

        /*-------------------------backend solvers and stacks-------------------------*/
        boost::shared_ptr<qp_problem> iHQP_SoT;
        // TODO: This is hard coded. FIXIT!
        XBot::ModelInterface::Ptr model;
        bool model_configured;
        std::string path_to_model_config;

        /*------------------------------helper functions------------------------------*/
        double getTime();
        void init_ports();
        void prepare_monitors();
        void write_ports();
        bool check_ports_connectivity();
        bool load_config(std::string config_path);
        double kv;

        void printInfo();

        void setPosition(double x, double y, double z);
        void setOrientation(double x, double y, double  z, double w);
        void setCartStiffness(double KP);
        void setCartDamping(double KD);
        void setJointStiffness(double KP);
        void setJointDamping(double KD);
        void setFF(double x, double y, double z);
        void setFFRot(double x, double y, double z);
        void setJntPosture(int idx, double value);

        Eigen::VectorXd ff_out_data, t_ff_out_data, s_ff_out_data, e_ff_out_data;
        Eigen::MatrixXd cart_stiff_out_data;
        Eigen::MatrixXd cart_damp_out_data;
        Eigen::MatrixXd jnt_stiff_out_data;
        Eigen::MatrixXd jnt_damp_out_data;
        Eigen::VectorXd des_posture_out_data;

        Eigen::MatrixXd s_cart_stiff_out_data;
        Eigen::MatrixXd s_cart_damp_out_data;
        Eigen::MatrixXd e_cart_stiff_out_data;
        Eigen::MatrixXd t_cart_stiff_out_data;
        Eigen::MatrixXd e_cart_damp_out_data;
        Eigen::MatrixXd t_cart_damp_out_data;

        /*-------------------output torques and robot configruation-------------------*/
        // fix the DoF size in the next line during code generation!
        RTT::OutputPort<Eigen::VectorXd> out_torques_port;
        // RTT::InputPort<Eigen::VectorXd> q_in, qd_in, qdd_in, tau_in; // robot feedback
        Eigen::VectorXd out_torques_data;
        Eigen::VectorXd q, qd, qdd, tau;
        // We decided to go all the way Eigen... what is this rtt-sim-embeded!??
        RTT::InputPort<sensor_msgs::JointState> in_robotstatus_port;
        RTT::FlowStatus in_robotstatus_flow;
        sensor_msgs::JointState in_robotstatus_data;

        RTT::InputPort<Eigen::VectorXd> in_coriolisAndGravity_port;
        RTT::FlowStatus in_coriolisAndGravity_flow;
        Eigen::VectorXd in_coriolisAndGravity_data;

        bool first_no_command;

        bool finished_update_gains;
        bool triggered_update_gains;
        double command_gains_update_duration;
        double gains_update_duration;
        double gains_update_time;

        bool finished_update_force;
        bool triggered_update_force;
        double command_force_update_duration;
        double force_update_duration;
        double force_update_time;

        //
        std::mutex m, m_force;
        std::condition_variable cv, cv_force;
    };

  } // namespace controller

} // namespace cosima