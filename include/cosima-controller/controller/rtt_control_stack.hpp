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
#include <geometry_msgs/Wrench.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "rtt_stack_cart.hpp"
#include "rtt_stack_cart_wo_js.hpp"

#include "rtt_cs.hpp"

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

        // void setJointStiffnessE(const Eigen::VectorXd &KP);
        // void setJointDampingE(const Eigen::VectorXd &KD);

        // void setCartStiffnessE(const Eigen::VectorXd &KP);
        // void setCartDampingE(const Eigen::VectorXd &KD);

    private:
        RTT::InputPort<trajectory_msgs::MultiDOFJointTrajectoryPoint> in_desiredTaskSpace_port;
        RTT::FlowStatus in_desiredTaskSpace_flow;
        trajectory_msgs::MultiDOFJointTrajectoryPoint in_desiredTaskSpace_var;

        RTT::InputPort<trajectory_msgs::JointTrajectoryPoint> in_desiredJointSpace_port;
        RTT::FlowStatus in_desiredJointSpace_flow;
        trajectory_msgs::JointTrajectoryPoint in_desiredJointSpace_var;

        RTT::InputPort<geometry_msgs::Wrench> in_contactWrench_port;
        RTT::FlowStatus in_contactWrench_flow;
        geometry_msgs::Wrench in_contactWrench_var;

        RTT::InputPort<geometry_msgs::Wrench> in_wrench_fdb_port;
        RTT::FlowStatus in_wrench_fdb_flow;
        geometry_msgs::Wrench in_wrench_fdb_var;

        Eigen::VectorXd in_wrench_fdb_data;

        bool simulation;

        double rate;


        XBot::ModelInterface::Ptr model;
        bool model_configured;
        std::string path_to_model_config;

        /*------------------------------helper functions------------------------------*/
        double getTime();
        void init_ports();
        // void prepare_monitors();
        void write_ports();
        bool check_ports_connectivity();
        bool load_config(std::string config_path);
        double kv;

        // void printInfo();

        // void setPosition(double x, double y, double z);
        // void setOrientation(double x, double y, double  z, double w);
        // void setCartStiffness(double KP);
        // void setCartDamping(double KD);
        // void setJointStiffness(double KP);
        // void setJointDamping(double KD);
        // void setFF(double x, double y, double z);
        // void setJntPosture(int idx, double value);

        Eigen::VectorXd ff_out_data;
        // Eigen::MatrixXd cart_stiff_out_data;
        // Eigen::MatrixXd cart_damp_out_data;
        // Eigen::MatrixXd jnt_stiff_out_data;
        // Eigen::MatrixXd jnt_damp_out_data;
        Eigen::VectorXd des_posture_out_data;

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

        // int stack_type;

        void addContact(const std::string &cs, const unsigned int &dir);
        void addCompliance(const std::string &cs, const unsigned int &dir);
        void addMSD(const std::string &cs, const Eigen::VectorXd &stiffness, const Eigen::VectorXd &damping);
        void addMSDdir(const std::string &cs, const unsigned int &dir, const double &stiffness, const double &damping);
        void addJSMsd(const std::string &cs, const Eigen::VectorXd &stiffness, const Eigen::VectorXd &damping);
        void addCS(const std::string &cs);
        void useTransitionWrench(const std::string &cs, const std::string &target, const unsigned int &dir, const bool &greater, const double &value);
        //
        std::map<std::string,std::shared_ptr<CS>> model_css;
        std::shared_ptr<CS> current_cs;
        std::shared_ptr<CS> old_cs; // TODO
        //
        Eigen::VectorXd cur_model_msd_gain_stiffness, old_msd_gain_stiffness;
        Eigen::VectorXd cur_model_msd_gain_damping, old_msd_gain_damping;
        Eigen::VectorXd cur_model_js_gain_stiffness, old_js_gain_stiffness;
        Eigen::VectorXd cur_model_js_gain_damping, old_js_gain_damping;
        

        double change_in_sec;


    };

  } // namespace controller

} // namespace cosima