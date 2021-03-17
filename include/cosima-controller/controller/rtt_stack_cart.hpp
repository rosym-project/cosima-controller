#pragma once

#include <OpenSoT/tasks/torque/CartesianImpedanceCtrl.h>
#include <OpenSoT/tasks/torque/JointImpedanceCtrl.h>
#include <OpenSoT/constraints/torque/JointLimits.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <OpenSoT/utils/AutoStack.h>


using namespace OpenSoT::tasks::torque;
using namespace OpenSoT::constraints::torque;
using namespace OpenSoT::solvers;

class qp_problem {

public:
    qp_problem(const XBot::ModelInterface::Ptr model, const Eigen::VectorXd &q) {
	
		/*--------------------------------constructors--------------------------------*/
		cart_imped_high.reset(new CartesianImpedanceCtrl("cart_imped_high", q, *model, "iiwa_link_ee", "world"));
		joint_space_redres.reset(new JointImpedanceCtrl(q, *model));
	
		/*-------------------------------initializations------------------------------*/
		// cart_imped_high->setLambda(0.001);
		// joint_space_redres->setLambda(0.05);
	
		/*--------------------------------joint limits--------------------------------*/
		// Eigen::VectorXd qmin, qmax;
		// model->getJointLimits (qmin, qmax);
		// global_jnt_limits.reset(new JointLimits(q, qmax, qmin, *model));
	
		/*-------------------------------stack of tasks-------------------------------*/
		stack = ((cart_imped_high) / (joint_space_redres));
		// qp_problem_solver.reset(new iHQP(stack->getStack(), 1e9, solver_back_ends::eiQuadProg));
    qp_problem_solver.reset(new iHQP(stack->getStack(), 1e5, solver_back_ends::qpOASES));
	}

    /*------------------------------task declaration------------------------------*/
    CartesianImpedanceCtrl::Ptr cart_imped_high;
    JointImpedanceCtrl::Ptr joint_space_redres;
    JointLimits::Ptr global_jnt_limits;

    /*-------------------------------solver settings------------------------------*/
    OpenSoT::AutoStack::Ptr stack;
    OpenSoT::solvers::iHQP::Ptr qp_problem_solver;

    void update(
           Eigen::VectorXd & _cart_imped_high_feedforward_forces,
           Eigen::MatrixXd & _cart_imped_high_cartesian_stiffness,
           Eigen::MatrixXd & _cart_imped_high_cartesian_damping,
           Eigen::MatrixXd & _cart_imped_high_cartesian_pose,
           Eigen::MatrixXd & _joint_space_redres_joint_stiffness,
           Eigen::MatrixXd & _joint_space_redres_joint_damping,
           Eigen::VectorXd & _joint_space_redres_desired_joint) {

               this->cart_imped_high->setFeedForwardForces(_cart_imped_high_feedforward_forces);
               this->cart_imped_high->setStiffness(_cart_imped_high_cartesian_stiffness);
               this->cart_imped_high->setDamping(_cart_imped_high_cartesian_damping);
               this->cart_imped_high->setReference(_cart_imped_high_cartesian_pose);
               this->joint_space_redres->setStiffness(_joint_space_redres_joint_stiffness);
               this->joint_space_redres->setDamping(_joint_space_redres_joint_damping);
               this->joint_space_redres->setReference(_joint_space_redres_desired_joint);
    }

};
