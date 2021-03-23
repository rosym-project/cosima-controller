#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include <OpenSoT/tasks/torque/CartesianImpedanceCtrl.h>
#include <OpenSoT/tasks/torque/JointImpedanceCtrl.h>
#include <OpenSoT/constraints/torque/JointLimits.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>


using namespace OpenSoT::tasks::torque;
using namespace OpenSoT::constraints::torque;
using namespace OpenSoT::solvers;

namespace cosima
{
  namespace controller
  {
    class WrenchTransition {
    public:
        WrenchTransition(const std::string &target, const unsigned int &dir, const bool &greater, const double &value)
        {
          t_target = target;
          t_dir = dir;
          t_greater = greater;
          t_value = value;
        };

        bool eval(const Eigen::VectorXd &cur)
        {
          if (t_greater)
          {
            if (cur(t_dir) > t_value)
            {
              return true;
            }
          }
          else
          {
            if (cur(t_dir) < t_value)
            {
              return true;
            }
          }

          return false;
        };

        std::string t_target;
    private:
        unsigned int t_dir;
        bool t_greater;
        double t_value;
    };

    class CS {
    public:
        CS(std::string const &name)
        {
          model_contacts = Eigen::VectorXi::Zero(6);
          model_compliance = Eigen::VectorXi::Zero(6);
          model_msd = Eigen::VectorXi::Zero(6);
          model_msd_gain_stiffness = Eigen::VectorXd::Zero(6);
          model_msd_gain_damping = Eigen::VectorXd::Zero(6);
          model_js_gain_stiffness = Eigen::VectorXd::Zero(7);
          model_js_gain_damping = Eigen::VectorXd::Zero(7);

          tmp_m_msd_gain_stiffness = Eigen::MatrixXd::Zero(6,6);
          tmp_m_msd_gain_damping = Eigen::MatrixXd::Zero(6,6);
          tmp_m_js_gain_stiffness = Eigen::MatrixXd::Zero(7,7);
          tmp_m_js_gain_damping = Eigen::MatrixXd::Zero(7,7);

          wrench_command = Eigen::VectorXd::Zero(6);

          cs_name = name;
        };

        Eigen::VectorXi model_contacts;
        Eigen::VectorXi model_compliance;
        Eigen::VectorXi model_msd;
        Eigen::VectorXd model_msd_gain_stiffness;
        Eigen::VectorXd model_msd_gain_damping;
        Eigen::VectorXd model_js_gain_stiffness;
        Eigen::VectorXd model_js_gain_damping;

        Eigen::MatrixXd tmp_m_msd_gain_stiffness;
        Eigen::MatrixXd tmp_m_msd_gain_damping;
        Eigen::MatrixXd tmp_m_js_gain_stiffness;
        Eigen::MatrixXd tmp_m_js_gain_damping;
        bool enable_js_task;

        Eigen::VectorXd wrench_command;

        std::string cs_name;

        std::vector<std::shared_ptr<WrenchTransition>> transitions;

        /*------------------------------task declaration------------------------------*/
        CartesianImpedanceCtrl::Ptr task_msd;
        JointImpedanceCtrl::Ptr task_js;
        JointLimits::Ptr global_jnt_limits;

        /*-------------------------------solver settings------------------------------*/
        OpenSoT::AutoStack::Ptr stack;

        OpenSoT::solvers::iHQP::Ptr qp_problem_solver;

        bool enable_contact;
        bool enable_compliance;
        bool enable_msd;

        void configure(const XBot::ModelInterface::Ptr model, const Eigen::VectorXd &q)
        {
          ///////////////////////////////
          // Here comes now the magic! //
          ///////////////////////////////
          enable_contact = false;
          enable_compliance = false;
          enable_msd = false;
          enable_js_task = false;

          //////////////////////////////// 1) Find out which tasks should be used ///////////////////////////////
          enable_contact = !model_contacts.isZero();
          enable_compliance = !model_compliance.isZero();
          enable_msd = !model_msd.isZero();
          // enable_js_task = !model_contacts.isZero();

          task_msd.reset(new CartesianImpedanceCtrl("msd", q, *model, "iiwa_link_ee", "world"));
          if (enable_js_task)
          {
            task_js.reset(new JointImpedanceCtrl(q, *model));
          }

          //////////////////////////////// 2) Configure execution semantics ///////////////////////////////
          // 1) Start with MSD impedance parameters.
          // 2) Adapt gains according to CT and C.
          for (unsigned int i = 0; i < 6; i++)
          {
            if ((model_contacts(i) == 1) || (model_compliance(i) == 1))
            {
              model_msd_gain_stiffness(i) = 0.0;
              model_msd_gain_damping(i) = 0.0;
            }
          }

          // msd < c < ct
          if (enable_js_task)
          {
            stack = ((task_msd) / (task_js));
          }
          else
          {
            stack.reset(new OpenSoT::AutoStack(task_msd));
          }
          // qp_problem_solver.reset(new iHQP(stack->getStack(), 1e9, solver_back_ends::eiQuadProg));
          qp_problem_solver.reset(new iHQP(stack->getStack(), 1e5, solver_back_ends::qpOASES));
        };

        void setImpedance(Eigen::VectorXd & _task_msd_cartesian_stiffness,
                          Eigen::VectorXd & _task_msd_cartesian_damping)
        {
          // 3) only allows changing stiffness in msd dirs
          for (unsigned int i = 0; i < 6; i++)
          {
            if ((model_contacts(i) == 1) || (model_compliance(i) == 1))
            {
              model_msd_gain_stiffness(i) = 0.0;
              model_msd_gain_damping(i) = 0.0;
            }
            else
            {
              model_msd_gain_stiffness(i) = _task_msd_cartesian_stiffness(i);
              model_msd_gain_damping(i) = _task_msd_cartesian_damping(i);
            }
          }

          // this->task_msd->setStiffness(model_msd_gain_stiffness);
          // this->task_msd->setDamping(model_msd_gain_damping);
        };

        void setJsImpedance(Eigen::VectorXd & _task_js_joint_stiffness,
                            Eigen::VectorXd & _task_js_joint_damping)
        {
          for (unsigned int i = 0; i < 7; i++)
          {
            model_js_gain_stiffness(i) = _task_js_joint_stiffness(i);
            model_js_gain_damping(i) = _task_js_joint_damping(i);
          }

          // this->task_js->setStiffness(model_js_gain_stiffness);
          // this->task_js->setDamping(model_js_gain_damping);
        };

        void update(Eigen::VectorXd & _task_msd_feedforward_forces,
                    Eigen::MatrixXd & _task_msd_cartesian_pose,
                    Eigen::VectorXd & _task_js_desired_joint,
                    Eigen::VectorXd & _task_msd_cartesian_stiffness,
                    Eigen::VectorXd & _task_msd_cartesian_damping,
                    Eigen::VectorXd & _task_js_joint_stiffness,
                    Eigen::VectorXd & _task_js_joint_damping)
        {

          // 4) Only allow forces to be applied in contact directions
          for (unsigned int i = 0; i < 6; i++)
          {
            if ((model_contacts(i) == 1) || (model_compliance(i) == 1))
            {
              wrench_command(i) = _task_msd_feedforward_forces(i);
            }
            else
            {
              wrench_command(i) = 0.0;
            }

            tmp_m_msd_gain_stiffness(i,i) = _task_msd_cartesian_stiffness(i);
            tmp_m_msd_gain_damping(i,i) = _task_msd_cartesian_damping(i);
          }


          for (unsigned int i = 0; i < 7; i++)
          {
            tmp_m_js_gain_stiffness(i,i) = _task_js_joint_stiffness(i);
            tmp_m_js_gain_damping(i,i) = _task_js_joint_damping(i);
          }


          this->task_msd->setStiffness(tmp_m_msd_gain_stiffness);
          this->task_msd->setDamping(tmp_m_msd_gain_damping);
          if (enable_js_task)
          {
            this->task_js->setStiffness(tmp_m_js_gain_stiffness);
            this->task_js->setDamping(tmp_m_js_gain_damping);
          }
          
          this->task_msd->setFeedForwardForces(wrench_command);
          
          this->task_msd->setReference(_task_msd_cartesian_pose);

          if (enable_js_task)
          {
            this->task_js->setReference(_task_js_desired_joint);
          }
        };

    };

  } // namespace controller

} // namespace cosima