/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2019 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
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
#pragma once
#include <iostream>
#include <iterator>
#include <map>
#include <memory> // for std::shared_ptr
#include <vector>
#include <string>

#include "rtt_ghc_projections.hpp"

namespace cosima
{

    namespace prioritization
    {

        struct BlockStorage
        {
            unsigned int coIndex;
            unsigned int jac_begin_i;
            unsigned int jac_begin_j;
            unsigned int p;
            unsigned int q;
            unsigned int tau_begin;
        };

        struct VMCombinedRobotStorage
        {
            std::string name;
            unsigned int startingIndexInVector;
            unsigned int jointDof;
        };

        class RobotContactSituations
        {
        public:
            RobotContactSituations();
            RobotContactSituations(const unsigned int numberOfCS, const unsigned int jointDoF, const unsigned int numTasks, bool amVM);
            void setRobotName(const std::string &name);

            bool addCSAlpha(const std::string &csName, const Eigen::MatrixXd &alpha, const std::vector<std::string> &header);

            void initGHC();

            void addControlObjectiveTask(const std::string &coName, const unsigned int index, const unsigned int taskDoF);
            void addPortMapping(const unsigned int portId, const unsigned int coIndex, const unsigned int jac_begin_i, const unsigned int jac_begin_j, const unsigned int selfRowSize, const unsigned int tau_begin);

            void setNewTargetCS(const std::string &csName);
            void updateOutputTorques(const float transitionIndex, Eigen::VectorXd &torques_out);
            void updateOutputTorquesVM(const float transitionIndex, Eigen::VectorXd &torques_out);

            unsigned int getControlObjectiveTaskIndex(const std::string &name);

            void addVM2RealMapping(const std::string &name, const unsigned int startingIndexInVector, const unsigned int jointDof);

            void addRobot2CombinedRobots(const std::string &name);
            std::vector<std::string> getCombinedRobotNames();

            bool amIaVM();

            void updateJCombinedRobots();

            bool setStartCS(const std::string &name);

            void updateJacobian(const unsigned int portId, const Eigen::MatrixXd &metaJac, const Eigen::VectorXd &metaCmds);
            void updateInertia(const unsigned int portId, const Eigen::MatrixXd &inertia);

            std::string getRobotName();

            void updateTorqueCommandFromCombinedRobots(const Eigen::VectorXd &metaCmds, const Eigen::VectorXd &robotstatus_pos, const Eigen::VectorXd &robotstatus_vel, const Eigen::VectorXd &robotstatus_trq, const Eigen::VectorXd &coriolisAndGravity);

            void debug_print_state(bool debug);

            void setPortIdforVMSpecificPorts(unsigned int portIndex);

            bool amIaVM(unsigned int portIndex);

            void updateInteralAndExternalProjection(Eigen::MatrixXd &internal, Eigen::MatrixXd &internal_dot);

            unsigned int getTotalJoints();

            void addCompensationForCOT(const unsigned int index, const unsigned int index_task_to_be_compensated_for);

            // void setStartingIndex(unsigned int startingIndex);

            void updateStatusAndGravity(const Eigen::VectorXd &robotstatus_pos, const Eigen::VectorXd &robotstatus_vel, const Eigen::VectorXd &robotstatus_trq, const Eigen::VectorXd &coriolisAndGravity);

            // void logDebugInfo(const RTT::LoggerLevel lvl);

            Eigen::VectorXd torque_commands_proj_states_stacked;
            Eigen::MatrixXd allProjections_stacked;

            void getCurrentAlphas(Eigen::MatrixXd &out_alphas);

        private:
            std::string robotName;
            unsigned int jointDoF;
            unsigned int numTasks;

            GHCProjections ghcProj;

            // std::vector<Eigen::MatrixXd> alpha_per_cs;
            std::map<std::string, Eigen::MatrixXd> alpha_per_cs;

            Eigen::VectorXd tasksizes;

            Eigen::MatrixXd lastActiveAlpha;

            Eigen::MatrixXd selection;

            std::vector<Eigen::VectorXd> torque_commands;

            std::vector<Eigen::MatrixXd> allProjections;

            Eigen::VectorXd ranks;

            // std::map<int, std::string> controlObjectivesMap;
            std::vector<std::string> controlObjectivesList;

            std::vector<std::vector<int>> compensation_for_cots;

            // This here has the size of all available ports
            std::map<unsigned int, std::vector<BlockStorage>> coIdandPortPartIdPerPort;

            //TODO can be optimized!
            std::vector<Eigen::MatrixXd> jacs;

            // Eigen::MatrixXd inertia_;

            std::vector<VMCombinedRobotStorage> mapping_from_VMmanipulator_to_REALmanipulator;

            bool amVM;

            float transitionIndex;

            std::vector<std::string> combinedRobotNames;

            // unsigned int singleRobotJointDoFSize;

            std::string debug_global_status_string_;

            bool debug_print_state_;

            unsigned int portIndex_;

            Eigen::MatrixXd projection_internal_;
            Eigen::MatrixXd projection_internal_dot_;

            Eigen::VectorXd robotstatus_pos_;
            Eigen::VectorXd robotstatus_vel_;
            Eigen::VectorXd robotstatus_trq_;
            Eigen::VectorXd coriolisAndGravity_;

            Eigen::MatrixXd identityDOFsizeDOFsize;
        };

    } // namespace prioritization

} // namespace cosima