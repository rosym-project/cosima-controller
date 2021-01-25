/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2018 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
 *                    based on Niels Dehio's,
 *                             Sina Mirrazavi's,
 *                             Joshua Smith's,
 *                             Hsiu-Chin Lin's implementations
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

// #include "prioritization/GHCProjections.hpp"

#include "../../include/cosima-controller/controller/rtt_cart_pid_controller.hpp"

#include <rtt/Component.hpp> // needed for the macro at the end of this file

using namespace cosima;
using namespace controller;

RTTCartPIDController::RTTCartPIDController(std::string const &name)
    : RTT::TaskContext(name)
{
    addOperation("useTSgravitycompensation", &RTTCartPIDController::useTSgravitycompensation, this).doc("use taskspace gravitycompensation");
    addOperation("addJSgravitycompensation", &RTTCartPIDController::addJSgravitycompensation, this).doc("add jointspace gravitycompensation");
    addOperation("setGains", &RTTCartPIDController::setGains, this).doc("set gains");
    addOperation("setGainsOrientation", &RTTCartPIDController::setGainsOrientation, this).doc("set gains orientation");
    addOperation("displayStatus", &RTTCartPIDController::displayStatus, this).doc("print status");
    addOperation("preparePorts", &RTTCartPIDController::preparePorts, this).doc("preparePorts");
    addOperation("checkConnections", &RTTCartPIDController::checkConnections, this, RTT::ClientThread).doc("check connections");
    addOperation("setNoCommandReceivedBehavior", &RTTCartPIDController::setNoCommandReceivedBehavior, this, RTT::ClientThread).doc("Set the behavior when no command is received").arg("type", "nothing : won't send anything. holdposition : holds the position received when the component was started. gravity : gravity compensation");

    addOperation("loadYAMLConfig", &RTTCartPIDController::loadYAMLConfig, this, RTT::ClientThread).doc("Read Config from YAML.").arg("myname", "Name of the component in the config").arg("file", "File path of the config YAML");
    addOperation("addRobot", &RTTCartPIDController::addRobot, this, RTT::ClientThread).doc("Manually add a robot.").arg("task_dof", "Task Space Dimensions of the Manipulator").arg("joint_dof", "Joint Space Dimensions of the Manipulator");
    addOperation("calculateTotalDofs", &RTTCartPIDController::calculateTotalDofs, this, RTT::ClientThread).doc("Needs to be called after (all) robots are manually added.");

    addOperation("setGainsForEE", &RTTCartPIDController::setGainsForEE, this, RTT::ClientThread)
        .doc("Position gains kp and kd can be set for the controlled robots independently.")
        .arg("kp", "Stiffness gain")
        .arg("kd", "Damping gain")
        .arg("index", "Index of the robot in question");
    addOperation("setGainsOrientationForEE", &RTTCartPIDController::setGainsOrientationForEE, this, RTT::ClientThread)
        .doc("Orientation gains kp and kd can be set for the controlled robots independently.")
        .arg("kp", "Stiffness gain")
        .arg("kd", "Damping gain")
        .arg("index", "Index of the robot in question");

    addOperation("setGainsBatch", &RTTCartPIDController::setGainsBatch, this, RTT::ClientThread)
        .doc("Position gains kp and kd can be set for the controlled robots as eigen vectors.")
        .arg("kps", "Stiffness gain eigen vector")
        .arg("kds", "Damping gain eigen vector");
    addOperation("setGainsOrientationBatch", &RTTCartPIDController::setGainsOrientationBatch, this, RTT::ClientThread)
        .doc("Orientation gains kp and kd can be set for the controlled robots as eigen vectors.")
        .arg("kps", "Stiffness gain eigen vector")
        .arg("kds", "Damping gain eigen vector");

    addOperation("setGainsSingleForEE", &RTTCartPIDController::setGainsSingleForEE, this, RTT::ClientThread)
        .doc("Single position gain kp and kd can be set for the controlled robots independently.")
        .arg("kp", "Stiffness gain")
        .arg("kd", "Damping gain")
        .arg("index", "Index of the robot in question")
        .arg("dir", "Directional Axis for single gain");
    addOperation("setGainsOrientationSingleForEE", &RTTCartPIDController::setGainsOrientationSingleForEE, this, RTT::ClientThread)
        .doc("Single orientation gain kp and kd can be set for the controlled robots independently.")
        .arg("kp", "Stiffness gain")
        .arg("kd", "Damping gain")
        .arg("index", "Index of the robot in question")
        .arg("dir", "Directional Axis for single gain");

    addProperty("impedanceCTRL", impedanceCTRL);

    addProperty("lockOrientation", lockOrientation); // TODO
    lockOrientation = false;

    noCommandReceivedBehaviorType = 1;

    impedanceCTRL = true;

    hold_current_position = true;
    hold_current_position_last = false;

    portsArePrepared = false;
    this->useTSgravitycompensation(false);
    this->addJSgravitycompensation(true);

    desiredPosition = Eigen::Vector3d::Zero();
    currentPosition = Eigen::Vector3d::Zero();
    desiredQuaternionPosition = Eigen::Vector4d::Zero();
    currentQuaternionPosition = Eigen::Vector4d::Zero();
    desiredVelocity = Eigen::Vector3d::Zero();
    currentVelocity = Eigen::Vector3d::Zero();

    errorTranslationPosition = Eigen::Vector3d::Zero();
    errorTranslationVelocity = Eigen::Vector3d::Zero();
    errorOrientationPosition = Eigen::Vector3d::Zero();
    errorOrientationVelocity = Eigen::Vector3d::Zero();

    errorPosition = Eigen::VectorXd::Zero(6);
    errorVelocity = Eigen::VectorXd::Zero(6);

    // qh = QuaternionHelper();

    total_dof_size_ = 0;

    WorkspaceDimension = 6;
    WorkspaceQuaternionDimension = 7;
    TaskSpaceDimension = 6;
    TaskSpaceQuaternionDimension = 7;
}

bool RTTCartPIDController::loadYAMLConfig(const std::string &myname, const std::string &file)
{
    YAML::Node config = YAML::LoadFile(file);

    YAML::Node rootControlObjectivesConfig = config["RootControlObjectives"];
    if (!rootControlObjectivesConfig.IsDefined() || rootControlObjectivesConfig.IsNull())
    {
        RTT::log(RTT::Error) << "RootControlObjectives is not defined properly!" << RTT::endlog();
        return false;
    }

    YAML::Node portMapsConfig = config["PortMaps"];
    if (!portMapsConfig.IsDefined() || portMapsConfig.IsNull())
    {
        RTT::log(RTT::Error) << "PortMaps is not defined properly!" << RTT::endlog();
        return false;
    }

    YAML::Node prioComponent = portMapsConfig["Prioritization"];
    if (!prioComponent.IsDefined() || prioComponent.IsNull() || !prioComponent.IsSequence())
    {
        RTT::log(RTT::Error) << "Prioritization is not defined properly in PortMaps!" << RTT::endlog();
        return false;
    }

    for (std::size_t i = 0; i < prioComponent.size(); i++)
    {
        YAML::Node port = prioComponent[i];
        YAML::Node port_name = port["Name"];

        std::string port_name_val = port_name.as<std::string>();
        if (port_name_val.compare(myname) == 0) // Found the entry that is relevant for me!
        {
            // for each of the involved control objectives also find the associated robot
            YAML::Node port_co_split_seq = port["ControlObjectives"];
            if (!port_co_split_seq.IsDefined() || port_co_split_seq.IsNull() || !port_co_split_seq.IsSequence())
            {
                RTT::log(RTT::Error) << "ControlObjectives are not defined in port " << i << "!" << RTT::endlog();
                return false;
            }

            unsigned int _totalTaskDof = 0;
            unsigned int _totalJointDof = 0;
            for (std::size_t j = 0; j < port_co_split_seq.size(); j++)
            {
                std::string involved_co_name = port_co_split_seq[j].as<std::string>();

                for (std::size_t i = 0; i < rootControlObjectivesConfig.size(); i++)
                {
                    YAML::Node entry = rootControlObjectivesConfig[i];
                    YAML::Node robot = entry["Robot"];
                    std::string robotName = robot.as<std::string>();
                    unsigned int jointDOF = entry["JointDof"].as<unsigned int>();

                    // YAML::Node amVM = entry["VM"];
                    // bool amIaVM = true;
                    // if (!amVM.IsDefined() || amVM.IsNull())
                    // {
                    //     amIaVM = false;
                    // }
                    YAML::Node cos = entry["ControlObjectives"];
                    for (std::size_t j = 0; j < cos.size(); j++)
                    {
                        std::string candidate_co_name = cos[j]["Name"].as<std::string>();

                        if (involved_co_name.compare(candidate_co_name) == 0) // Found the involved CO in the wild!
                        {
                            unsigned int taskdof = cos[j]["OperationalFrame"]["Dof"].as<unsigned int>();
                            std::string frame_name = cos[j]["OperationalFrame"]["Name"].as<std::string>();
                            std::string type = cos[j]["OperationalFrame"]["Type"].as<std::string>();

                            // Store all the relevant information that in this case is: taskdof (co) and jointdof (robot)!
                            // vec_dimensions_.push_back(std::make_pair(taskdof, jointDOF));
                            this->addRobot(taskdof, jointDOF);
                        }
                    }
                }
            }
        }
    }

    // this->calculateTotalDofs();
    return true;
}

void RTTCartPIDController::addRobot(unsigned int task_dof, unsigned int joint_dof)
{
    vec_dimensions_.push_back(std::make_pair(task_dof, joint_dof));
}

void RTTCartPIDController::calculateTotalDofs()
{
    total_dof_size_ = 0;
    TaskSpaceDimension = 0;
    TaskSpaceQuaternionDimension = 0;
    for (unsigned int i = 0; i < vec_dimensions_.size(); i++)
    {
        TaskSpaceDimension += vec_dimensions_[i].first;
        total_dof_size_ += vec_dimensions_[i].second;
        // This cannot be anything else, because in this controller we are specifically working with 6 DOF Operational Space!
        TaskSpaceQuaternionDimension += 7;
    }

    errorPosition = Eigen::VectorXd::Zero(TaskSpaceDimension);
    errorVelocity = Eigen::VectorXd::Zero(TaskSpaceDimension);

    lambda_global_ = Eigen::MatrixXd::Zero(TaskSpaceDimension, TaskSpaceDimension);
    svd_solver_lambda_c_global_ = Eigen::JacobiSVD<Eigen::MatrixXd>(TaskSpaceDimension, TaskSpaceDimension);
    singular_values_lambda_c_global_.resize(TaskSpaceDimension);

    lambda_.clear();
    singular_values_lambda_c_.clear();
    svd_solver_lambda_c_.clear();

    gainTranslationP_.clear();
    gainTranslationD_.clear();
    gainOrientationP_.clear();
    gainOrientationD_.clear();
    for (unsigned int i = 0; i < vec_dimensions_.size(); i++)
    {
        gainTranslationP_.push_back(Eigen::VectorXd::Zero(3));
        gainTranslationD_.push_back(Eigen::VectorXd::Zero(3));
        gainOrientationP_.push_back(Eigen::VectorXd::Zero(3));
        gainOrientationD_.push_back(Eigen::VectorXd::Zero(3));

        lambda_.push_back(Eigen::MatrixXd::Zero(vec_dimensions_[i].first, vec_dimensions_[i].first));
        svd_solver_lambda_c_.push_back(Eigen::JacobiSVD<Eigen::MatrixXd>(vec_dimensions_[i].first, vec_dimensions_[i].first));
        singular_values_lambda_c_.push_back(Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType(vec_dimensions_[i].first));
    }
}

void RTTCartPIDController::setNoCommandReceivedBehavior(std::string const &type)
{
    if (type.compare("nothing") == 0)
    {
        noCommandReceivedBehaviorType = 0;
    }
    else if (type.compare("holdposition") == 0)
    {
        noCommandReceivedBehaviorType = 1;
    }
    else if (type.compare("gravity") == 0)
    {
        noCommandReceivedBehaviorType = 2;
    }
}

bool RTTCartPIDController::configureHook()
{
    return true;
}

bool RTTCartPIDController::startHook()
{
    // When the controller is started and did not receive a command then it should hold the current position.
    hold_current_position = true;
    hold_current_position_last = false;

    // Check for mandatory ports to be connected.
    if (!in_robotstatus_port.connected())
    {
        RTT::log(RTT::Error) << "in_robotstatus_port not connected"
                             << RTT::endlog();
        return false;
    }

    // if (!in_desiredTaskSpacePosition_port.connected())
    // {
    //     RTT::log(RTT::Error) << "in_desiredTaskSpacePosition_port not connected"
    //                          << RTT::endlog();
    //     return false;
    // }

    // if (!in_desiredTaskSpaceVelocity_port.connected())
    // {
    //     RTT::log(RTT::Error) << "in_desiredTaskSpaceVelocity_port not connected"
    //                          << RTT::endlog();
    //     return false;
    // }

    // if (!in_desiredTaskSpaceAcceleration_port.connected())
    // {
    //     RTT::log(RTT::Error) << "in_desiredTaskSpaceAcceleration_port not connected"
    //                          << RTT::endlog();
    //     return false;
    // }

    if (!in_currentTaskSpacePosition_port.connected())
    {
        RTT::log(RTT::Error) << "in_currentTaskSpacePosition_port not connected"
                             << RTT::endlog();
        return false;
    }

    if (!in_currentTaskSpaceVelocity_port.connected())
    {
        RTT::log(RTT::Error) << "in_currentTaskSpaceVelocity_port not connected"
                             << RTT::endlog();
        return false;
    }

    if (!in_jacobian_port.connected())
    {
        RTT::log(RTT::Error) << "in_jacobian_port not connected"
                             << RTT::endlog();
        return false;
    }

    if (!in_jacobianDot_port.connected())
    {
        RTT::log(RTT::Error) << "in_jacobianDot_port not connected"
                             << RTT::endlog();
        return false;
    }

    if (!in_coriolisAndGravity_port.connected())
    {
        RTT::log(RTT::Error) << "in_coriolisAndGravity_port not connected"
                             << RTT::endlog();
        return false;
    }

    if (!in_inertia_port.connected())
    {
        RTT::log(RTT::Error) << "in_inertia_port not connected"
                             << RTT::endlog();
        return false;
    }

    return true;
}

void RTTCartPIDController::updateHook()
{
    // Read data from mandatory ports
    in_desiredTaskSpacePosition_flow = in_desiredTaskSpace_port.read(in_desiredTaskSpace_var);
    // in_desiredTaskSpacePosition_flow = in_desiredTaskSpacePosition_port.read(in_desiredTaskSpacePosition_var);
    // in_desiredTaskSpaceVelocity_flow = in_desiredTaskSpaceVelocity_port.read(in_desiredTaskSpaceVelocity_var);
    // in_desiredTaskSpaceAcceleration_flow = in_desiredTaskSpaceAcceleration_port.read(in_desiredTaskSpaceAcceleration_var);

    in_currentTaskSpacePosition_flow = in_currentTaskSpacePosition_port.read(in_currentTaskSpacePosition_var);

    in_currentTaskSpaceVelocity_flow = in_currentTaskSpaceVelocity_port.read(in_currentTaskSpaceVelocity_var);

    in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);

    in_jacobian_flow = in_jacobian_port.read(in_jacobian_var);

    in_jacobianDot_flow = in_jacobianDot_port.read(in_jacobianDot_var);

    in_coriolisAndGravity_flow = in_coriolisAndGravity_port.read(in_coriolisAndGravity_var);

    in_inertia_flow = in_inertia_port.read(in_inertia_var);

    // Read data from optional ports
    if (in_projection_port.connected())
    {
        in_projection_flow = in_projection_port.read(in_projection_var);
    }
    if (in_projectionDot_port.connected())
    {
        in_projectionDot_flow = in_projectionDot_port.read(in_projectionDot_var);
    }

    // Handle hold position behavior when no command was received yet.
    if ((in_desiredTaskSpacePosition_flow == RTT::NoData) || (in_desiredTaskSpaceVelocity_flow == RTT::NoData) || (in_desiredTaskSpaceAcceleration_flow == RTT::NoData))
    {
        if (noCommandReceivedBehaviorType == 0)
        {
            return;
        }
        hold_current_position = true;
    }
    else
    {
        hold_current_position = false;
    }

    // in_desiredTaskSpacePosition_var_eig(0) = in_desiredTaskSpacePosition_var.position.x;
    // in_desiredTaskSpacePosition_var_eig(1) = in_desiredTaskSpacePosition_var.position.y;
    // in_desiredTaskSpacePosition_var_eig(2) = in_desiredTaskSpacePosition_var.position.z;
    // in_desiredTaskSpacePosition_var_eig(3) = in_desiredTaskSpacePosition_var.orientation.w;
    // in_desiredTaskSpacePosition_var_eig(4) = in_desiredTaskSpacePosition_var.orientation.x;
    // in_desiredTaskSpacePosition_var_eig(5) = in_desiredTaskSpacePosition_var.orientation.y;
    // in_desiredTaskSpacePosition_var_eig(6) = in_desiredTaskSpacePosition_var.orientation.z;

    // in_desiredTaskSpaceVelocity_var_eig(0) = in_desiredTaskSpaceVelocity_var.linear.x;
    // in_desiredTaskSpaceVelocity_var_eig(1) = in_desiredTaskSpaceVelocity_var.linear.y;
    // in_desiredTaskSpaceVelocity_var_eig(2) = in_desiredTaskSpaceVelocity_var.linear.z;
    // in_desiredTaskSpaceVelocity_var_eig(3) = in_desiredTaskSpaceVelocity_var.angular.x;
    // in_desiredTaskSpaceVelocity_var_eig(4) = in_desiredTaskSpaceVelocity_var.angular.y;
    // in_desiredTaskSpaceVelocity_var_eig(5) = in_desiredTaskSpaceVelocity_var.angular.z;

    // in_desiredTaskSpaceAcceleration_var_eig(0) = in_desiredTaskSpaceAcceleration_var.linear.x;
    // in_desiredTaskSpaceAcceleration_var_eig(1) = in_desiredTaskSpaceAcceleration_var.linear.y;
    // in_desiredTaskSpaceAcceleration_var_eig(2) = in_desiredTaskSpaceAcceleration_var.linear.z;
    // in_desiredTaskSpaceAcceleration_var_eig(3) = in_desiredTaskSpaceAcceleration_var.angular.x;
    // in_desiredTaskSpaceAcceleration_var_eig(4) = in_desiredTaskSpaceAcceleration_var.angular.y;
    // in_desiredTaskSpaceAcceleration_var_eig(5) = in_desiredTaskSpaceAcceleration_var.angular.z;

    in_desiredTaskSpacePosition_var_eig(0) = in_desiredTaskSpace_var.transforms[0].translation.x;
    in_desiredTaskSpacePosition_var_eig(1) = in_desiredTaskSpace_var.transforms[0].translation.y;
    in_desiredTaskSpacePosition_var_eig(2) = in_desiredTaskSpace_var.transforms[0].translation.z;
    in_desiredTaskSpacePosition_var_eig(3) = in_desiredTaskSpace_var.transforms[0].rotation.w;
    in_desiredTaskSpacePosition_var_eig(4) = in_desiredTaskSpace_var.transforms[0].rotation.x;
    in_desiredTaskSpacePosition_var_eig(5) = in_desiredTaskSpace_var.transforms[0].rotation.y;
    in_desiredTaskSpacePosition_var_eig(6) = in_desiredTaskSpace_var.transforms[0].rotation.z;

    in_desiredTaskSpaceVelocity_var_eig(0) = in_desiredTaskSpace_var.velocities[0].linear.x;
    in_desiredTaskSpaceVelocity_var_eig(1) = in_desiredTaskSpace_var.velocities[0].linear.y;
    in_desiredTaskSpaceVelocity_var_eig(2) = in_desiredTaskSpace_var.velocities[0].linear.z;
    in_desiredTaskSpaceVelocity_var_eig(3) = in_desiredTaskSpace_var.velocities[0].angular.x;
    in_desiredTaskSpaceVelocity_var_eig(4) = in_desiredTaskSpace_var.velocities[0].angular.y;
    in_desiredTaskSpaceVelocity_var_eig(5) = in_desiredTaskSpace_var.velocities[0].angular.z;

    in_desiredTaskSpaceAcceleration_var_eig(0) = in_desiredTaskSpace_var.accelerations[0].linear.x;
    in_desiredTaskSpaceAcceleration_var_eig(1) = in_desiredTaskSpace_var.accelerations[0].linear.y;
    in_desiredTaskSpaceAcceleration_var_eig(2) = in_desiredTaskSpace_var.accelerations[0].linear.z;
    in_desiredTaskSpaceAcceleration_var_eig(3) = in_desiredTaskSpace_var.accelerations[0].angular.x;
    in_desiredTaskSpaceAcceleration_var_eig(4) = in_desiredTaskSpace_var.accelerations[0].angular.y;
    in_desiredTaskSpaceAcceleration_var_eig(5) = in_desiredTaskSpace_var.accelerations[0].angular.z;

    // TODO handle ill-sized dimensions of input data.
    if (in_desiredTaskSpacePosition_flow != RTT::NoData)
    {
        // if (in_desiredTaskSpacePosition_var.rows() != TaskSpaceQuaternionDimension)
        // {
        //     // RTT::log(RTT::Error) << "ERROR in_desiredTaskSpacePosition_var " << in_desiredTaskSpacePosition_var << RTT::endlog();
        // }
        // if (in_desiredTaskSpacePosition_var.cols() != 1)
        // {
        //     // RTT::log(RTT::Error) << "ERROR in_desiredTaskSpacePosition_var " << in_desiredTaskSpacePosition_var << RTT::endlog();
        // }
    }
    if (in_desiredTaskSpaceVelocity_flow != RTT::NoData)
    {
        // if (in_desiredTaskSpaceVelocity_var.rows() != TaskSpaceDimension)
        // {
        //     // RTT::log(RTT::Error) << "ERROR in_desiredTaskSpaceVelocity_var " << in_desiredTaskSpaceVelocity_var << RTT::endlog();
        // }
        // if (in_desiredTaskSpaceVelocity_var.cols() != 1)
        // {
        //     // RTT::log(RTT::Error) << "ERROR in_desiredTaskSpaceVelocity_var " << in_desiredTaskSpaceVelocity_var << RTT::endlog();
        // }
    }
    if (in_desiredTaskSpaceAcceleration_flow != RTT::NoData)
    {
        // if (in_desiredTaskSpaceAcceleration_var.rows() != TaskSpaceDimension)
        // {
        //     // RTT::log(RTT::Error) << "ERROR in_desiredTaskSpaceAcceleration_var " << in_desiredTaskSpaceAcceleration_var << RTT::endlog();
        // }
        // if (in_desiredTaskSpaceAcceleration_var.cols() != 1)
        // {
        //     // RTT::log(RTT::Error) << "ERROR in_desiredTaskSpaceAcceleration_var " << in_desiredTaskSpaceAcceleration_var << RTT::endlog();
        // }
    }

    // Return and do not calculate a robot command if mandatory data is missing.
    if (in_currentTaskSpacePosition_flow == RTT::NoData)
    {
        // RTT::log(RTT::Error) << "in_currentTaskSpacePosition_flow return??" << RTT::endlog();
        return;
    }
    if (in_currentTaskSpaceVelocity_flow == RTT::NoData)
    {
        // RTT::log(RTT::Error) << "in_currentTaskSpaceVelocity_flow return??" << RTT::endlog();
        return;
    }
    if (in_robotstatus_flow == RTT::NoData)
    {
        // RTT::log(RTT::Error) << "in_robotstatus_flow return??" << RTT::endlog();
        return;
    }
    if (in_jacobian_flow == RTT::NoData)
    {
        // RTT::log(RTT::Error) << "in_jacobian_flow return??" << RTT::endlog();
        return;
    }
    if (in_jacobianDot_flow == RTT::NoData)
    {
        // RTT::log(RTT::Error) << "in_jacobianDot_flow return??" << RTT::endlog();
        return;
    }
    if (in_coriolisAndGravity_flow == RTT::NoData)
    {
        // RTT::log(RTT::Error) << "in_coriolisAndGravity_flow return??" << RTT::endlog();
        return;
    }

    if (in_inertia_flow == RTT::NoData)
    {
        // RTT::log(RTT::Error) << "in_inertia_flow return??" << RTT::endlog();
        return;
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

    // TODO Handle ill-sized data.
    if (in_currentTaskSpacePosition_var.rows() != TaskSpaceQuaternionDimension)
    {
        // RTT::log(RTT::Error) << "ERROR in_currentTaskSpacePosition_var " << in_currentTaskSpacePosition_var << RTT::endlog();
    }
    if (in_currentTaskSpacePosition_var.cols() != 1)
    {
        // RTT::log(RTT::Error) << "ERROR in_currentTaskSpacePosition_var " << in_currentTaskSpacePosition_var << RTT::endlog();
    }

    if (in_currentTaskSpaceVelocity_var.rows() != TaskSpaceDimension)
    {
        // RTT::log(RTT::Error) << "ERROR in_currentTaskSpaceVelocity_var " << in_currentTaskSpaceVelocity_var << RTT::endlog();
    }
    if (in_currentTaskSpaceVelocity_var.cols() != 1)
    {
        // RTT::log(RTT::Error) << "ERROR in_currentTaskSpaceVelocity_var " << in_currentTaskSpaceVelocity_var << RTT::endlog();
    }

    if (in_robotstatus_var.position.size() != total_dof_size_)
    {
        // RTT::log(RTT::Error) << "ERROR in_robotstatus_var.angles " << in_robotstatus_var.angles << RTT::endlog();
    }
    if (in_robotstatus_var.velocity.size() != total_dof_size_)
    {
        // RTT::log(RTT::Error) << "ERROR in_robotstatus_var.velocities " << in_robotstatus_var.velocities << RTT::endlog();
    }
    if (in_robotstatus_var.effort.size() != total_dof_size_)
    {
        // RTT::log(RTT::Error) << "ERROR in_robotstatus_var.torques " << in_robotstatus_var.torques << RTT::endlog();
    }

    if (in_jacobian_var.rows() != TaskSpaceDimension)
    {
        // RTT::log(RTT::Error) << "ERROR in_jacobian_var " << in_jacobian_var << RTT::endlog();
    }
    if (in_jacobian_var.cols() != total_dof_size_)
    {
        // RTT::log(RTT::Error) << "ERROR in_jacobian_var " << in_jacobian_var << RTT::endlog();
    }

    if (in_jacobianDot_var.rows() != TaskSpaceDimension)
    {
        // RTT::log(RTT::Error) << "ERROR in_jacobianDot_var " << in_jacobianDot_var << RTT::endlog();
    }
    if (in_jacobianDot_var.cols() != total_dof_size_)
    {
        // RTT::log(RTT::Error) << "ERROR in_jacobianDot_var " << in_jacobianDot_var << RTT::endlog();
    }

    if (in_coriolisAndGravity_var.rows() != total_dof_size_)
    {
        // RTT::log(RTT::Error) << "ERROR in_coriolisAndGravity_var " << in_coriolisAndGravity_var << RTT::endlog();
    }
    if (in_coriolisAndGravity_var.cols() != 1)
    {
        // RTT::log(RTT::Error) << "ERROR in_coriolisAndGravity_var " << in_coriolisAndGravity_var << RTT::endlog();
    }

    if (in_inertia_var.rows() != total_dof_size_)
    {
        // RTT::log(RTT::Error) << "ERROR in_inertia_var " << in_inertia_var << RTT::endlog();
    }
    if (in_inertia_var.cols() != total_dof_size_)
    {
        // RTT::log(RTT::Error) << "ERROR in_inertia_var " << in_inertia_var << RTT::endlog();
    }

    if (in_projection_var.rows() != total_dof_size_)
    {
        // RTT::log(RTT::Error) << "ERROR in_projection_var " << in_projection_var << RTT::endlog();
    }
    if (in_projection_var.cols() != total_dof_size_)
    {
        // RTT::log(RTT::Error) << "ERROR in_projection_var " << in_projection_var << RTT::endlog();
    }

    if (in_projectionDot_var.rows() != total_dof_size_)
    {
        // RTT::log(RTT::Error) << "ERROR in_projectionDot_var " << in_projectionDot_var << RTT::endlog();
    }
    if (in_projectionDot_var.cols() != total_dof_size_)
    {
        // RTT::log(RTT::Error) << "ERROR in_projectionDot_var " << in_projectionDot_var << RTT::endlog();
    }

    // If no command received, hold position.
    if (hold_current_position)
    {
        if (!hold_current_position_last && noCommandReceivedBehaviorType == 1)
        {
            RTT::log(RTT::Error) << "Holding current position until a command is received." << RTT::endlog();
            in_desiredTaskSpacePosition_var_eig = in_currentTaskSpacePosition_var;
            //
            // in_desiredTaskSpaceVelocity_var = in_currentTaskSpaceVelocity_var;
            //
            in_desiredTaskSpaceVelocity_var_eig.setZero();
            //
            in_desiredTaskSpacePosition_proxy = in_desiredTaskSpacePosition_var_eig;
            in_desiredTaskSpaceAcceleration_var_eig.setZero();
        }
        else if (noCommandReceivedBehaviorType == 2)
        {
            out_torques_var = in_coriolisAndGravity_var;
            out_force_var = in_jacobian_var * in_coriolisAndGravity_var;
            out_torques_port.write(out_torques_var);
            out_force_port.write(out_force_var);
            return;
        }
    }
    else if (hold_current_position_last)
    {
        RTT::log(RTT::Error) << "Command received proceed normally." << RTT::endlog();
    }
    hold_current_position_last = hold_current_position;

    // Compute the actual robot command.
    this->compute(in_desiredTaskSpacePosition_var_eig,
                  in_desiredTaskSpaceVelocity_var_eig,
                  in_desiredTaskSpaceAcceleration_var_eig,
                  in_currentTaskSpacePosition_var,
                  in_currentTaskSpaceVelocity_var,
                  in_robotstatus_var,
                  in_jacobian_var,
                  in_jacobianDot_var,
                  in_coriolisAndGravity_var,
                  in_inertia_var,
                  in_projection_var,
                  in_projectionDot_var,
                  out_torques_var,
                  out_force_var);

    // Write the torque as well as the force commands.
    out_torques_port.write(out_torques_var);
    out_force_port.write(out_force_var);

    this->out_error_position_port.write(this->errorPosition);
    this->out_error_velocity_port.write(this->errorVelocity);
}

void RTTCartPIDController::stopHook()
{
}

void RTTCartPIDController::cleanupHook()
{
    portsArePrepared = false;
}

void RTTCartPIDController::compute(
    Eigen::VectorXd &in_desiredTaskSpacePosition,
    Eigen::VectorXd &in_desiredTaskSpaceVelocity,
    Eigen::VectorXd &in_desiredTaskSpaceAcceleration,
    Eigen::VectorXd &in_currentTaskSpacePosition,
    Eigen::VectorXd &in_currentTaskSpaceVelocity,
    sensor_msgs::JointState &in_robotstatus,
    Eigen::MatrixXd &in_jacobian,
    Eigen::MatrixXd &in_jacobianDot,
    Eigen::VectorXd &in_coriolisAndGravity,
    Eigen::MatrixXd &in_inertia,
    Eigen::MatrixXd &in_projection,
    Eigen::MatrixXd &in_projectionDot,
    Eigen::VectorXd &out_torques,
    Eigen::VectorXd &out_force)
{
    // Convert ROS std type to Eigen
    Eigen::VectorXd fdb_positions = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(in_robotstatus.position.data(), in_robotstatus.position.size());
    Eigen::VectorXd fdb_velocities = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(in_robotstatus.velocity.data(), in_robotstatus.velocity.size());
    // Eigen::VectorXd fdb_efforts = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(in_robotstatus.effort.data(), in_robotstatus.effort.size());

    if (in_currentTaskSpacePosition.hasNaN())
    {
        RTT::log(RTT::Error) << "in_currentTaskSpacePosition NAN!=\n"
                             << in_currentTaskSpacePosition << RTT::endlog();
    }

    Eigen::MatrixXd in_inertiaInv = in_inertia.inverse();

    // TODO don't send anything.
    if (in_inertiaInv.isZero())
    {
        RTT::log(RTT::Error) << this->getName() << ": in_inertiaInv is zero..." << RTT::endlog();
        out_torques.setZero();
        out_force.setZero();
        return;
    }

    // TODO don't send anything.
    if (in_projection.isZero())
    {
        RTT::log(RTT::Error) << this->getName() << ": in_projection is zero..." << RTT::endlog();
        out_torques.setZero();
        out_force.setZero();
        return;
    }

    errorPosition.setZero();
    errorVelocity.setZero();

    for (unsigned int i = 0; i < vec_dimensions_.size(); i++)
    {
        errorTranslationPosition.setZero();
        errorTranslationVelocity.setZero();
        desiredPosition = in_desiredTaskSpacePosition.segment<3>(WorkspaceQuaternionDimension * i);
        currentPosition = in_currentTaskSpacePosition.segment<3>(WorkspaceQuaternionDimension * i);
        desiredVelocity = in_desiredTaskSpaceVelocity.segment<3>(WorkspaceDimension * i);
        currentVelocity = in_currentTaskSpaceVelocity.segment<3>(WorkspaceDimension * i);

        this->computeTranslationError(desiredPosition, currentPosition, desiredVelocity, currentVelocity,
                                      errorTranslationPosition, errorTranslationVelocity);

        errorPosition.segment<3>(WorkspaceDimension * i) = gainTranslationP_[i].cwiseProduct(errorTranslationPosition);
        errorVelocity.segment<3>(WorkspaceDimension * i) = gainTranslationD_[i].cwiseProduct(errorTranslationVelocity);

        errorOrientationPosition.setZero();
        errorOrientationVelocity.setZero();

        // Do not change the orientation and take the one that was received at the very beginning. For Debugging purposes.
        if (!lockOrientation)
        {
            in_desiredTaskSpacePosition_proxy = in_desiredTaskSpacePosition;
        }
        desiredQuaternionPosition = in_desiredTaskSpacePosition_proxy.segment<4>(WorkspaceQuaternionDimension * i + 3); // These are quaternions.

        currentQuaternionPosition = in_currentTaskSpacePosition.segment<4>(WorkspaceQuaternionDimension * i + 3);
        desiredVelocity = in_desiredTaskSpaceVelocity.segment<3>(WorkspaceDimension * i + 3); // These are euler or axis angles.
        currentVelocity = in_currentTaskSpaceVelocity.segment<3>(WorkspaceDimension * i + 3);

        this->computeOrientationError(desiredQuaternionPosition, currentQuaternionPosition, desiredVelocity, currentVelocity,
                                      errorOrientationPosition, errorOrientationVelocity);

        errorPosition.segment<3>(WorkspaceDimension * i + 3) = gainOrientationP_[i].cwiseProduct(errorOrientationPosition);
        errorVelocity.segment<3>(WorkspaceDimension * i + 3) = gainOrientationD_[i].cwiseProduct(errorOrientationVelocity);
    }

    /**
     * Start Khatib projected endeffector motion controller in case of free motion without contact-constraint we assume that
     *  - in_projection is equal to identity,
     *  - in_projectionDot is zero.
     */
    // Set output vars to zero.
    out_torques.setZero();
    out_force.setZero();

    // Calculate aux var: M^-1 * P
    inertiaInvP = in_inertiaInv * in_projection;

    for (unsigned int i = 0; i < vec_dimensions_.size(); i++)
    {
        if (in_jacobian.block(WorkspaceDimension * i, 0, 6, total_dof_size_).isZero())
        {
            lambda_[i].setZero();
            out_force.segment<6>(WorkspaceDimension * i).setZero();
        }
        else
        {
            // ################################################################################################################
            // THIS IS CRITICAL BECAUSE SOMETIMES WE CANNOT INVERT
            // lambda = (in_jacobian * inertiaInvP * in_jacobian.transpose()).inverse();

            // Calculate lambda_c
            lambda_[i] = in_jacobian.block(WorkspaceDimension * i, 0, 6, total_dof_size_) * inertiaInvP * in_jacobian.block(WorkspaceDimension * i, 0, 6, total_dof_size_).transpose();
            svd_solver_lambda_c_[i].compute(lambda_[i], Eigen::ComputeFullU | Eigen::ComputeFullV);

            singular_values_lambda_c_[i] = svd_solver_lambda_c_[i].singularValues();
            for (int k = 0; k < singular_values_lambda_c_[i].size(); k++)
            {
                if (singular_values_lambda_c_[i](k) < 1.e-06)
                {
                    singular_values_lambda_c_[i](k) = 0;
                }
                else
                {
                    singular_values_lambda_c_[i](k) = 1 / singular_values_lambda_c_[i](k);
                }
                //if (singular_values_lambda_c(k) > 1.e-03){
                //		rank_mc = rank_mc + 1 ;
                //}
            }

            lambda_[i] = svd_solver_lambda_c_[i].matrixV().leftCols(singular_values_lambda_c_[i].size()) * singular_values_lambda_c_[i].asDiagonal() * svd_solver_lambda_c_[i].matrixU().leftCols(singular_values_lambda_c_[i].size()).transpose();
            // ################################################################################################################

            // ################################################# Controller ###################################################
            if (!impedanceCTRL)
            {
                // Inverse dynamics control law.
                out_force.segment<6>(WorkspaceDimension * i) = lambda_[i] * (in_desiredTaskSpaceAcceleration.segment<6>(WorkspaceDimension * i) + errorPosition.segment<6>(WorkspaceDimension * i) + errorVelocity.segment<6>(WorkspaceDimension * i));
            }
            else
            {
                // Impedance control law.
                out_force.segment<6>(WorkspaceDimension * i) = lambda_[i] * in_desiredTaskSpaceAcceleration.segment<6>(WorkspaceDimension * i) + errorPosition.segment<6>(WorkspaceDimension * i) + errorVelocity.segment<6>(WorkspaceDimension * i);
            }
            // TODO TS Gravity included (verify if properly done!)
            if (use_TSgravitycompensation)
            {
                out_force.segment<6>(WorkspaceDimension * i) += (lambda_[i] * in_jacobian.block(WorkspaceDimension * i, 0, 6, total_dof_size_) * in_inertiaInv * (in_projection * in_coriolisAndGravity - in_projectionDot * fdb_velocities)) - (lambda_[i] * in_jacobianDot.block(WorkspaceDimension * i, 0, 6, total_dof_size_) * fdb_velocities);
            }

            // if (use_TSgravitycompensation)
            // {
            //     out_force += lambda * (in_jacobian * inertiaInvP * in_coriolisAndGravity) + lambda * (-in_jacobianDot - in_jacobian * in_inertiaInv * in_projectionDot) * fdb_velocities;
            // }
            // ################################################################################################################
        }
    }

    // ########################################## DLW TEST COMPENSATION FOR GRASPED OBJECT

    // torque in the un-constrained space
    if (add_JSgravitycompensation)
    {
        out_torques = in_projection * in_jacobian.transpose() * out_force + in_coriolisAndGravity;
    }
    else
    {
        out_torques = in_projection * in_jacobian.transpose() * out_force;
    }

    // Eigen::VectorXd ttt = (nP * (pos_rest - fdb_positions) - nD * fdb_velocities + in_coriolisAndGravity);
    // if (also_debug_2)
    // {
    //     // TODO DLW TEST
    //     int _dofsize = 7;
    //     int _numTasks = 2;

    //     //choose dimensionality of each task
    //     Eigen::VectorXd _tasksize;
    //     _tasksize = Eigen::VectorXd::Zero(_numTasks);
    //     //in this example "task0" and "task1" already consume all degress of freedom
    //     _tasksize[0] = 6;
    //     _tasksize[1] = 7;

    //     //initialize GHC class
    //     GHCProjections GP;
    //     GP = GHCProjections();
    //     GP.init(_numTasks, _tasksize, _dofsize);

    //     // RTT::log(RTT::Error) << "### 1 ###: A =\n" << GP.getAlphas() << RTT::endlog();

    //     std::vector<Eigen::MatrixXd> allJacobians;
    //     allJacobians.push_back(in_jacobian);
    //     allJacobians.push_back(Eigen::MatrixXd::Identity(7, 7));
    //     GP.setJacobianMatrices(allJacobians);

    //     GP.setInertiaMatrix(in_inertia);

    //     Eigen::VectorXd prioritiesVector;
    //     prioritiesVector = Eigen::VectorXd::Zero(0.5 * (_numTasks * _numTasks + _numTasks));
    //     //exampleA: strict hierachy with "task0" strict more important that "task1" and "task1" strict more important that "task2"
    //     prioritiesVector[0] = 0.0;
    //     prioritiesVector[1] = 0.0;
    //     prioritiesVector[2] = 0.0;
    //     // prioritiesVector[3] = 0.0;
    //     // prioritiesVector[4] = 0.0;
    //     // prioritiesVector[5] = 0.0;

    //     // //exampleB: strict hierachy with "task2" strict more important that "task1" and "task1" strict more important that "task0"
    //     // prioritiesVector[0] = 0.0;
    //     // prioritiesVector[1] = 1.0;
    //     // prioritiesVector[2] = 1.0;
    //     // prioritiesVector[3] = 0.0;
    //     // prioritiesVector[4] = 1.0;
    //     // prioritiesVector[5] = 0.0;
    //     // //exampleC: all tasks with equal (soft) priority
    //     // prioritiesVector[0] = 0.0;
    //     // prioritiesVector[1] = 0.5;
    //     // prioritiesVector[2] = 0.5;
    //     // prioritiesVector[3] = 0.0;
    //     // prioritiesVector[4] = 0.5;
    //     // prioritiesVector[5] = 0.0;
    //     int counter = 0;
    //     for (unsigned int i = 0; i < _numTasks; i++)
    //     {
    //         for (unsigned int j = i; j < _numTasks; j++)
    //         {
    //             if (prioritiesVector(counter) < 0)
    //             {
    //                 std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector(counter) << " < 0" << std::endl;
    //                 // return;
    //             }
    //             if (prioritiesVector(counter) > 1)
    //             {
    //                 std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector(counter) << " > 1" << std::endl;
    //                 // return;
    //             }
    //             GP.setAlphaIJ(i, j, prioritiesVector(counter));
    //             // RTT::log(RTT::Error) << "### 2 ######>: set i = " << i << ", j = " << j << " : " << prioritiesVector(counter) << RTT::endlog();
    //             counter++;
    //         }
    //     }

    //     // RTT::log(RTT::Error) << "### 3 ###: A =\n" << GP.getAlphas() << RTT::endlog();

    //     std::vector<Eigen::MatrixXd> allProjections;
    //     for (unsigned int i = 0; i < _numTasks; i++)
    //     {
    //         allProjections.push_back(Eigen::MatrixXd::Zero(_dofsize, _dofsize));
    //     }
    //     Eigen::VectorXd ranks;
    //     ranks = Eigen::VectorXd::Zero(_numTasks);
    //     bool ok = GP.getAllGeneralizedProjectors(allProjections, ranks);

    //     RTT::log(RTT::Error) << "inertiaInv 333 =\n"
    //                          << in_inertiaInv << RTT::endlog();
    //     RTT::log(RTT::Error) << "inertia 333 =\n"
    //                          << in_inertia << RTT::endlog();
    //     RTT::log(RTT::Error) << "jac 333 =\n"
    //                          << in_jacobian << RTT::endlog();
    //     RTT::log(RTT::Error) << "grav 333 =\n"
    //                          << in_coriolisAndGravity << RTT::endlog();
    //     RTT::log(RTT::Error) << "nullspace 333 =\n"
    //                          << allProjections[1] << RTT::endlog();
    //     RTT::log(RTT::Error) << "trq task 333 =\n"
    //                          << out_torques.torques << RTT::endlog();
    //     RTT::log(RTT::Error) << "trq ns 333 =\n"
    //                          << ttt << RTT::endlog();
    //     RTT::log(RTT::Error) << "nullspace * trq 333 =\n"
    //                          << (allProjections[1] * ttt) << RTT::endlog();
    //     if (also_debug_switch == 2)
    //     {
    //         out_torques.torques += allProjections[1] * ttt;
    //     }
    //     else if (also_debug_switch == 3)
    //     {
    //         out_torques.torques = allProjections[0] * out_torques.torques + allProjections[1] * ttt;
    //     }
    //     RTT::log(RTT::Error) << "out_torques.torques 333 =\n"
    //                          << (out_torques.torques + allProjections[1] * ttt) << RTT::endlog();
    // }

    // if (also_debug_1)
    // {
    //     lambda_global_ = (in_jacobian * inertiaInvP * in_jacobian.transpose());
    //     svd_solver_lambda_c_global_.compute(lambda_global_, Eigen::ComputeFullU | Eigen::ComputeFullV);

    //     singular_values_lambda_c_global_ = svd_solver_lambda_c_global_.singularValues();
    //     for (int i = 0; i < singular_values_lambda_c_global_.size(); i++)
    //     {
    //         if (singular_values_lambda_c_global_(i) < 1.e-06)
    //         {
    //             singular_values_lambda_c_global_(i) = 0;
    //         }
    //         else
    //         {
    //             singular_values_lambda_c_global_(i) = 1 / singular_values_lambda_c_global_(i);
    //         }
    //     }

    //     lambda_global_ = svd_solver_lambda_c_global_.matrixV().leftCols(singular_values_lambda_c_global_.size()) * singular_values_lambda_c_global_.asDiagonal() * svd_solver_lambda_c_global_.matrixU().leftCols(singular_values_lambda_c_global_.size()).transpose();
    //     nullspace = (Eigen::MatrixXd::Identity(total_dof_size_, total_dof_size_) - (in_jacobian.transpose() * lambda_global_ * in_jacobian * inertiaInvP));
    //     // DLW TEST

    //     RTT::log(RTT::Error) << "in_projection 222 =\n"
    //                          << in_projection << RTT::endlog();
    //     RTT::log(RTT::Error) << "in_inertiaInv 222 =\n"
    //                          << in_inertiaInv << RTT::endlog();
    //     RTT::log(RTT::Error) << "in_inertiaInv * in_projection 222 =\n"
    //                          << (in_inertiaInv * in_projection) << RTT::endlog();
    //     RTT::log(RTT::Error) << "nullspace 222 =\n"
    //                          << nullspace << RTT::endlog();
    //     RTT::log(RTT::Error) << "trq 222 =\n"
    //                          << ttt << RTT::endlog();
    //     RTT::log(RTT::Error) << "nullspace * trq 222 =\n"
    //                          << (nullspace * ttt) << RTT::endlog();
    //     if (also_debug_switch == 1)
    //     {
    //         out_torques.torques += nullspace * ttt;
    //     }
    //     RTT::log(RTT::Error) << "out_torques.torques 222 =\n"
    //                          << (out_torques.torques + nullspace * ttt) << RTT::endlog();
    // }
}

// bool RTTCartPIDController::setDOFsize(unsigned int DOFsize)
// {
//     if (DOFsize <= 0)
//     {
//         RTT::log(RTT::Error) << "DOFsize needs to be > 0." << RTT::endlog();
//         return false;
//     }
//     this->total_dof_size_ = DOFsize;

//     return true;
// }

void RTTCartPIDController::useTSgravitycompensation(bool useTSgravitycompensation)
{
    this->use_TSgravitycompensation = useTSgravitycompensation;
}

void RTTCartPIDController::addJSgravitycompensation(bool addJSgravitycompensation)
{
    this->add_JSgravitycompensation = addJSgravitycompensation;
}

bool RTTCartPIDController::setGains(double kp, double kd)
{
    if (vec_dimensions_.size() <= 0)
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "Number of manipulators is zero, cannot set translation gains." << RTT::endlog();
        return false;
    }
    for (unsigned int i = 0; i < vec_dimensions_.size(); i++)
    {
        gainTranslationP_[i](0) = kp;
        gainTranslationP_[i](1) = kp;
        gainTranslationP_[i](2) = kp;
        //
        gainTranslationD_[i](0) = kd;
        gainTranslationD_[i](1) = kd;
        gainTranslationD_[i](2) = kd;
    }
    return true;
}

bool RTTCartPIDController::setGainsOrientation(double kp, double kd)
{
    if (vec_dimensions_.size() <= 0)
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "Number of manipulators is zero, cannot set orientation gains." << RTT::endlog();
        return false;
    }
    for (unsigned int i = 0; i < vec_dimensions_.size(); i++)
    {
        gainOrientationP_[i](0) = kp;
        gainOrientationP_[i](1) = kp;
        gainOrientationP_[i](2) = kp;
        //
        gainOrientationD_[i](0) = kd;
        gainOrientationD_[i](1) = kd;
        gainOrientationD_[i](2) = kd;
    }
    return true;
}

bool RTTCartPIDController::setGainsForEE(double kp, double kd, unsigned int index)
{
    if (vec_dimensions_.size() <= 0)
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "Number of endeffectors (" << vec_dimensions_.size() << ") < " << index << RTT::endlog();
        return false;
    }
    gainTranslationP_[index](0) = kp;
    gainTranslationP_[index](1) = kp;
    gainTranslationP_[index](2) = kp;
    //
    gainTranslationD_[index](0) = kd;
    gainTranslationD_[index](1) = kd;
    gainTranslationD_[index](2) = kd;
    return true;
}

bool RTTCartPIDController::setGainsOrientationForEE(double kp, double kd, unsigned int index)
{
    if (vec_dimensions_.size() <= 0)
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "Number of endeffectors (" << vec_dimensions_.size() << ") < " << index << RTT::endlog();
        return false;
    }
    gainOrientationP_[index](0) = kp;
    gainOrientationP_[index](1) = kp;
    gainOrientationP_[index](2) = kp;
    //
    gainOrientationD_[index](0) = kd;
    gainOrientationD_[index](1) = kd;
    gainOrientationD_[index](2) = kd;
    return true;
}

bool RTTCartPIDController::setGainsBatch(const Eigen::VectorXd &kps, const Eigen::VectorXd &kds)
{
    if (vec_dimensions_.size() <= 0)
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "Number of endeffectors is zero, cannot set translation gains." << RTT::endlog();
        return false;
    }
    if (kps.size() != 3)
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "kps size != 3" << RTT::endlog();
        return false;
    }
    if (kds.size() != 3)
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "kds size != 3" << RTT::endlog();
        return false;
    }
    for (unsigned int i = 0; i < vec_dimensions_.size(); i++)
    {
        gainTranslationP_[i] = kps.head<3>();
        //
        gainTranslationD_[i] = kds.head<3>();
    }
    return true;
}

bool RTTCartPIDController::setGainsOrientationBatch(const Eigen::VectorXd &kps, const Eigen::VectorXd &kds)
{
    if (vec_dimensions_.size() <= 0)
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "Number of endeffectors is zero, cannot set translation gains." << RTT::endlog();
        return false;
    }
    if (kps.size() != 3)
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "kps size != 3" << RTT::endlog();
        return false;
    }
    if (kds.size() != 3)
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "kds size != 3" << RTT::endlog();
        return false;
    }
    for (unsigned int i = 0; i < vec_dimensions_.size(); i++)
    {
        gainOrientationP_[i] = kps.head<3>();
        //
        gainOrientationD_[i] = kds.head<3>();
    }
    return true;
}

bool RTTCartPIDController::setGainsSingleForEE(double kp, double kd, unsigned int index, unsigned int dir)
{
    if (vec_dimensions_.size() <= 0)
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "Number of endeffectors (" << vec_dimensions_.size() << ") < " << index << RTT::endlog();
        return false;
    }
    if (dir > 2)
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "dir > 2" << RTT::endlog();
        return false;
    }
    gainTranslationP_[index](dir) = kp;
    //
    gainTranslationD_[index](dir) = kd;
    return true;
}

bool RTTCartPIDController::setGainsOrientationSingleForEE(double kp, double kd, unsigned int index, unsigned int dir)
{
    if (vec_dimensions_.size() <= index)
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "Number of endeffectors (" << vec_dimensions_.size() << ") < " << index << RTT::endlog();
        return false;
    }
    if (dir > 2)
    {
        RTT::log(RTT::Error) << "[" << this->getName() << "] "
                             << "dir > 2" << RTT::endlog();
        return false;
    }
    gainOrientationP_[index](dir) = kp;
    //
    gainOrientationD_[index](dir) = kd;
    return true;
}

void RTTCartPIDController::computeTranslationError(
    Eigen::Vector3d const &cart_desiredPosition,
    Eigen::Vector3d const &cart_currentPosition,
    Eigen::Vector3d const &cart_desiredVelocity,
    Eigen::Vector3d const &cart_currentVelocity,
    Eigen::Vector3d &cart_errorPosition,
    Eigen::Vector3d &cart_errorVelocity)
{
    cart_errorPosition.setZero();
    cart_errorVelocity.setZero();

    cart_errorPosition = desiredPosition - currentPosition;
    cart_errorVelocity = desiredVelocity - currentVelocity;
}

void RTTCartPIDController::computeOrientationError(
    Eigen::Vector4d const &quaternion_desired,
    Eigen::Vector4d const &quaternion_current,
    Eigen::Vector3d const &axisangle_desiredVelocity,
    Eigen::Vector3d const &axisangle_currentVelocity,
    Eigen::Vector3d &axisangle_errorPosition,
    Eigen::Vector3d &axisangle_errorVelocity)
{
    //orientation proportional feedback part
    axisangle_errorPosition.setZero();

    //version 5 Sina and Josh (SingleArmController, KTSController)
    KDL::Rotation rot_KDL_des = KDL::Rotation::Quaternion(quaternion_desired(1), quaternion_desired(2), quaternion_desired(3), quaternion_desired(0));
    Eigen::Matrix3d rot_mat;
    rot_mat(0, 0) = rot_KDL_des.data[0];
    rot_mat(0, 1) = rot_KDL_des.data[1];
    rot_mat(0, 2) = rot_KDL_des.data[2];
    rot_mat(1, 0) = rot_KDL_des.data[3];
    rot_mat(1, 1) = rot_KDL_des.data[4];
    rot_mat(1, 2) = rot_KDL_des.data[5];
    rot_mat(2, 0) = rot_KDL_des.data[6];
    rot_mat(2, 1) = rot_KDL_des.data[7];
    rot_mat(2, 2) = rot_KDL_des.data[8];

    KDL::Rotation rot_KDL_curr = KDL::Rotation::Quaternion(quaternion_current(1), quaternion_current(2), quaternion_current(3), quaternion_current(0));
    Eigen::Matrix3d rot_mat_curr;
    rot_mat_curr(0, 0) = rot_KDL_curr.data[0];
    rot_mat_curr(0, 1) = rot_KDL_curr.data[1];
    rot_mat_curr(0, 2) = rot_KDL_curr.data[2];
    rot_mat_curr(1, 0) = rot_KDL_curr.data[3];
    rot_mat_curr(1, 1) = rot_KDL_curr.data[4];
    rot_mat_curr(1, 2) = rot_KDL_curr.data[5];
    rot_mat_curr(2, 0) = rot_KDL_curr.data[6];
    rot_mat_curr(2, 1) = rot_KDL_curr.data[7];
    rot_mat_curr(2, 2) = rot_KDL_curr.data[8];
    Eigen::Matrix3d relative_rot = rot_mat.transpose() * rot_mat_curr;
    Eigen::Quaterniond relative_quat(relative_rot);
    // delta_quat = relative_quat.vec().cast<double>();
    // posError.tail<3>() = -rot_mat * delta_quat;
    axisangle_errorPosition = -rot_mat * relative_quat.vec();
    // RTT::log(RTT::Error) << "axisangle_errorPosition v5:\n"
    //                      << axisangle_errorPosition << RTT::endlog();

    //orientation derivative feedback part
    axisangle_errorVelocity.setZero();
    axisangle_errorVelocity = (axisangle_desiredVelocity - axisangle_currentVelocity);
}

void RTTCartPIDController::preparePorts()
{
    this->calculateTotalDofs();
    if (portsArePrepared)
    {
        ports()->removePort("in_desiredTaskSpacePosition_port");
        ports()->removePort("in_desiredTaskSpaceVelocity_port");
        ports()->removePort("in_desiredTaskSpaceAcceleration_port");

        ports()->removePort("in_currentTaskSpacePosition_port");
        ports()->removePort("in_currentTaskSpaceVelocity_port");
        ports()->removePort("in_robotstatus_port");

        ports()->removePort("in_jacobian_port");
        ports()->removePort("in_jacobianDot_port");
        ports()->removePort("in_coriolisAndGravity_port");
        ports()->removePort("in_inertiaInv_port");
        ports()->removePort("in_projection_port");
        ports()->removePort("in_projectionDot_port");

        ports()->removePort("out_torques_port");
        ports()->removePort("out_force_port");

        ports()->removePort("out_error_position_port");
        ports()->removePort("out_error_velocity_port");
    }

    //prepare input
    in_desiredTaskSpace_var = trajectory_msgs::MultiDOFJointTrajectoryPoint();
    in_desiredTaskSpace_var.transforms.push_back(geometry_msgs::Transform());
    in_desiredTaskSpace_var.velocities.push_back(geometry_msgs::Twist());
    in_desiredTaskSpace_var.accelerations.push_back(geometry_msgs::Twist());

    in_desiredTaskSpacePosition_var_eig = Eigen::VectorXd::Zero(TaskSpaceQuaternionDimension);
    in_desiredTaskSpacePosition_proxy = Eigen::VectorXd::Zero(TaskSpaceQuaternionDimension);
    // in_desiredTaskSpacePosition_var = geometry_msgs::Pose();
    // in_desiredTaskSpacePosition_var.position.x = 0;
    // in_desiredTaskSpacePosition_var.position.y = 0;
    // in_desiredTaskSpacePosition_var.position.z = 0;
    // in_desiredTaskSpacePosition_var.orientation.w = 1;
    // in_desiredTaskSpacePosition_var.orientation.x = 0;
    // in_desiredTaskSpacePosition_var.orientation.y = 0;
    // in_desiredTaskSpacePosition_var.orientation.z = 0;
    // in_desiredTaskSpacePosition_port.setName(
    //     "in_desiredTaskSpacePosition_port");
    // in_desiredTaskSpacePosition_port.doc(
    //     "Input port for reading the desired position from Trajectory Generator");
    // ports()->addPort(in_desiredTaskSpacePosition_port);
    in_desiredTaskSpace_port.setName(
        "in_desiredTaskSpace_port");
    in_desiredTaskSpace_port.doc(
        "Input port for reading the desired position, velocity, and acceleration from a Trajectory Generator");
    ports()->addPort(in_desiredTaskSpace_port);
    in_desiredTaskSpacePosition_flow = RTT::NoData;

    in_desiredTaskSpaceVelocity_var_eig = Eigen::VectorXd::Zero(TaskSpaceDimension);
    // in_desiredTaskSpaceVelocity_var.linear.x = 0;
    // in_desiredTaskSpaceVelocity_var.linear.y = 0;
    // in_desiredTaskSpaceVelocity_var.linear.z = 0;
    // in_desiredTaskSpaceVelocity_var.angular.x = 0;
    // in_desiredTaskSpaceVelocity_var.angular.y = 0;
    // in_desiredTaskSpaceVelocity_var.angular.z = 0;
    // in_desiredTaskSpaceVelocity_port.setName(
    //     "in_desiredTaskSpaceVelocity_port");
    // in_desiredTaskSpaceVelocity_port.doc(
    //     "Input port for reading the desired velocity from Trajectory Generator");
    // ports()->addPort(in_desiredTaskSpaceVelocity_port);
    // in_desiredTaskSpaceVelocity_flow = RTT::NoData;

    in_desiredTaskSpaceAcceleration_var_eig = Eigen::VectorXd::Zero(TaskSpaceDimension);
    // in_desiredTaskSpaceAcceleration_var.linear.x = 0;
    // in_desiredTaskSpaceAcceleration_var.linear.y = 0;
    // in_desiredTaskSpaceAcceleration_var.linear.z = 0;
    // in_desiredTaskSpaceAcceleration_var.angular.x = 0;
    // in_desiredTaskSpaceAcceleration_var.angular.y = 0;
    // in_desiredTaskSpaceAcceleration_var.angular.z = 0;
    // in_desiredTaskSpaceAcceleration_port.setName(
    //     "in_desiredTaskSpaceAcceleration_port");
    // in_desiredTaskSpaceAcceleration_port.doc(
    //     "Input port for reading the desired acceleration from Trajectory Generator");
    // ports()->addPort(in_desiredTaskSpaceAcceleration_port);
    // in_desiredTaskSpaceAcceleration_flow = RTT::NoData;

    in_currentTaskSpacePosition_var = Eigen::VectorXd::Zero(TaskSpaceQuaternionDimension);
    in_currentTaskSpacePosition_port.setName(
        "in_currentTaskSpacePosition_port");
    in_currentTaskSpacePosition_port.doc(
        "Input port for reading the current position");
    ports()->addPort(in_currentTaskSpacePosition_port);
    in_currentTaskSpacePosition_flow = RTT::NoData;

    in_currentTaskSpaceVelocity_var = Eigen::VectorXd::Zero(TaskSpaceDimension);
    in_currentTaskSpaceVelocity_port.setName(
        "in_currentTaskSpaceVelocity_port");
    in_currentTaskSpaceVelocity_port.doc(
        "Input port for reading the current position velocity");
    ports()->addPort(in_currentTaskSpaceVelocity_port);
    in_currentTaskSpaceVelocity_flow = RTT::NoData;

    in_robotstatus_var = sensor_msgs::JointState();
    for (unsigned int i = 0; i < total_dof_size_; i++)
    {
        in_robotstatus_var.position.push_back(0.0);
        in_robotstatus_var.velocity.push_back(0.0);
        in_robotstatus_var.effort.push_back(0.0);
    }
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for reading the joint velocity");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    in_jacobian_var = Eigen::MatrixXd::Zero(TaskSpaceDimension, total_dof_size_);
    in_jacobian_port.setName("in_jacobian_port");
    in_jacobian_port.doc("Input port for reading the jacobian");
    ports()->addPort(in_jacobian_port);
    in_jacobian_flow = RTT::NoData;

    inertiaInvP = Eigen::MatrixXd::Zero(total_dof_size_, total_dof_size_);

    in_jacobianDot_var = Eigen::MatrixXd::Zero(TaskSpaceDimension, total_dof_size_);
    in_jacobianDot_port.setName("in_jacobianDot_port");
    in_jacobianDot_port.doc(
        "Input port for reading the derivative of jacobian");
    ports()->addPort(in_jacobianDot_port);
    in_jacobianDot_flow = RTT::NoData;

    in_coriolisAndGravity_var = Eigen::VectorXd::Zero(total_dof_size_);
    in_coriolisAndGravity_port.setName("in_coriolisAndGravity_port");
    in_coriolisAndGravity_port.doc(
        "Input port for reading h vector");
    ports()->addPort(in_coriolisAndGravity_port);
    in_coriolisAndGravity_flow = RTT::NoData;

    in_inertia_var = Eigen::MatrixXd::Zero(total_dof_size_, total_dof_size_);
    in_inertia_port.setName("in_inertia_port");
    in_inertia_port.doc(
        "Input port for reading the (constraint) inertia matrix");
    ports()->addPort(in_inertia_port);
    in_inertia_flow = RTT::NoData;

    in_projection_var = Eigen::MatrixXd::Identity(total_dof_size_, total_dof_size_);
    in_projection_port.setName("in_projection_port");
    in_projection_port.doc(
        "Input port for reading the projection matrix");
    ports()->addPort(in_projection_port);
    in_projection_flow = RTT::NoData;

    in_projectionDot_var = Eigen::MatrixXd::Zero(total_dof_size_, total_dof_size_);
    in_projectionDot_port.setName("in_projectionDot_port");
    in_projectionDot_port.doc("Input port for reading the projectionDot matrix");
    ports()->addPort(in_projectionDot_port);
    in_projectionDot_flow = RTT::NoData;

    //prepare output
    out_torques_var = Eigen::VectorXd(total_dof_size_);
    out_torques_var.setZero();
    out_torques_port.setName("out_torques_port");
    out_torques_port.doc("Output port for sending torque values");
    out_torques_port.setDataSample(out_torques_var);
    ports()->addPort(out_torques_port);

    out_force_var = Eigen::VectorXd::Zero(TaskSpaceDimension);
    out_force_port.setName("out_force_port");
    out_force_port.doc("Output port for sending force vector");
    out_force_port.setDataSample(out_force_var);
    ports()->addPort(out_force_port);

    out_error_position_port.setName("out_error_position_port");
    out_error_position_port.doc("Output port for sending position error vector");
    out_error_position_port.setDataSample(errorPosition);
    ports()->addPort(out_error_position_port);

    out_error_velocity_port.setName("out_error_velocity_port");
    out_error_velocity_port.doc("Output port for sending velocity error vector");
    out_error_velocity_port.setDataSample(errorVelocity);
    ports()->addPort(out_error_velocity_port);

    portsArePrepared = true;
}

void RTTCartPIDController::displayStatus()
{

    std::string position;
    for (const auto &piece : in_robotstatus_var.position)
        position += piece;

    std::string velocity;
    for (const auto &piece : in_robotstatus_var.velocity)
        velocity += piece;

    std::cout << "############## RTTCartPIDController State begin " << std::endl;
    RTT::log(RTT::Error) << "in_robotstatus_var.angles \n"
                         << position << RTT::endlog();
    RTT::log(RTT::Error) << "in_robotstatus_var.velocities \n"
                         << velocity << RTT::endlog();
    // RTT::log(RTT::Error) << "in_desiredTaskSpacePosition_var \n"
    //                      << in_desiredTaskSpacePosition_var << RTT::endlog();
    // RTT::log(RTT::Error) << "in_desiredTaskSpaceVelocity_var \n"
    //                      << in_desiredTaskSpaceVelocity_var << RTT::endlog();
    // RTT::log(RTT::Error) << "in_desiredTaskSpaceAcceleration_var \n"
    //                      << in_desiredTaskSpaceAcceleration_var << RTT::endlog();
    RTT::log(RTT::Error) << "in_desiredTaskSpace_var \n"
                         << in_desiredTaskSpace_var << RTT::endlog();

    RTT::log(RTT::Error) << "in_currentTaskSpacePosition_var \n"
                         << in_currentTaskSpacePosition_var << RTT::endlog();
    RTT::log(RTT::Error) << "in_currentTaskSpaceVelocity_var \n"
                         << in_currentTaskSpaceVelocity_var << RTT::endlog();

    RTT::log(RTT::Error) << "in_jacobian_var \n"
                         << in_jacobian_var
                         << RTT::endlog();
    RTT::log(RTT::Error) << "in_jacobianDot_var \n"
                         << in_jacobianDot_var
                         << RTT::endlog();
    RTT::log(RTT::Error) << "in_coriolisAndGravity_var \n"
                         << in_coriolisAndGravity_var << RTT::endlog();
    RTT::log(RTT::Error) << "in_inertia_var \n"
                         << in_inertia_var
                         << RTT::endlog();
    RTT::log(RTT::Error) << "in_projection_var \n"
                         << in_projection_var
                         << RTT::endlog();
    RTT::log(RTT::Error) << "in_projectionDot_var \n"
                         << in_projectionDot_var
                         << RTT::endlog();
    RTT::log(RTT::Error) << "out_torques_var \n"
                         << out_torques_var
                         << RTT::endlog();
    RTT::log(RTT::Error) << "out_force_var \n"
                         << out_force_var
                         << RTT::endlog();

    RTT::log(RTT::Error) << "errorPosition \n"
                         << errorPosition << RTT::endlog();
    RTT::log(RTT::Error) << "errorVelocity \n"
                         << errorVelocity << RTT::endlog();

    // RTT::log(RTT::Error) << "gainTranslationP = "
    //                      << gainTranslationP << RTT::endlog();
    // RTT::log(RTT::Error) << "gainTranslationD = "
    //                      << gainTranslationD << RTT::endlog();

    // RTT::log(RTT::Error) << "gainOrientationP = "
    //                      << gainOrientationP << RTT::endlog();
    // RTT::log(RTT::Error) << "gainOrientationD = "
    //                      << gainOrientationD << RTT::endlog();

    RTT::log(RTT::Error) << "in_currentTaskSpacePosition_var \n"
                         << in_currentTaskSpacePosition_var << RTT::endlog();

    RTT::log(RTT::Error) << "use_TSgravitycompensation "
                         << use_TSgravitycompensation << RTT::endlog();
    RTT::log(RTT::Error) << "total_dof_size_ "
                         << total_dof_size_ << RTT::endlog();

    if (impedanceCTRL)
    {
        RTT::log(RTT::Error) << "impedance control law " << RTT::endlog();
    }
    else
    {
        RTT::log(RTT::Error) << "inverse dynamics control law " << RTT::endlog();
    }

    std::cout << "############## RTTCartPIDController State end " << std::endl;
}

void RTTCartPIDController::checkConnections()
{
    if (!in_robotstatus_port.connected())
    {
        RTT::log(RTT::Info) << "in_robotstatus_port not connected"
                            << RTT::endlog();
    }

    if (!in_desiredTaskSpace_port.connected())
    {
        RTT::log(RTT::Info) << "in_desiredTaskSpace_port not connected"
                            << RTT::endlog();
    }

    // if (!in_desiredTaskSpacePosition_port.connected())
    // {
    //     RTT::log(RTT::Info) << "in_desiredTaskSpacePosition_port not connected"
    //                         << RTT::endlog();
    // }

    // if (!in_desiredTaskSpaceVelocity_port.connected())
    // {
    //     RTT::log(RTT::Info) << "in_desiredTaskSpaceVelocity_port not connected"
    //                         << RTT::endlog();
    // }

    // if (!in_desiredTaskSpaceAcceleration_port.connected())
    // {
    //     RTT::log(RTT::Info) << "in_desiredTaskSpaceAcceleration_port not connected"
    //                         << RTT::endlog();
    // }

    if (!in_currentTaskSpacePosition_port.connected())
    {
        RTT::log(RTT::Info) << "in_currentTaskSpacePosition_port not connected"
                            << RTT::endlog();
    }

    if (!in_currentTaskSpaceVelocity_port.connected())
    {
        RTT::log(RTT::Info) << "in_currentTaskSpaceVelocity_port not connected"
                            << RTT::endlog();
    }

    if (!in_jacobian_port.connected())
    {
        RTT::log(RTT::Info) << "in_jacobian_port not connected"
                            << RTT::endlog();
    }

    if (!in_jacobianDot_port.connected())
    {
        RTT::log(RTT::Info) << "in_jacobianDot_port not connected"
                            << RTT::endlog();
    }

    if (!in_coriolisAndGravity_port.connected())
    {
        RTT::log(RTT::Info) << "in_coriolisAndGravity_port not connected"
                            << RTT::endlog();
    }

    if (!in_inertia_port.connected())
    {
        RTT::log(RTT::Info) << "in_inertia_port not connected"
                            << RTT::endlog();
    }

    if (!in_projection_port.connected())
    {
        RTT::log(RTT::Info) << "in_projection_port not connected"
                            << RTT::endlog();
    }

    if (!in_projectionDot_port.connected())
    {
        RTT::log(RTT::Info) << "in_projectionDot_port not connected"
                            << RTT::endlog();
    }

    if (!out_torques_port.connected())
    {
        RTT::log(RTT::Info) << "out_torques_port not connected"
                            << RTT::endlog();
    }
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(cosima::controller::RTTCartPIDController)
