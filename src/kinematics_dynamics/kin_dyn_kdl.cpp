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

#include "../../include/cosima-controller/kinematics_dynamics/kin_dyn_kdl.hpp"

#include <rtt/Logger.hpp>
#define PRELOG(X) (RTT::log(RTT::X) << "[KinDynMultiArm_KDL:" << this->model_name << "] ")

using namespace cosima;

KinDynMultiArm_KDL::KinDynMultiArm_KDL(const std::string &modelname) : KinDynInterface(modelname)
{
  this->loadModel(modelname);
  quat_helper.setZero();

  this->gravity_vectorKDL = KDL::Vector();
  this->gravity_vectorKDL.data[0] = 0.0;
  this->gravity_vectorKDL.data[1] = 0.0;
  this->gravity_vectorKDL.data[2] = -9.81;
}

void KinDynMultiArm_KDL::initializeVariables()
{
  g = KDL::JntArray(this->kdl_chain.getNrOfJoints());
  M = KDL::JntSpaceInertiaMatrix(this->kdl_chain.getNrOfJoints());
  c = KDL::JntArray(this->kdl_chain.getNrOfJoints());

  this->q_qd = KDL::JntArrayVel(this->kdl_chain.getNrOfJoints());
  this->g = KDL::JntArray(this->kdl_chain.getNrOfJoints());
  this->M = KDL::JntSpaceInertiaMatrix(this->kdl_chain.getNrOfJoints());
  this->c = KDL::JntArray(this->kdl_chain.getNrOfJoints());
  this->J = KDL::Jacobian(this->kdl_chain.getNrOfJoints());
  this->Jd = KDL::Jacobian(this->kdl_chain.getNrOfJoints());
  this->cartPosFrame = KDL::Frame();
  this->cartVelFrame = KDL::FrameVel();

  this->jnt_array_zero = KDL::JntArray(this->kdl_chain.getNrOfJoints());
  this->wrench_zero = KDL::Wrenches(this->kdl_chain.getNrOfSegments(), KDL::Wrench::Zero());
}

void KinDynMultiArm_KDL::computeStackPart(const sensor_msgs::JointState &in_robotstatus,
                                          const unsigned int &index_js,
                                          const unsigned int &index_ts,
                                          const unsigned int &index_ts_quat,
                                          Eigen::MatrixXd &out_inertia,
                                          Eigen::MatrixXd &out_inertiaInv,
                                          Eigen::VectorXd &out_gravity,
                                          Eigen::VectorXd &out_coriolis,
                                          Eigen::VectorXd &out_coriolisAndGravity,
                                          Eigen::VectorXd &out_cartPos,
                                          Eigen::VectorXd &out_cartVel,
                                          Eigen::VectorXd &out_cartAcc,
                                          Eigen::MatrixXd &out_jacobian,
                                          Eigen::MatrixXd &out_jacobianDot,
                                          const Eigen::VectorXd &ext_coriolisAndGravity,
                                          const Eigen::MatrixXd &ext_inertia,
                                          bool override)
{
  this->solve(in_robotstatus);

  //////////////////////////////////
  // CONVERT TO OUTPUT DATA TYPES //
  //////////////////////////////////

  unsigned int _js_dof = this->getDoF();

  if (override)
  {
    RTT::log(RTT::Error) << "ext_inertia.block =\n" << ext_inertia.block(index_js, index_js, _js_dof, _js_dof) << RTT::endlog();
    // Use externally provided data
    out_inertia.block(index_js, index_js, _js_dof, _js_dof) = ext_inertia.block(index_js, index_js, _js_dof, _js_dof);
    // TODO perhaps do something else, because I dunno if the inertia is always invertable
    out_inertiaInv.block(index_js, index_js, _js_dof, _js_dof) = ext_inertia.block(index_js, index_js, _js_dof, _js_dof).inverse();
    out_gravity.segment(index_js, _js_dof) = ext_coriolisAndGravity.segment(index_js, _js_dof);
    // out_coriolis.segment(index_js, _js_dof) = this->c.data;
    out_coriolis.segment(index_js, _js_dof).setZero();
    out_coriolisAndGravity.segment(index_js, _js_dof) = ext_coriolisAndGravity.segment(index_js, _js_dof);
  }
  else
  {  
    out_inertia.block(index_js, index_js, _js_dof, _js_dof) = this->M.data;
    out_inertiaInv.block(index_js, index_js, _js_dof, _js_dof) = this->M.data.inverse();

    out_gravity.segment(index_js, _js_dof) = this->g.data;
    out_coriolis.segment(index_js, _js_dof) = this->c.data;
    out_coriolisAndGravity.segment(index_js, _js_dof) = out_gravity + out_coriolis;
  }

  out_cartPos(index_ts_quat + 0) = this->cartPosFrame.p.x();
  out_cartPos(index_ts_quat + 1) = this->cartPosFrame.p.y();
  out_cartPos(index_ts_quat + 2) = this->cartPosFrame.p.z();
  double _x, _y, _z, _w;
  this->cartPosFrame.M.GetQuaternion(_x, _y, _z, _w);
  this->quat_helper(0) = _w;
  this->quat_helper(1) = _x;
  this->quat_helper(2) = _y;
  this->quat_helper(3) = _z;
  // TODO stuff from Josh? Dunno why we might need this. I guess it takes care of the flipping of orientation
  if (this->quat_helper(0) < 0)
  {
    this->quat_helper = -this->quat_helper;
  }
  else if (this->quat_helper(0) == 0)
  {
    if (this->quat_helper(1) < 0)
    {
      this->quat_helper.segment<3>(1) = -this->quat_helper.segment<3>(1);
    }
    else if (this->quat_helper(1) == 0)
    {
      if (this->quat_helper(2) < 0)
      {
        this->quat_helper.segment<2>(2) = -this->quat_helper.segment<2>(2);
      }
      else if (this->quat_helper(2) == 0)
      {
        if (this->quat_helper(3) < 0)
        {
          this->quat_helper(3) = -this->quat_helper(3);
        }
      }
    }
  }
  out_cartPos(index_ts_quat + 3) = this->quat_helper(0);
  out_cartPos(index_ts_quat + 4) = this->quat_helper(1);
  out_cartPos(index_ts_quat + 5) = this->quat_helper(2);
  out_cartPos(index_ts_quat + 6) = this->quat_helper(3);

  out_cartVel(index_ts + 0) = this->cartVelFrame.GetTwist().vel.x(); //similar to velFrame.p.v.x();
  out_cartVel(index_ts + 1) = this->cartVelFrame.GetTwist().vel.y(); //similar to velFrame.p.v.y();
  out_cartVel(index_ts + 2) = this->cartVelFrame.GetTwist().vel.z(); //similar to velFrame.p.v.z();
  out_cartVel(index_ts + 3) = this->cartVelFrame.GetTwist().rot.x();
  out_cartVel(index_ts + 4) = this->cartVelFrame.GetTwist().rot.y();
  out_cartVel(index_ts + 5) = this->cartVelFrame.GetTwist().rot.z();

  out_jacobian.block(index_ts, index_js, 6, _js_dof) = this->J.data;
  out_jacobianDot.block(index_ts, index_js, 6, _js_dof) = this->Jd.data;

  out_cartAcc.segment(index_ts, 6) = this->Jd.data * this->q_qd.qdot.data; // TODO: add out_jacobian_var * in_robotstatus_var.accelerations
}

void KinDynMultiArm_KDL::solve(const sensor_msgs::JointState &in_robotstatus)
{
  // //////////////////////////
  // // Initialize Variables //
  // //////////////////////////
  // out_inertia.setZero();
  // out_gravity.setZero();
  // out_coriolis.setZero();
  // out_cartPos.position.x = 0;
  // out_cartPos.position.y = 0;
  // out_cartPos.position.z = 0;
  // out_cartPos.orientation.w = 0;
  // out_cartPos.orientation.x = 0;
  // out_cartPos.orientation.y = 0;
  // out_cartPos.orientation.z = 0;
  // out_cartVel.setZero();
  // out_jacobian.setZero();
  // out_jacobianDot.setZero();
  // // initialize KDL fields
  // G_.data.setZero();
  // M_.data.setZero();
  // C_.data.setZero();

  ///////////////////////////////////////////
  // RETRIEVE INFORMATION FROM THE SOLVERS //
  ///////////////////////////////////////////
  this->q_qd.q.data.setZero();
  this->q_qd.qdot.data.setZero();
  this->g.data.setZero();
  this->M.data.setZero();
  this->c.data.setZero();
  this->J.data.setZero();
  this->Jd.data.setZero();

  // Convert in_robotstatus to KDL
  for (unsigned int i = 0; i < this->kdl_chain.getNrOfJoints(); i++) // TODO optimize to make it faster
  {
    this->q_qd.q.data(i) = in_robotstatus.position[i];
    this->q_qd.qdot.data(i) = in_robotstatus.velocity[i];
  }

  ////////////////////////////////
  // CALCULATE INVERSE DYNAMICS //
  ////////////////////////////////
  this->id_dyn_solver->JntToGravity(this->q_qd.q, this->g);
  this->id_dyn_solver->JntToCoriolis(this->q_qd.q, this->q_qd.qdot, this->c);
  this->id_dyn_solver->JntToMass(this->q_qd.q, this->M);

  // // With or without velocity (qdot)?
  // int code = this->jnt_gravity_solver->CartToJnt(this->q_qd.q,
  //                                                this->q_qd.qdot,
  //                                                this->jnt_array_zero,
  //                                                this->wrench_zero,
  //                                                this->g);

  //////////////////////////////////////////////
  // CLOSE FORM FOR LWR 4(+) TO OVERRRIDE KDL //
  //////////////////////////////////////////////

  // kukaClosedForm.calc_gravity_torque_vector(tempGravity, this->q_qd.q.data.data());
  // g.data = Eigen::Map<Eigen::VectorXd>(tempGravity, 7);
  // kukaClosedForm.calc_coriolis_matrix(tempCorilois, this->q_qd.q.data.data(), this->q_qd.qdot.data.data());
  // c.data = Eigen::Map<Eigen::MatrixXd>(*tempCorilois, 7, 7) * this->q_qd.qdot.data;

  // kukaClosedForm.calc_inertia_matrix(tempInertia, this->q_qd.q.data.data());
  // M.data = Eigen::Map<Eigen::MatrixXd>(*tempInertia, 7, 7);

  //////////////////////////////////
  // CALCULATE FORWARD KINEMATICS //
  //////////////////////////////////
  unsigned int _number_of_segments = this->kdl_chain.getNrOfSegments();

  this->jnt_to_jac_solver->JntToJac(this->q_qd.q, this->J, _number_of_segments);

  // PRELOG(Error) << "this->q_qd.q = " << this->q_qd.q.data << RTT::endlog();
  // PRELOG(Error) << "this->q_qd.qdot = " << this->q_qd.qdot.data << RTT::endlog();
  // PRELOG(Error) << "_number_of_segments = " << _number_of_segments << RTT::endlog();
  // PRELOG(Error) << "this->Jd 1 = (" << this->Jd.rows() << ", " << this->Jd.columns() << ")" << RTT::endlog();
  this->jnt_to_jac_dot_solver->JntToJacDot(this->q_qd, this->Jd, _number_of_segments);
  // PRELOG(Error) << "this->Jd 2 = (" << this->Jd.rows() << ", " << this->Jd.columns() << ")" << RTT::endlog();

  this->jnt_to_cart_pos_solver->JntToCart(this->q_qd.q, this->cartPosFrame, _number_of_segments);
  this->jnt_to_cart_vel_solver->JntToCart(this->q_qd, this->cartVelFrame, _number_of_segments);
}

void KinDynMultiArm_KDL::compute(const sensor_msgs::JointState &in_robotstatus,
                                 Eigen::MatrixXd &out_inertia,
                                 Eigen::MatrixXd &out_inertiaInv,
                                 Eigen::VectorXd &out_gravity,
                                 Eigen::VectorXd &out_coriolis,
                                 Eigen::VectorXd &out_coriolisAndGravity,
                                 Eigen::VectorXd &out_cartPos,
                                 Eigen::VectorXd &out_cartVel,
                                 Eigen::VectorXd &out_cartAcc,
                                 Eigen::MatrixXd &out_jacobian,
                                 Eigen::MatrixXd &out_jacobianDot)
{
  this->solve(in_robotstatus);

  //////////////////////////////////
  // CONVERT TO OUTPUT DATA TYPES //
  //////////////////////////////////

  out_inertia = this->M.data;
  out_inertiaInv = out_inertia.inverse();

  out_gravity = this->g.data;
  out_coriolis = this->c.data;
  out_coriolisAndGravity = out_gravity + out_coriolis;

  // out_cartPos.position.x = cartPosFrame.p.x();
  // out_cartPos.position.y = cartPosFrame.p.y();
  // out_cartPos.position.z = cartPosFrame.p.z();
  out_cartPos(0) = this->cartPosFrame.p.x();
  out_cartPos(1) = this->cartPosFrame.p.y();
  out_cartPos(2) = this->cartPosFrame.p.z();
  double _x, _y, _z, _w;
  this->cartPosFrame.M.GetQuaternion(_x, _y, _z, _w);
  this->quat_helper(0) = _w;
  this->quat_helper(1) = _x;
  this->quat_helper(2) = _y;
  this->quat_helper(3) = _z;
  // TODO stuff from Josh? Dunno why we might need this. I guess it takes care of the flipping of orientation
  if (this->quat_helper(0) < 0)
  {
    this->quat_helper = -this->quat_helper;
  }
  else if (this->quat_helper(0) == 0)
  {
    if (this->quat_helper(1) < 0)
    {
      this->quat_helper.segment<3>(1) = -this->quat_helper.segment<3>(1);
    }
    else if (this->quat_helper(1) == 0)
    {
      if (this->quat_helper(2) < 0)
      {
        this->quat_helper.segment<2>(2) = -this->quat_helper.segment<2>(2);
      }
      else if (this->quat_helper(2) == 0)
      {
        if (this->quat_helper(3) < 0)
        {
          this->quat_helper(3) = -this->quat_helper(3);
        }
      }
    }
  }
  // out_cartPos.orientation.w = this->quat_helper(0);
  // out_cartPos.orientation.x = this->quat_helper(1);
  // out_cartPos.orientation.y = this->quat_helper(2);
  // out_cartPos.orientation.z = this->quat_helper(3);
  out_cartPos(3) = this->quat_helper(0);
  out_cartPos(4) = this->quat_helper(1);
  out_cartPos(5) = this->quat_helper(2);
  out_cartPos(6) = this->quat_helper(3);

  out_cartVel(0) = this->cartVelFrame.GetTwist().vel.x(); //similar to velFrame.p.v.x();
  out_cartVel(1) = this->cartVelFrame.GetTwist().vel.y(); //similar to velFrame.p.v.y();
  out_cartVel(2) = this->cartVelFrame.GetTwist().vel.z(); //similar to velFrame.p.v.z();
  out_cartVel(3) = this->cartVelFrame.GetTwist().rot.x();
  out_cartVel(4) = this->cartVelFrame.GetTwist().rot.y();
  out_cartVel(5) = this->cartVelFrame.GetTwist().rot.z();

  out_jacobian = this->J.data;
  out_jacobianDot = this->Jd.data;

  out_cartAcc = out_jacobianDot * this->q_qd.qdot.data; // TODO: add out_jacobian_var * in_robotstatus_var.accelerations
}

bool KinDynMultiArm_KDL::exists_test(const std::string &name)
{
  if (FILE *file = fopen(name.c_str(), "r"))
  {
    fclose(file);
    return true;
  }
  else
  {
    return false;
  }
}

unsigned int KinDynMultiArm_KDL::getDoF()
{
  return kdl_chain.getNrOfJoints();
}

bool KinDynMultiArm_KDL::loadModel(const std::string &modelname)
{
  if (modelname.length() <= 0)
  {
    PRELOG(Error) << "Model name is empty. Returning." << RTT::endlog();
    return false;
  }
  if (exists_test(modelname) != true)
  {
    PRELOG(Error) << "File for model name does not exist. Returning." << RTT::endlog();
    return false;
  }

  if (!kdl_parser::treeFromFile(modelname, this->kdl_tree))
  {
    PRELOG(Error) << "Could not extract kdl tree. Returning." << RTT::endlog();
    return false;
  }

  if (this->kdl_tree.getNrOfJoints() <= 0)
  {
    PRELOG(Error) << "this->kdl_tree.getNrOfJoints() = " << this->kdl_tree.getNrOfJoints() << " need to be > 0. Returning." << RTT::endlog();
    return false;
  }

  if (this->kdl_tree.getNrOfSegments() <= 0)
  {
    PRELOG(Error) << "this->kdl_tree.getNrOfSegments() = " << this->kdl_tree.getNrOfSegments() << " need to be > 0. Returning." << RTT::endlog();
    return false;
  }

  PRELOG(Debug) << "kdl_tree NrOfJoints = " << this->kdl_tree.getNrOfJoints() << RTT::endlog();
  PRELOG(Debug) << "kdl_tree getNrOfSegments = " << this->kdl_tree.getNrOfSegments() << RTT::endlog();
  return true;
}

bool KinDynMultiArm_KDL::setChain(const std::string &chain_root_link_name, const std::string &chain_tip_link_name)
{
  geometry_msgs::Pose _worldOffset;
  _worldOffset.position.x = 0;
  _worldOffset.position.y = 0;
  _worldOffset.position.z = 0;

  _worldOffset.orientation.w = 1;
  _worldOffset.orientation.x = 0;
  _worldOffset.orientation.y = 0;
  _worldOffset.orientation.z = 0;

  return this->setChainWithWorldOffset(chain_root_link_name, chain_tip_link_name, _worldOffset);
}

bool KinDynMultiArm_KDL::setChainWithWorldOffset(const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const Eigen::VectorXd &worldOffsetTranslation, const Eigen::VectorXd &worldOffsetRotation)
{
  if (worldOffsetTranslation.size() != 3)
  {
    RTT::log(RTT::Error) << "Please supply a 3d position" << RTT::endlog();
    return false;
  }
  if (worldOffsetRotation.size() != 4)
  {
    RTT::log(RTT::Error) << "Please supply a quaternion (wxyz)" << RTT::endlog();
    return false;
  }
  geometry_msgs::Pose _worldOffset;
  _worldOffset.position.x = worldOffsetTranslation(0);
  _worldOffset.position.y = worldOffsetTranslation(1);
  _worldOffset.position.z = worldOffsetTranslation(2);

  _worldOffset.orientation.w = worldOffsetRotation(0);
  _worldOffset.orientation.x = worldOffsetRotation(1);
  _worldOffset.orientation.y = worldOffsetRotation(2);
  _worldOffset.orientation.z = worldOffsetRotation(3);

  return this->setChainWithWorldOffset(chain_root_link_name, chain_tip_link_name, _worldOffset);
}

bool KinDynMultiArm_KDL::setChainWithWorldOffset(const std::string &chain_root_link_name, const std::string &chain_tip_link_name, const geometry_msgs::Pose &worldOffset)
{
  if (chain_root_link_name.length() <= 0)
  {
    RTT::log(RTT::Error) << "chain_root_link_name seems to be empty. Returning." << RTT::endlog();
    return false;
  }
  if (chain_tip_link_name.length() <= 0)
  {
    RTT::log(RTT::Error) << "chain_tip_link_name seems to be empty. Returning." << RTT::endlog();
    return false;
  }

  // Make sure that the model was properly loaded in the first place!
  if ((this->kdl_tree.getNrOfJoints() <= 0) || (this->kdl_tree.getNrOfSegments() <= 0))
  {
    RTT::log(RTT::Error) << "this->kdl_tree.getNrOfJoints() = " << this->kdl_tree.getNrOfJoints() << ", this->kdl_tree.getNrOfSegments() = " << this->kdl_tree.getNrOfSegments() << RTT::endlog();
    return false;
  }

  //////////////////////////////////////////////
  ////// CHOOSE THE KINEMATIC CHAIN CHAIN //////
  //////////////////////////////////////////////
  KDL::Chain _chain_selected_tmp = KDL::Chain();
  this->kdl_tree.getChain(chain_root_link_name, chain_tip_link_name, _chain_selected_tmp);

  // Make sure that the selected kinematic chain, actually is a valid one!
  if (_chain_selected_tmp.getNrOfJoints() <= 0)
  {
    RTT::log(RTT::Error) << "Number of joints is not > 0. Please check the link names. Returning." << RTT::endlog();
    return false;
  }

  if (_chain_selected_tmp.getNrOfSegments() <= 0)
  {
    RTT::log(RTT::Error) << "Number of segments is not > 0. Please check the link names. Returning." << RTT::endlog();
    return false;
  }

  //////////////////////////////////////////////////
  ////// OFFSET THE TMP KINEMATIC CHAIN CHAIN //////
  //////////////////////////////////////////////////
  KDL::Vector _trans;
  KDL::Rotation _rot;
  KDL::Frame _worldOffset_frame;
  KDL::Segment _worldOffset_segment;
  KDL::Chain _chain_offset_tmp;

  _trans = KDL::Vector(worldOffset.position.x, worldOffset.position.y, worldOffset.position.z);
  _rot = KDL::Rotation::Quaternion(worldOffset.orientation.x, worldOffset.orientation.y, worldOffset.orientation.z, worldOffset.orientation.w); //convert representation convention from w,x,y,z ==> x,y,z,w
  _worldOffset_frame = KDL::Frame(_rot, _trans);
  _worldOffset_segment = KDL::Segment("worldOffset", KDL::Joint(KDL::Joint::None), _worldOffset_frame);
  _chain_offset_tmp = KDL::Chain();
  _chain_offset_tmp.addSegment(_worldOffset_segment);
  _chain_offset_tmp.addChain(_chain_selected_tmp);

  // Make sure that the offset produced a valid chain!
  if (_chain_offset_tmp.getNrOfJoints() <= 0)
  {
    RTT::log(RTT::Error) << "Number of joints of offset chain is not > 0. Please check the link names. Returning." << RTT::endlog();
    return false;
  }

  if (_chain_offset_tmp.getNrOfSegments() <= 0)
  {
    RTT::log(RTT::Error) << "Number of segments of offset chain is not > 0. Please check the link names. Returning." << RTT::endlog();
    return false;
  }

  /////////////////////////////////////////////
  ////// ADD PAYLOAD OF THE LAST SEGMENT //////
  /////////////////////////////////////////////

  /**
   * https://github.com/ubi-agni/lwr_robot/blob/f98f09b760969f88039d257e0b239918fcaaa8cb/lwr_simulation/src/lwr_controller.cpp#L280
   * Many thanks to Dr. Guillaume Walck (gwalck[AT]techfak.uni-bielefeld.de) for sharing this code snipped!
   */
  // KDL::Segment *segment_ee_ptr = &(_chain_offset_tmp.segments[_chain_offset_tmp.getNrOfSegments() - 1]);
  // KDL::RigidBodyInertia inertia_ee = segment_ee_ptr->getInertia();
  // // retrieve the initial mass, cog and inertia of the last segment
  // double _ee_mass = inertia_ee.getMass();
  // KDL::Vector _ee_cog = inertia_ee.getCOG();
  // KDL::RotationalInertia rot_inertia_ee = inertia_ee.getRotationalInertia();
  // // add payload cog offset and mass
  // // retrieve tool mass and cog offset.
  // // the offset is the relative position of the tool com from the last segment cog
  // // given in the frame orientation of the last segment.
  // // so this offset is valid only for an certain assembly (calib of the tool)

  // segment_ee_ptr->setInertia(KDL::RigidBodyInertia(_ee_mass, _ee_cog, rot_inertia_ee));

  // get last segment
  KDL::Segment *segment_ee_ptr = &(_chain_offset_tmp.segments[_chain_offset_tmp.getNrOfSegments() - 1]);
  // get its dynamic parameters
  KDL::RigidBodyInertia inertia_ee = segment_ee_ptr->getInertia();
  double eeMass_ = inertia_ee.getMass();
  KDL::Vector eeCOG_ = inertia_ee.getCOG();
  KDL::RotationalInertia rot_inertia_ee = inertia_ee.getRotationalInertia();
  double cog_x = 0.0;
  double cog_y = 0.0;
  double cog_z = 0.108000;
  KDL::Vector payload_cog_offset(cog_x, cog_y, cog_z);
  double m = 0.01 + 0.01;
  double m_combined = eeMass_ + m;
  KDL::Vector cog_offset_combined = (payload_cog_offset - eeCOG_) * m / m_combined;
  cog_offset_combined += eeCOG_;
  // set the new inertia at the end-effector.
  // segment_ee_ptr->setInertia(KDL::RigidBodyInertia(m_combined, cog_offset_combined, rot_inertia_ee));

  this->kdl_chain = _chain_offset_tmp;

  ////////////////////////////////////////
  ////// INITIALIZE THE KDL SOLVERS //////
  ////////////////////////////////////////
  this->id_dyn_solver.reset(new KDL::ChainDynParam(this->kdl_chain, this->gravity_vectorKDL));
  this->jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(this->kdl_chain));
  this->jnt_to_jac_dot_solver.reset(new KDL::ChainJntToJacDotSolver(this->kdl_chain));
  this->jnt_to_cart_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(this->kdl_chain));
  this->jnt_to_cart_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(this->kdl_chain));
  this->jnt_gravity_solver.reset(new KDL::ChainIdSolver_RNE(this->kdl_chain, this->gravity_vectorKDL));

  this->initializeVariables();

  return true;
}