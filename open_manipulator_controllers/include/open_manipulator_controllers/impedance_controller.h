#ifndef OPEN_MANIPULATOR_CONTROLLERS_IMPEDANCE_CONTROLLER_H
#define OPEN_MANIPULATOR_CONTROLLERS_IMPEDANCE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <urdf/model.h>

#include <boost/scoped_ptr.hpp>
#include <eigen3/Eigen/Eigen>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <string>
#include <vector>
#include <fstream>

namespace open_manipulator_controllers {

class ImpedanceController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  // Joint handle
  unsigned int n_joints_;
  hardware_interface::EffortJointInterface* effort_joint_interface_;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_;
  std::vector<std::string> joint_names_;
  std::string root_name_;
  std::string tip_name_;

  // KDL
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;

  // KDL Feedback Variables
  KDL::JntArray q_;
  KDL::JntArray q_dot_;

  // KDL ChainDynParam Variables
  KDL::JntSpaceInertiaMatrix M_;
  KDL::JntArray C_;
  KDL::JntArray G_;
  KDL::Vector grav_;

  // ChainFkSolverVel_recursive
  KDL::FrameVel frame_vel_;
  KDL::JntArrayVel pos_vel_;
  KDL::Vector x_;
  KDL::Vector x_dot_;
  Eigen::Vector3d x_eigen_;
  Eigen::Vector3d x_dot_eigen_;

  // Jacobian
  KDL::Jacobian J_;
  KDL::Jacobian J_dot_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> J_pos_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> J_pos_dot_;
  Eigen::Matrix<double, Eigen::Dynamic, 3> J_pos_pinv_;

  // Impedance shit
  Eigen::Matrix3d Md_;
  Eigen::Matrix3d Bd_;
  Eigen::Matrix3d Kd_;
  KDL::JntArray tau_;

  // Desired Position and Velocity
  Eigen::Vector3d desired_x_;
  Eigen::Vector3d desired_x_dot_;
  Eigen::Vector3d desired_x_dot_dot_;

  // KDL solvers
  boost::scoped_ptr<KDL::ChainDynParam> MCG_solver_;
  boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> Fk_vel_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> Jac_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacDotSolver> Jac_dot_solver_;
};

}  // namespace open_manipulator_controllers

#endif  // OPEN_MANIPULATOR_CONTROLLERS_IMPEDANCE_CONTROLLER_H
