/*
 File name: TrajectoryGeneratorBase.cpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 17.09.2019
 Date last modified: 17.09.2019
 */

#pragma once

#include "robot_container/RobotModuleContainerBase.hpp"
#include "ros_node_utils/RosNodeModuleBase.hpp"
#include <string>

template <typename Robot>
class TrajectoryGeneratorBase : public RosNodeModuleBase {
public:
  TrajectoryGeneratorBase(ros::NodeHandle *nodeHandle, Robot &robot)
      : RosNodeModuleBase(nodeHandle), robot_(robot),
        name_("trajectory_generator_base"), referencePointChanged_(false) {}
  virtual ~TrajectoryGeneratorBase() {}

  std::string getName() { return name_; }

  virtual bool isReferenceChanged() { return referencePointChanged_; }

  virtual void advance(double dt) {}

  virtual void publish() {}

  virtual Eigen::Vector3d getDesiredPositionInWorldFrame() {
    return Eigen::Vector3d::Zero();
  }

  virtual Eigen::Quaterniond getDesiredOrientationInWorldFrame() {
    return Eigen::Quaterniond(1, 0, 0, 0);
  }

  virtual Eigen::Vector3d getDesiredLinearVelocityInWorldFrame() {
    return Eigen::Vector3d::Zero();
  }

  virtual Eigen::Vector3d getDesiredAngularVelocityInWorldFrame() {
    return Eigen::Vector3d::Zero();
  }

  virtual Eigen::VectorXd getDesiredStateInWorldFrame(){
    return Eigen::Vector3d::Zero();
  }

  virtual std::vector<Eigen::VectorXd> getTrajectory(int stepNumber,
                                                     double timeStep) {
    return std::vector<Eigen::VectorXd>(0);
  }

protected:
  std::string name_;
  Robot &robot_;

  bool referencePointChanged_;
};
