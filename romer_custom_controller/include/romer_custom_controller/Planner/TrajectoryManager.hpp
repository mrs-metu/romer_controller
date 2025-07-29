/*
 File name: TrajectoryManager.cpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 17.09.2019
 Date last modified: 17.09.2019
 */

#pragma once

#include <string>
#include <unordered_map>

#include <std_msgs/msg/string.hpp>

#include <romer_custom_controller/Planner/TrajectoryGeneratorBase.hpp>
#include <romer_node_utils/RosNodeModuleBase.hpp>
#include <romer_node_utils/ros_node_utils.hpp>

using namespace romer_node_utils;

template <typename Robot> class TrajectoryManager : public RosNodeModuleBase {
public:
  TrajectoryManager(const std::string& node_name)
      : RosNodeModuleBase(node_name),
        generatorNameList_(std::vector<std::string>()),
        trajectoryGenerators_(
            std::vector<std::unique_ptr<TrajectoryGeneratorBase<Robot>>>()),
        generatorId_(-1) {}

  ~TrajectoryManager() = default;

  void create() {
    for (int i = 0; i < generatorNameList_.size(); i++) {
      trajectoryGenerators_[i]->create();
    }

    if (generatorNameList_.size() > 0) {
      generatorName_ = generatorNameList_[0];
      generatorId_ = 0;
    } else {
      generatorName_ = "empty";
      generatorId_ = -1;
    }
  }

  void readParameters() {
    for (int i = 0; i < generatorNameList_.size(); i++) {
      trajectoryGenerators_[i]->readParameters();
    }

    rclcpp::Parameter name_param;
    if (paramRead(*this, "planner/trajectory_generator_name", name_param)) {
      generatorName_ = name_param.as_string();
    }

    for (int i = 0; i < generatorNameList_.size(); i++) {
      if (generatorNameList_[i] == generatorName_) {
        generatorId_ = i;
        break;
      }
    }

    CONFIRM("Trajectory generator is " + generatorName_);
  }

  void initialize() {
    for (int i = 0; i < generatorNameList_.size(); i++) {
      trajectoryGenerators_[i]->initialize();
    }
  }

  void initializePublishers() {
    for (int i = 0; i < generatorNameList_.size(); i++) {
      trajectoryGenerators_[i]->initializePublishers();
    }
  }

  void initializeSubscribers() {
    trajectorySubscriber_ = this->create_subscription<std_msgs::msg::String>(
      "/planner/trajector_manager", 10, std::bind(&TrajectoryManager::trajectoryManagerCallback, this, std::placeholders::_1));
    for (int i = 0; i < generatorNameList_.size(); i++) {
      trajectoryGenerators_[i]->initializeSubscribers();
    }
  }

  void initializeServices() {
    for (int i = 0; i < generatorNameList_.size(); i++) {
      trajectoryGenerators_[i]->initializeServices();
    }
  }

  void publish() {
    if (generatorId_ >= 0) {
      trajectoryGenerators_[generatorId_]->publish();
    }
  }

  void advance(double dt) {
    if (generatorId_ >= 0) {
      trajectoryGenerators_[generatorId_]->advance(dt);
    }
  }

  void addTrajectoryGenerator(
      std::unique_ptr<TrajectoryGeneratorBase<Robot>> generator) {
    generatorNameList_.push_back(generator->get_name());
    trajectoryGenerators_.push_back(std::move(generator));
  }

  std::string getCurrentTrajectoryGenerator() { return generatorName_; }

  void setCurrentTrajectoryGenerator(std::string generatorName) {
    generatorName_ = generatorName;
  }

  Eigen::Vector3d getDesiredPositionInWorldFrame() {
    if (generatorId_ >= 0)
      return trajectoryGenerators_[generatorId_]
          ->getDesiredPositionInWorldFrame();
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Quaterniond getDesiredOrientationInWorldFrame() {

    if (generatorId_ >= 0)
      return trajectoryGenerators_[generatorId_]
          ->getDesiredOrientationInWorldFrame();
    else
      return Eigen::Quaterniond(1, 0, 0, 0);
  }
  Eigen::Vector3d getDesiredLinearVelocityInWorldFrame() {
    if (generatorId_ >= 0)
      return trajectoryGenerators_[generatorId_]
          ->getDesiredLinearVelocityInWorldFrame();
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d getDesiredAngularVelocityInWorldFrame() {
    if (generatorId_ >= 0)
      return trajectoryGenerators_[generatorId_]
          ->getDesiredAngularVelocityInWorldFrame();
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::VectorXd getDesiredStateInWorldFrame(){
      if (generatorId_ >= 0)
      return trajectoryGenerators_[generatorId_]
          ->getDesiredStateInWorldFrame();
    else
      return Eigen::VectorXd::Zero(0);
  }

  /*! \~english
   * @brief calculates the trajectory for given stepNumber
   * @details
   */
  std::vector<Eigen::VectorXd> getTrajectory(int stepNumber, double timeStep) {
    if (generatorId_ >= 0)
      return trajectoryGenerators_[generatorId_]->getTrajectory(stepNumber,
                                                                timeStep);
    else
      return std::vector<Eigen::VectorXd>(0);
  }

  bool isReferenceChanged() {
    if (generatorId_ >= 0)
      return trajectoryGenerators_[generatorId_]->isReferenceChanged();
    else
      return false;
  }

protected:
  void trajectoryManagerCallback(const std_msgs::String &msg) {
    std::string name = msg.data;

    for (int i = 0; i < generatorNameList_.size(); i++) {
      if (generatorNameList_[i] == name) {
        generatorName_ = name;
        generatorId_ = i;
        CONFIRM("Trajectory generator is changed to " + generatorName_);
        break;
      }
    }
  }

protected:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectorySubscriber_;
  std::vector<std::unique_ptr<TrajectoryGeneratorBase<Robot>>>
      trajectoryGenerators_;
  std::vector<std::string> generatorNameList_;
  std::string generatorName_;
  int generatorId_;
};
