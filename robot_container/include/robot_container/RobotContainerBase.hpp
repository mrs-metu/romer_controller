/*
 File name: RobotContainerBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 2018
 Date last modified: 13.02.2019
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <math.h>
#include <memory>
#include <mutex>
#include <Eigen/Dense>

#include <romer_node_utils/RosNodeModuleBase.hpp>

using namespace romer_node_utils;

namespace robot {

class RobotContainerBase : public RosNodeModuleBase
{
 public:
  RobotContainerBase(rclcpp::Node::SharedPtr node)
      : RosNodeModuleBase(node),
        isSimulation_(true),
        dt_(0.0),
        stateMutex_(std::make_unique<std::mutex>())
  {
  }

  virtual ~RobotContainerBase() = default;

  virtual void create() override
  {
    RosNodeModuleBase::create();
   //CONFIRM("create : [Robot_Container_Base]");
  }

  virtual void readParameters() override
  {
    RosNodeModuleBase::readParameters();
    getNode()->declare_parameter("simulation", true);
    
    rclcpp::Parameter sim_param;
    if (paramRead(*getNode(), "simulation", sim_param)) {
      isSimulation_ = sim_param.as_bool();
    }
   //CONFIRM("readParameters : [Robot_Container_Base]");
  }

  virtual void initialize() override
  {
    RosNodeModuleBase::initialize();
   //CONFIRM("initialize : [Robot_Container_Base]");
  }

  virtual void shutdown() override
  {
    RosNodeModuleBase::shutdown();
   //ERROR("shutdown : [Robot_container_base]");
  }

  virtual void initializePublishers() override
  {
    RosNodeModuleBase::initializePublishers();
  }

  virtual void initializeSubscribers() override
  {
    RosNodeModuleBase::initializeSubscribers();
  }

  virtual void initializeServices() override
  {
    RosNodeModuleBase::initializeServices();
  }

  virtual void advance(double dt)
  {
  }

  virtual void publish()
  {
  }

  virtual void setTimeStep(double dt)
  {
    dt_ = dt;
  }

 protected:
  bool isSimulation_;
  std::unique_ptr<std::mutex> stateMutex_;
  double dt_;
};

} // namespace robot
