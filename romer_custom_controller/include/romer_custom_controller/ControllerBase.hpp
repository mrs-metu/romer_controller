/*
 File name: ControllerBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 19.06.2018
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

namespace controller {
template<typename Robot>
class ControllerBase: public RosNodeModuleBase
{
 public:
  ControllerBase(const std::string& node_name, Robot& robot)
      : RosNodeModuleBase(node_name),
        isSimulation_(true),
        dt_(0.0),
        controllerRate_(0),
        robot_(robot)
  {
  }

  virtual ~ControllerBase() = default;


  virtual void create()
  {
    isSimulation_ = true ;
    dt_= 0.0;
    controllerRate_ = 0 ;
   //CONFIRM("create : [Controller_Base]");
  }

  virtual void readParameters()
  {

    this->declare_parameter("simulation", true);
    this->declare_parameter("controller/rate", 100.0);

    // Then read them using paramRead
    rclcpp::Parameter sim_param;
    rclcpp::Parameter rate_param;
    
    if (paramRead(*this, "simulation", sim_param)) {
      isSimulation_ = sim_param.as_bool();
    }

    if (paramRead(*this, "controller/rate", rate_param)) {
      controllerRate_ = rate_param.as_double();
    }
    dt_ = 1.0 / controllerRate_;
    rate_ = std::make_shared<rclcpp::Rate>(controllerRate_);
    // CONFIRM("readParameters : [Controller_Base]");
  }

  virtual void advance(double dt)
  {
  }


  virtual void publish()
  {
    robot_.publish();
  }



 protected:

  std::mutex mutex_;
  double dt_;
  double controllerRate_;
  std::shared_ptr<rclcpp::Rate> rate_;

  bool isSimulation_;

  Robot& robot_;
};

} // namespace controller
