/*
 File name: FeedbackControllerBase.cpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 2018
 Date last modified: 13.02.2019
 */
#pragma once

#include <romer_custom_controller/ControllerBase.hpp>

#include <ctime>

namespace controller {

template<typename Robot>
class FeedbackControllerBase : public ControllerBase<Robot>
{
 public:
  FeedbackControllerBase(rclcpp::Node::SharedPtr node, Robot& robot)
      : ControllerBase<Robot>(node, robot),
        time_start_(0.0),
        time_stop_(0.0),
        time_start_ros_(0.0),
        time_stop_ros_(0.0)

  {
    time_start_ = clock();
    time_start_ros_ = this->getNode()->now().seconds();
  };

  virtual ~FeedbackControllerBase() = default;

  virtual void create() override
  {
    ControllerBase<Robot>::create();
    x_err_ = Eigen::VectorXd::Zero(this->robot_.getState().size());
   //CONFIRM("create : [Feedback_Controller_Base]");
  };

  virtual void advance(double dt) override
  {
    this->dt_ = dt;
    calculateError();
    calculateInput();

  };

 protected:

  virtual void calculateError(){};

  virtual void calculateInput(){};

  Eigen::VectorXd x_err_;

  double time_start_;
  double time_stop_;
  double time_start_ros_;
  double time_stop_ros_;

};
} // namespace controller
