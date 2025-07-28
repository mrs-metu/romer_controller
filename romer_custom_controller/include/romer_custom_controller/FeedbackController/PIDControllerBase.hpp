/*
 File name: PIDControllerBase.cpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 2018
 Date last modified: 13.02.2019
 */
#pragma once

#include "ros_custom_controller/FeedbackController/FeedbackControllerBase.hpp"

#include <ctime>

namespace controller {

template<typename Robot>
class PIDControllerBase : public FeedbackControllerBase<Robot>
{
 public:
  PIDControllerBase(ros::NodeHandle *nodeHandle, Robot &robot)
      :
      FeedbackControllerBase<Robot>(nodeHandle, robot)
  {

  }

  virtual ~PIDControllerBase()
  {
  }

  virtual void create() override
  {
    FeedbackControllerBase<Robot>::create();
    k_p_ = Eigen::MatrixXd::Zero(this->robot_.getState().size(), this->robot_.getState().size());
    k_i_ = Eigen::MatrixXd::Zero(this->robot_.getState().size(), this->robot_.getState().size());
    k_d_ = Eigen::MatrixXd::Zero(this->robot_.getState().size(), this->robot_.getState().size());
    //CONFIRM("create : [Feedback_Controller_Base]");
  }

  virtual void readParameters() override
  {
    FeedbackControllerBase<Robot>::readParameters();
    // PARAMETERS
    // K_P
    ros_node_utils::paramRead(this->nodeHandle_,
                              "/" + this->namespace_ + "/controller/P_Controller/KP", k_p_);
    // K_I
    ros_node_utils::paramRead(this->nodeHandle_,
                              "/" + this->namespace_ + "/controller/P_Controller/KI", k_i_);
    ros_node_utils::paramRead(this->nodeHandle_,
                              "/" + this->namespace_ + "/controller/P_Controller/integral_limit",
                              integralLimit_);
    // K_D
    ros_node_utils::paramRead(this->nodeHandle_,
                              "/" + this->namespace_ + "/controller/P_Controller/KD", k_d_);

  }

  virtual void initialize(){
    proportionalTerm_ = Eigen::VectorXd::Zero(this->robot_.getState().size());
    derivativeTerm_ = Eigen::VectorXd::Zero(this->robot_.getState().size());
    integralTerm_ = Eigen::VectorXd::Zero(this->robot_.getState().size());

    integral_ = Eigen::VectorXd::Zero(this->robot_.getState().size());
    integralLimit_ = Eigen::VectorXd::Zero(this->robot_.getState().size());
    x_d_ = Eigen::VectorXd::Zero(this->robot_.getState().size());
    this->x_err_ = Eigen::VectorXd::Zero(this->robot_.getState().size());
    x_err_last_ = Eigen::VectorXd::Zero(this->robot_.getState().size());

  }
 protected:

  virtual void calculateError()
  {

  }

  virtual void calculateInput()
  {
    calculateIntegral();
    proportionalTerm_ = k_p_ * this->x_err_;
    derivativeTerm_ = k_d_ * -this->x_d_;
    integralTerm_ = k_i_ * this->integral_;

    this->robot_.setInput((proportionalTerm_ + derivativeTerm_ + integralTerm_));
  }

  virtual void calculateIntegral()
  {

    Eigen::VectorXd delta = (this->x_err_ + this->x_err_last_) / 2 * this->dt_;

    for (int i = 0; i < integral_.size(); i++) {
      if (integral_[i] + delta[i] > integralLimit_[i]) {
        integral_[i] = integralLimit_[i];
      } else if (integral_[i] + delta[i] < -integralLimit_[i]) {
        integral_[i] = -integralLimit_[i];
      } else {
        integral_[i] += delta[i];
      }
    }

  }

 protected:

  Eigen::MatrixXd k_p_;
  Eigen::MatrixXd k_i_;
  Eigen::MatrixXd k_d_;
  Eigen::VectorXd proportionalTerm_;
  Eigen::VectorXd derivativeTerm_;
  Eigen::VectorXd integralTerm_;

  Eigen::VectorXd feedforward_;

  Eigen::VectorXd x_d_;
  Eigen::VectorXd x_d_des_;
  Eigen::VectorXd x_d_err_;
  Eigen::VectorXd x_err_last_;
  Eigen::VectorXd integral_;
  Eigen::VectorXd integralLimit_;

}
;
}
