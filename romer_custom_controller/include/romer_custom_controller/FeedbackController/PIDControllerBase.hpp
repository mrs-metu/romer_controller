/*
 File name: PIDControllerBase.cpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 2018
 Date last modified: 13.02.2019
 */
#pragma once

#include <romer_custom_controller/FeedbackController/FeedbackControllerBase.hpp>

#include <ctime>

namespace controller {

template<typename Robot>
class PIDControllerBase : public FeedbackControllerBase<Robot>
{
 public:
  PIDControllerBase(const std::string& node_name, Robot &robot)
      :
      FeedbackControllerBase<Robot>(node_name, robot)
  {

  }

  virtual ~PIDControllerBase() = default;

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
    
    // Declare all parameters with default values
    // For now this is for robots' parameters not the controller's parameters check if this is correct TODO:
    std::vector<double> default_gains(this->robot_.getState().size(), 0.0);
    this->robot_.declare_parameter("controller/P_Controller/KP", default_gains);
    this->robot_.declare_parameter("controller/P_Controller/KI", default_gains);
    this->robot_.declare_parameter("controller/P_Controller/KD", default_gains);
    this->robot_.declare_parameter("controller/P_Controller/integral_limit", default_gains);

    // Read parameters
    if (paramRead(this->robot_, "controller/P_Controller/KP", k_p)){
      k_p_ = k_p;
    }
    if (paramRead(this->robot_, "controller/P_Controller/KI", k_i)){
      k_i_ = k_i;
    }
    if (paramRead(this->robot_, "controller/P_Controller/KD", k_d)){
      k_d_ = k_d;
    }
    if (paramRead(this->robot_, "controller/P_Controller/integral_limit", integralLimit)){
      integralLimit_ = integralLimit;
    }
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

};
}
