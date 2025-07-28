/*
 File name: OptimalControllerBase.cpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 2018
 Date last modified: 13.02.2019
 */
#pragma once

#include "ros_custom_controller/ControllerBase.hpp"

#include <ctime>

namespace controller {

template<typename Robot>
class OptimalControllerBase : public ControllerBase<Robot>
{
 public:
  OptimalControllerBase(ros::NodeHandle* nodeHandle, Robot& robot)
      : ControllerBase<Robot>(nodeHandle,robot)
  {
  }

  virtual void create() override
  {
    ControllerBase<Robot>::create();

  }


  virtual void advance(double dt) override
  {
    this->dt_ = dt;
    calculateDynamics();
    solveRiccatiEquation();
    calculateInput();
  }

protected:

  virtual void calculateDynamics(){};

  virtual void solveRiccatiEquation(){};

  virtual void calculateInput(){};

protected:

    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;

    Eigen::MatrixXd P_;

    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;

    Eigen::VectorXd x_;
    Eigen::VectorXd x_des_;
    Eigen::VectorXd u_;

}
;
}
