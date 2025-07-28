/*
 File name: FeedbackControllerBase.cpp
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
class FeedbackControllerBase : public ControllerBase<Robot>
{
 public:
  FeedbackControllerBase(ros::NodeHandle* nodeHandle, Robot& robot)
      : ControllerBase<Robot>(nodeHandle,robot),
        time_start_(0.0),
        time_stop_(0.0),
        time_start_ros_(0.0),
        time_stop_ros_(0.0)

  {
    time_start_ = clock();
    time_start_ros_ = ros::Time::now().toSec();
  }
  ;

  virtual ~FeedbackControllerBase()
  {
  }
  ;

  virtual void create() override
  {
    ControllerBase<Robot>::create();
    x_err_ = Eigen::VectorXd::Zero(this->robot_.getState().size());
   //CONFIRM("create : [Feedback_Controller_Base]");
  }
  ;

  virtual void advance(double dt) override
  {
    this->dt_ = dt;
    calculateError();
    calculateInput();
    //time_stop_ = clock();
    //time_stop_ros_ = ros::Time::now().toSec();
    //std::cout << "dt : " <<this->dt_<<" sec" << std::endl;
    //std::cout << "Time: " << (time_stop_-time_start_)/double(CLOCKS_PER_SEC) << " sec "<< std::endl;
    //std::cout << "Rime: " << time_stop_ros_-time_start_ros_<<" sec" <<std::endl;
    //std::cout << "_____________" ;

    //time_start_ = clock();
    //time_start_ros_ = ros::Time::now().toSec();

  }

 protected:

  virtual void calculateError(){};

  virtual void calculateInput(){};

  Eigen::VectorXd x_err_;

  double time_start_;
  double time_stop_;
  double time_start_ros_;
  double time_stop_ros_;

}
;
}
