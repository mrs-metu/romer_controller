/*
 File name: ControllerBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 19.06.2018
 Date last modified: 13.02.2019
 */
#pragma once

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <math.h>
#include <memory>
#include <mutex>
#include <Eigen/Dense>
#include "ros_node_utils/RosNodeModuleBase.hpp"

using namespace ros_node_utils;

namespace controller {
template<typename Robot>
class ControllerBase: public RosNodeModuleBase
{
 public:
  ControllerBase(ros::NodeHandle* nodeHandle, Robot& robot)
      : RosNodeModuleBase(nodeHandle),
        isSimulation_(true),
        dt_(0.0),
        controllerRate_(0),
        robot_(robot)
  {
  }

  virtual ~ControllerBase()
  {
  }


  virtual void create()
  {
    isSimulation_ = true ;
    dt_= 0.0;
    controllerRate_ = 0 ;
   //CONFIRM("create : [Controller_Base]");
  }

  virtual void readParameters()
  {
    paramRead(this->nodeHandle_, "/simulation", isSimulation_);
    if (!isSimulation_) {
    }
    paramRead(this->nodeHandle_, "/" + this->namespace_ + "/controller/rate", controllerRate_);
    dt_ = 1.0 / controllerRate_;
   //CONFIRM("readParameters : [Controller_Base]");
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

  bool isSimulation_;

  std::vector<ros::Publisher> publishers_;
  std::vector<ros::Subscriber> subscribers_;

  Robot& robot_;
};
}  // namespace estimator
