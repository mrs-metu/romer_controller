/*
 File name: ControllerFrameBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 19.06.2018
 Date last modified: 13.02.2019
 */
 #pragma once

 #include <romer_node_utils/RosNodeModuleBase.hpp>
 #include <thread>
 #include <chrono>
 #include <math.h>
 #include <memory>
 #include <mutex>
 #include <Eigen/Dense>
 
 #include <std_srvs/srv/set_bool.hpp>
 #include <robot_container/RobotContainerBase.hpp>
 #include <romer_custom_estimator/EstimatorBase.hpp>
 #include <romer_custom_hardware_adapter/HardwareBase.hpp>
 
 namespace controller {
 template<typename Robot>
 class ControllerFrameBase : public romer_node_utils::RosNodeModuleBase
 {
  public:
   ControllerFrameBase(const std::string& node_name)
       : romer_node_utils::RosNodeModuleBase(node_name),
         controllerRate_(100),
         isSimulation_(true),
         run_(false),
         dt_(0.0),
         robot_(),
         controllerThread_()
   {
   }
 
   virtual ~ControllerFrameBase() = default;
 
   virtual void create() override
   {
     RosNodeModuleBase::create();
     controllerRate_ = 100;
     isSimulation_ = true;
     run_ = false;
     dt_ = 0.0;
 
     robot_ = std::make_unique<Robot>(this->get_name());
     robot_->create();
   }
 
   virtual void readParameters() override
   {
     RosNodeModuleBase::readParameters();
     
     // First declare the parameters
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
 
     robot_->readParameters();
   }
 
   virtual void initializePublishers() override
   {
     RosNodeModuleBase::initializePublishers();
     robot_->initializePublishers();
 
   }
 
   virtual void initializeSubscribers() override
   {
     RosNodeModuleBase::initializeSubscribers();
     robot_->initializeSubscribers();
   }
 
   virtual void initializeServices() override
   {
     RosNodeModuleBase::initializeServices();
     // Controller Stop Service
 
     stopServices_ = this->create_service<std_srvs::srv::SetBool>(
       this->get_name() + std::string("/controller/stop"),
       std::bind(&ControllerFrameBase::controllerStopServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2));
     robot_->initializeServices();
   }
 
   virtual void initialize() override
   {
     RosNodeModuleBase::initialize();
     robot_->initialize();
   }
 
   virtual void shutdown() override
   {
     std::lock_guard<std::mutex> lock(*shutdownMutex_);
     RosNodeModuleBase::shutdown();
     stop();
 
   }
 
   virtual void advance(double dt)
   {
     dt_ = dt;
     robot_->advance(dt_);
   }
 
   virtual void execute()
   {
     while (rclcpp::ok()) {
       {
         std::lock_guard<std::mutex> lock(*shutdownMutex_);
         if (!isTerminationStarted()) {
           if (run_) {
             advance(dt_);
           }
         } else {
           break;
         }
       }
       rate_->sleep();
     }
     terminate();
   }
 
   void start() override
   {
     controllerThread_ = std::make_unique<std::thread>(&ControllerFrameBase::execute, this);
   }
 
   virtual void stop()
   {
     if (controllerThread_ && controllerThread_->joinable()) {
       controllerThread_->detach();
     }
   }
 
  protected:
   virtual bool controllerStopServiceCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
   {
     run_ = !request->data;
     response->success = true;
     return true;
   }
 
  protected:
 
   std::unique_ptr<std::thread> controllerThread_;
 
   std::mutex mutex_;
   double controllerRate_;
   double dt_;
 
   bool isSimulation_;
   std::string stopServiceName_;
   rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stopServices_;
 // Flag for running
   bool run_;
 
   std::unique_ptr<Robot> robot_;
 };
 }  // namespace controller
 