#pragma once

#include "ros_custom_controller/ModelPredictiveController/ModelPredictiveControllerBase.hpp"

#include <ctime>

#include <Eigen/Core>
#include "ooqp_eigen_interface/QuadraticProblemFormulation.hpp"
#include "ooqp_eigen_interface/OoqpEigenInterface.hpp"

namespace controller {

template<typename Robot>
class TrajectoryTrackingBase : public ModelPredictiveControllerBase<Robot>
{
 public:
  TrajectoryTrackingBase()
      : ModelPredictiveControllerBase<Robot>()
  {
  }


  virtual ~TrajectoryTrackingBase()
  {
  }


  virtual void create(Robot* r) override
  {
    ModelPredictiveControllerBase<Robot>::create(r);
  }
  

  virtual void initialize(ros::NodeHandle* nodeHandle) override
  {
    ModelPredictiveControllerBase<Robot>::initialize(nodeHandle);
  }

  virtual void readParameters() override
  {
    ModelPredictiveControllerBase<Robot>::readParameters();
  }

  virtual void setCostMatrices(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::MatrixXd R) override
  {
    int n = P.cols();
    int m = R.cols();
    this->Q_ = Eigen::MatrixXd::Identity(n * this->horizonLength_, n * this->horizonLength_);

    this->R_ = Eigen::MatrixXd::Identity(m * this->horizonLength_, m * this->horizonLength_);

    for (int i = 0; i < this->horizonLength_ - 1; i++) {
      this->Q_.block(i * n, i * n, n, n) = Q;
      this->R_.block(i * m, i * m, m, m) = R;
    }
    this->Q_.block(this->horizonLength_ - 1 * n, this->horizonLength_ - 1 * n, n, n) = P;
  }
 protected:

  virtual void initializeCostMatrixes() override
  {
    int n_x = this->b_x_.size();
    int n_u = this->b_u_.size();
    int n_f = this->b_f_.size();
    int N = this->horizonLength_;
    int n = this->robot_->getN();
    int m = this->robot_->getM();

    Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(n, n);

    this->S_x_ = Eigen::MatrixXd::Zero(n * N, n);
    this->S_u_ = Eigen::MatrixXd::Zero(n * N, m * N);
    this->A_bar_ = Eigen::MatrixXd::Zero(N * n, N * n);
    Eigen::MatrixXd G_upper = Eigen::MatrixXd::Zero(n_u * N, m * N);
    Eigen::MatrixXd E_upper = Eigen::MatrixXd::Zero(n_u * N, n);
    Eigen::VectorXd w_upper = Eigen::VectorXd::Zero(n_u * N);
    Eigen::MatrixXd G_lower = Eigen::MatrixXd::Zero(n_x * (N - 1) + n_f, m * N);
    Eigen::MatrixXd E_lower = Eigen::MatrixXd::Zero(n_x * (N - 1) + n_f, n);
    Eigen::MatrixXd E_s_lower = Eigen::MatrixXd::Zero(n_x * (N - 1) + n_f, N * n);
    Eigen::VectorXd w_lower = Eigen::VectorXd::Zero(n_x * (N - 1) + n_f);
    // S_X and S_u_
    for (int i = 0; i < N; i++) {
      // Matrix power
      temp = this->A_;
      for (int j = 0; j < i; j++) {
        temp = this->A_ * temp;
      }
      this->S_x_.block(i * n, 0, n, n) = temp;
    }
    for (int i = 0; i < N; i++) {
      for (int j = 0; j < i; j++) {
        // Matrix power
        temp = this->A_;
        for (int k = 0; k < i - j - 1; k++) {
          temp = this->A_ * temp;
        }
        this->S_u_.block(i * n, j * m, n, m) = temp * this->B_;
      }
      this->S_u_.block(i * n, i * m, n, m) = this->B_;
    }
    for (int i = 0; i < N; i++) {
      this->A_bar_.block(i * n, i * n, n, n) = this->A_;
    }
    // UPPER PART OF Constrains
    for (int i = 0; i < N; i++) {
      G_upper.block(i * n_u, i * m, n_u, m) = this->A_u_;
      w_upper.segment(i * n_u, n_u) = this->b_u_;
    }
    // LOWER PART OF CONSTRAINS
    for (int i = 0; i < N; i++) {
      for (int j = 0; j < i + 1; j++) {
        G_lower.block(i * n_x, j * m, n_x, m) = this->A_x_ * this->S_u_.block(i * n, j * m, n, m);
      }
      E_lower.block(i * n_x, 0, n_x, n) = -this->A_x_ * this->S_x_.block(i * n, 0, n, n);
      E_s_lower.block(i * n_x, i * n, n_x, n) = -this->A_x_ + this->A_x_ * this->A_;
      w_lower.segment(i * n_x, n_x) = this->b_x_;
    }
    for (int j = 0; j < N; j++) {
      G_lower.block((N - 1) * n_x, j * m, n_f, m) = this->A_f_
          * this->S_u_.block((N - 1) * n, j * m, n, m);
    }
    E_lower.block((N - 1) * n_x, 0, n_f, E_lower.cols()) = -this->A_f_
        * this->S_x_.block((N - 1) * n, 0, n, this->S_x_.cols());
    E_s_lower.block((N - 1) * n_x, (N - 1) * n, n_f, n) = -this->A_f_ + this->A_f_ * this->A_;
    w_lower.segment((N - 1) * n_x, n_x) = this->b_f_;
    Eigen::MatrixXd H = this->S_u_.transpose() * this->Q_ * this->S_u_ + this->R_;
    // Concatenate
    this->G_.resize(G_upper.rows() + G_lower.rows(), G_upper.cols());
    this->H_.resize(H.rows(), H.cols());
    this->G_.topRows(G_upper.rows()) = G_upper.sparseView();
    this->G_.bottomRows(G_lower.rows()) = G_lower.sparseView();

    this->E_ = Eigen::MatrixXd::Zero(n_u * N + n_x * (N - 1) + n_f, n);
    this->E_s_ = Eigen::MatrixXd::Zero(n_u * N + n_x * (N - 1) + n_f, n * N);
    this->W_ = Eigen::VectorXd::Zero(w_upper.size() + w_lower.size());
    this->E_.block(0, 0, E_upper.rows(), E_upper.cols()) = E_upper;
    this->E_.block(E_upper.rows(), 0, E_lower.rows(), E_lower.cols()) = E_lower;
    this->E_s_.block(E_upper.rows(), 0, E_s_lower.rows(), E_s_lower.cols()) = E_s_lower;
    //W_.segment(0, w_upper.size()) << w_upper;
    //W_.segment(w_upper.size(), w_lower.size()) << w_lower;
    this->W_ << w_upper, w_lower;
    this->H_ = H.sparseView();
    this->F_ = this->S_x_.transpose() * this->Q_ * this->S_u_;
    //*
    //std::cerr << "Sx\n" << this->S_x_ << std::endl;
    //std::cerr << "SU\n" << this->S_u_ << std::endl;

    //std::cerr << "G_\n" << this->G_ << std::endl;
    //std::cerr << "H_\n" << this->H_ << std::endl;
    //std::cerr << "E_\n" << this->E_ << std::endl;
    //std::cerr << "W_\n" << this->W_ << std::endl;
    //std::cerr << "F_\n" << this->F_ << std::endl;
    //*/
  }


  virtual void calculateCostMatrixes() override
  {
    int n = this->robot_->getN();
    int m = this->robot_->getM();
    Eigen::VectorXd x = this->robot_->getState();
    Eigen::VectorXd x_s = Eigen::VectorXd::Zero(this->horizonLength_ * n);
    std::vector<Eigen::VectorXd> traj = this->robot_->getTrajectory();
    x_s.segment(0, n) = this->robot_->getState();
    for (int i = 0; i < this->horizonLength_; i++) {
      x_s.segment(i * n, n) = traj[i];
    }

    this->q_ = (x.transpose() * this->S_x_.transpose() - x_s.transpose() * this->A_bar_.transpose())
        * this->Q_ * this->S_u_;
    // Constraint
    this->b_ = this->W_ + this->E_ * x + this->E_s_ * x_s;
  }


  virtual void setCommand() override
  {
    this->robot_->setInput( this->solution_.segment(0, this->robot_->getM()));
  }


 public:
  Eigen::MatrixXd getAbar()
  {
    return this->A_bar_;
  }
  Eigen::MatrixXd getEs()
  {
    return this->E_s_;
  }
 protected:

  Eigen::MatrixXd A_bar_;
  Eigen::MatrixXd E_s_;
}
;
}
