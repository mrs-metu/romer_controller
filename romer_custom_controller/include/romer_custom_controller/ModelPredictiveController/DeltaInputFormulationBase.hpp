#pragma once

#include "ros_custom_controller/ModelPredictiveController/ModelPredictiveControllerBase.hpp"

#include <ctime>

#include <Eigen/Core>
#include "ooqp_eigen_interface/QuadraticProblemFormulation.hpp"
#include "ooqp_eigen_interface/OoqpEigenInterface.hpp"

namespace controller {

template<typename Robot>
class DeltaInputFormulationBase : public ModelPredictiveControllerBase<Robot>
{
 public:
  DeltaInputFormulationBase()
      : ModelPredictiveControllerBase<Robot>()
  {
  }
  ;

  virtual ~DeltaInputFormulationBase()
  {
  }
  ;

  virtual void create(Robot* r) override
  {
    ModelPredictiveControllerBase<Robot>::create(r);
  }
  ;

  virtual void initialize(ros::NodeHandle* nodeHandle) override
  {
    ModelPredictiveControllerBase<Robot>::initialize(nodeHandle);
  }

  virtual void readParameters() override
  {
    ModelPredictiveControllerBase<Robot>::readParameters();
  }
  ;

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

    this->S_x_ = Eigen::MatrixXd::Zero(n * (N + 1), n);
    this->S_u_ = Eigen::MatrixXd::Zero(n * (N + 1), m * N);
    Eigen::MatrixXd T_u = Eigen::MatrixXd::Zero(m * N, m * N);
    Eigen::MatrixXd I_u = Eigen::MatrixXd::Zero(m * N, m);
    Eigen::MatrixXd S_delta = Eigen::MatrixXd::Zero(n * (N + 1), m * N);
    Eigen::MatrixXd S_0 = Eigen::MatrixXd::Zero(n * (N + 1), m);
    Eigen::MatrixXd G_upper = Eigen::MatrixXd::Zero(n_u * N, m * N);
    Eigen::MatrixXd E_upper = Eigen::MatrixXd::Zero(n_u * N, n + m);
    Eigen::VectorXd w_upper = Eigen::VectorXd::Zero(n_u * N);
    Eigen::MatrixXd G_lower = Eigen::MatrixXd::Zero(n_x * N + n_f, m * N);
    Eigen::MatrixXd E_lower = Eigen::MatrixXd::Zero(n_x * N + n_f, n + m);
    Eigen::VectorXd w_lower = Eigen::VectorXd::Zero(n_x * N + n_f);

    // this->S_x_ and this->S_u_
    this->S_x_.block(0, 0, n, n) = Eigen::MatrixXd::Identity(n, n);
    for (int i = 1; i < N + 1; i++) {
      // Matrix power
      temp = this->A_;
      for (int j = 1; j < i; j++) {
        temp = this->A_ * temp;
      }
      this->S_x_.block(i * n, 0, n, n) = temp;
    }

    for (int i = 1; i < N + 1; i++) {
      for (int j = 0; j < i - 1; j++) {
        // Matrix power
        temp = this->A_;
        for (int k = 1; k < i - j - 1; k++) {
          temp = this->A_ * temp;
        }
        this->S_u_.block(i * n, j * m, n, m) = temp * this->B_;
      }
      this->S_u_.block(i * n, i - 1 * m, n, m) = this->B_;
    }

    for (int i = 0; i < N; i++) {
      for (int j = 0; j < i + 1; j++) {
        T_u.block(i * m, j * m, m, m) = Eigen::MatrixXd::Identity(m, m);
      }
      I_u.block(i * m, 0, m, m) = Eigen::MatrixXd::Identity(m, m);
    }
    S_delta = this->S_u_ * T_u;
    S_0 = this->S_u_ * I_u;

    // UPPER PART OF Constrains
    for (int i = 0; i < N; i++) {
      for (int j = 0; j < i + 1; j++) {
        G_upper.block(i * n_u, j * m, n_u, m) = this->A_u_;
        E_upper.block(i * n_u, n, n_u, m) = -this->A_u_;
      }
      w_upper.segment(i * n_u, n_u) = this->b_u_;
    }
    // LOWER PART OF CONSTRAINS
    w_lower.segment(0, n_x) = this->b_x_;
    E_lower.block(0, 0, n_x, n) = -this->A_x_;
    E_lower.block(0, n, n_x, m) = -this->A_x_ * S_0.block(0, 0, n, m);
    for (int i = 1; i < N; i++) {
      for (int j = 0; j < i; j++) {
        G_lower.block(i * n_x, j * m, n_x, m) = this->A_x_ * S_delta.block(i * n, j * m, n, m);
      }
      E_lower.block(i * n_x, 0, n_x, n) = -this->A_x_ * this->S_x_.block(i * n, 0, n, n);
      E_lower.block(i * n_x, n, n_x, m) = -this->A_x_ * S_0.block(i * n, 0, n, m);
      w_lower.segment(i * n_x, n_x) = this->b_x_;
    }
    for (int j = 0; j < N; j++) {
      G_lower.block(N * n_x, j * m, n_f, m) = this->A_f_ * S_delta.block(N * n, j * m, n, m);
    }
    E_lower.block(N * n_x, 0, n_f, n) = -this->A_f_ * this->S_x_.block(N * n, 0, n, n);
    E_lower.block(N * n_x, n, n_f, m) = -this->A_f_ * S_0.block(N * n, 0, n, m);
    w_lower.segment(N * n_x, n_x) = this->b_f_;

    // Concatenate
    this->G_.resize(G_upper.rows() + G_lower.rows(), G_upper.cols());
    this->G_.topRows(G_upper.rows()) = G_upper.sparseView();
    this->G_.bottomRows(G_lower.rows()) = G_lower.sparseView();

    this->E_ = Eigen::MatrixXd::Zero(n_u * N + n_x * N + n_f, n + m);
    this->W_ = Eigen::VectorXd::Zero(w_upper.size() + w_lower.size());

    this->E_.block(0, 0, E_upper.rows(), E_upper.cols()) = E_upper;
    this->E_.block(E_upper.rows(), 0, E_lower.rows(), E_lower.cols()) = E_lower;

    //W_.segment(0, w_upper.size()) << w_upper;
    //W_.segment(w_upper.size(), w_lower.size()) << w_lower;
    this->W_ << w_upper, w_lower;
    Eigen::MatrixXd H = S_delta.transpose() * this->Q_ * S_delta + T_u.transpose() * this->R_ * T_u;
    this->H_.resize(H.rows(), H.cols());
    this->H_ = H.sparseView();
    this->F_ = this->S_x_.transpose() * this->Q_ * S_delta;
    this->F_U_ = S_0.transpose() * this->Q_ * S_delta + I_u.transpose() * this->R_ * T_u;

    /*
    std::cerr << "Sx\n" << this->S_x_ << std::endl;
    std::cerr << "SU\n" << this->S_u_ << std::endl;
    std::cerr << "I\n" << I_u << std::endl;
    std::cerr << "T\n" << T_u << std::endl;

    std::cerr << "S_delta\n" << S_delta << std::endl;
    std::cerr << "S_0\n" << S_0 << std::endl;
    //*/
  }

  ;

  virtual void calculateCostMatrixes() override
  {
    Eigen::VectorXd delta_x = this->robot_->getState() - this->robot_->getDesiredState();
    Eigen::VectorXd u = this->robot_->getInput();
    int n = this->robot_->getN();
    int m = this->robot_->getM();
    this->q_ = delta_x.transpose() * this->F_ + u.transpose() * this->F_U_;
    this->b_ = (this->W_ + this->E_.block(0, 0, this->E_.rows(), n) * delta_x
        + this->E_.block(0, n, this->E_.rows(), m) * u);
  }
  ;

  virtual void setCommand() override
  {
    this->robot_->setInput(
        this->robot_->getInput() + this->solution_.segment(0, this->robot_->getM()));
    // TODO : Use this solution for warm start in future
    //this->solution_.segment(0, this->solution_.size() - this->robot_->getM()) = this->solution_
    //    .segment(this->robot_->getM(), this->solution_.size());
    //this->solution_.segment(this->solution_.size() - this->robot_->getM(), this->solution_.size()) =
     //   Eigen::VectorXd::Zero(this->robot_->getM());
  }
  ;

 public:
  Eigen::MatrixXd getFu()
  {
    return this->F_U_;
  }
 protected:

  Eigen::MatrixXd F_U_;
}
;
}
