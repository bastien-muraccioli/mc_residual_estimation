/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>

enum class TorqueSourceType
{
  CommandedTorque,
  CurrentMeasurement,
  MotorTorqueMeasurement,
  JointTorqueMeasurement,
};

enum class EstimationMethod
{
  MomentumObserver,
  ForceSensorBased,
};

namespace mc_plugin
{

struct ExternalForcesEstimator : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~ExternalForcesEstimator() override;

  

private:
  static constexpr std::array<const char *, 4> torqueSourceNames =
  {
    "CommandedTorque",
    "CurrentMeasurement",
    "MotorTorqueMeasurement",
    "JointTorqueMeasurement"
  };

  static constexpr std::array<const char *, 2> estimationMethodNames =
  {
    "MomentumObserver",
    "ForceSensorBased"
  };

  static std::string toString(TorqueSourceType src);
  static TorqueSourceType toTorqueSource(const std::string & s);

  static std::string toString(EstimationMethod method);
  static EstimationMethod toEstimationMethod(const std::string & s);

  void loadConfig(const mc_rtc::Configuration & config);
  void addGui(mc_control::MCGlobalController & controller);
  void addLog(mc_control::MCGlobalController & controller);

  // Return the external torque estimation based on a momentum observer
  Eigen::VectorXd momentumObserver(mc_control::MCGlobalController & controller);

  // Return the external torque estimation based on force sensors measurements, minus the torque from the contact constraint
  // tau_{FT sensor} - J^T * F_{contact constraint}
  Eigen::VectorXd forceSensorBasedEstimation(mc_control::MCGlobalController & controller);

  bool isActive_ = false;
  double residualGain_;
  Eigen::VectorXd pZero_; // Momentum at t0
  TorqueSourceType tau_mes_src_;
  EstimationMethod estimation_method_;
  Eigen::VectorXd tau_ext_hat_; // Estimated external torque
  Eigen::VectorXd tau_momentum_observer_; // External torque estimation from the momentum observer
  Eigen::VectorXd tau_ext_diff_; // Difference between the measured torque and the model-based torque (tau_mes - C*qdot - g)
  Eigen::VectorXd tau_contact_; // Torque from the contact constraint
  Eigen::VectorXd tau_ext_ft_sensor_; // External torque estimation from force sensors
  Eigen::VectorXd integralTerm_;
  Eigen::VectorXd activeJoints_; // Mask for active joints in the estimation (1 for active, 0 for inactive)
};

} // namespace mc_plugin
