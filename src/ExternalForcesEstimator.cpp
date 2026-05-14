#include "ExternalForcesEstimator.h"
#include <mc_control/GlobalPluginMacros.h>
#include <mc_control/mc_global_controller.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/gui/ComboInput.h>

#include <RBDyn/Coriolis.h>
namespace mc_plugin
{

ExternalForcesEstimator::~ExternalForcesEstimator() = default;

void ExternalForcesEstimator::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  if(!ctl.controller().datastore().has("extTorquePlugin"))
  {
    ctl.controller().datastore().make_initializer<std::vector<std::string>>("extTorquePlugin", "");
  }

  loadConfig(config);

  // Initialize the momentum observer
  auto & robot = ctl.robot(ctl.robots()[0].name());
  bool robotIsFloatingBase = (robot.mb().nrJoints() > 0 && robot.mb().joint(0).type() == rbd::Joint::Free);
  rbd::ForwardDynamics fd = rbd::ForwardDynamics(robot.mb());
  fd.computeC(robot.mb(), robot.mbc());
  fd.computeH(robot.mb(), robot.mbc());
  Eigen::MatrixXd M = fd.H();
  if (tau_mes_src_ == TorqueSourceType::JointTorqueMeasurement)
  {
    // Removing rotor inertia effects
    M -= fd.HIr();
  }
  Eigen::VectorXd qdot = rbd::sDofToVector(robot.mb(), robot.mbc().alpha);
  pZero_ = M * qdot;
  tau_ext_hat_ = Eigen::VectorXd::Zero(robot.mb().nrDof());
  tau_momentum_observer_ = Eigen::VectorXd::Zero(robot.mb().nrDof());
  integralTerm_ = Eigen::VectorXd::Zero(robot.mb().nrDof());

  // Determine which joints are included in the estimation (floating base joints are included if the robot is floating base, and mimic joints are excluded)
  activeJoints_ = Eigen::VectorXd::Zero(robot.mb().nrDof());
  int jointIndex = 0;
  if(robotIsFloatingBase)
  {
    // First 6 DoFs are floating base, the external forces is estimated
    activeJoints_.head(6).setOnes();
    jointIndex = 6;
  }
  for(std::string joint : robot.refJointOrder())
  {
    int i = robot.jointIndexByName(joint);
    if(robot.mb().joint(i).dof() != 1 || robot.mb().joint(i).isMimic())
    {
      mc_rtc::log::info("[ExternalForcesEstimator][Init] Joint {} is excluded from the estimation.", joint);
    }
    else
    {
      activeJoints_(jointIndex) = 1;
      mc_rtc::log::info("[ExternalForcesEstimator][Init] Estimated joint -> {}", joint);
      jointIndex++;
    }
  }

  // Create datastore's entries to change modify parameters from code
  ctl.controller().datastore().make_call("EF_Estimator::isActive", [this]() { return this->isActive_; });
  ctl.controller().datastore().make_call("EF_Estimator::toggleActive", [this]() { this->isActive_ = !this->isActive_; });
  ctl.controller().datastore().make_call("EF_Estimator::setGain",
                                         [this](double gain)
                                         {
                                           this->tau_ext_hat_.setZero();
                                           this->residualGain_ = gain;
                                         });

  addGui(controller);
  addLog(controller);
  mc_rtc::log::info("[ExternalForcesEstimator][Init] called with configuration:\n{}", config.dump(true, true));
}

void ExternalForcesEstimator::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("[ExternalForcesEstimator][Reset] called");
}

void ExternalForcesEstimator::before(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot();
  
  if(estimation_method_ == EstimationMethod::MomentumObserver)
  {
    tau_ext_hat_ = momentumObserver(controller);
  }
  else if(estimation_method_ == EstimationMethod::ForceSensorBased)
  {
    tau_ext_hat_ = forceSensorBasedEstimation(controller);
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[ExternalForcesEstimator] Invalid estimation method.");
  }

  std::vector<std::string> & extTorquePlugin =
      ctl.controller().datastore().get<std::vector<std::string>>("extTorquePlugin");

  if(isActive_)
  {
    extTorquePlugin.push_back("ResidualEstimator");
  }
  else
  {
    extTorquePlugin.erase(std::remove(extTorquePlugin.begin(), extTorquePlugin.end(), "ResidualEstimator"),
                          extTorquePlugin.end());
  }

  bool onePluginIsActive = false;
  if(extTorquePlugin.size() > 0)
  {
    onePluginIsActive = true;
    for(const auto & pluginName : extTorquePlugin)
    {
      if(pluginName != "ResidualEstimator")
      {
          mc_rtc::log::info(
              "[ExternalForcesEstimator] Another plugin is active: {}, the last plugin sets the external torques.",
              pluginName);
        break;
      }
    }
  }

  if(isActive_)
  {
    robot.setExternalTorques(tau_ext_hat_);
  }
  else if(!onePluginIsActive)
  {
    robot.setExternalTorques(Eigen::VectorXd::Zero(ctl.robot().mb().nrDof()));
  }
}

void ExternalForcesEstimator::after(mc_control::MCGlobalController & controller)
{
}

mc_control::GlobalPlugin::GlobalPluginConfiguration ExternalForcesEstimator::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = false;
  return out;
}

void ExternalForcesEstimator::loadConfig(const mc_rtc::Configuration & config)
{
  // load config
  residualGain_ = config("residual_gain", 0.0);
  std::string source_type = config("torque_source_type", (std::string) "");
  if(source_type.compare("CommandedTorque") == 0)
  {
    tau_mes_src_ = TorqueSourceType::CommandedTorque;
  }
  else if(source_type.compare("CurrentMeasurement") == 0)
  {
    tau_mes_src_ = TorqueSourceType::CurrentMeasurement;
  }
  else if(source_type.compare("MotorTorqueMeasurement") == 0)
  {
    tau_mes_src_ = TorqueSourceType::MotorTorqueMeasurement;
  }
  else if(source_type.compare("JointTorqueMeasurement") == 0)
  {
    tau_mes_src_ = TorqueSourceType::JointTorqueMeasurement;
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[ExternalForceEstimator] error in configuration with entry\"torque_source_type\".\n\tPossible values are: "
        "CommandedTorque, CurrentMeasurement, MotorTorqueMeasurement, JointTorqueMeasurement");
  }

  std::string estimation_method_str = config("estimation_method", (std::string) "");
  if(estimation_method_str.compare("MomentumObserver") == 0)
  {    estimation_method_ = EstimationMethod::MomentumObserver;
  }
  else if(estimation_method_str.compare("ForceSensorBased") == 0)
  {    estimation_method_ = EstimationMethod::ForceSensorBased;
  }
  else
  {    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[ExternalForceEstimator] error in configuration with entry\"estimation_method\".\n\tPossible values are: "
        "MomentumObserver, ForceSensorBased");
  }
}

void ExternalForcesEstimator::addGui(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  ctl.controller().gui()->addElement({"Plugins", "External forces estimator"},
    mc_rtc::gui::Checkbox("Is estimation feedback active", isActive_),
    mc_rtc::gui::NumberInput(
      "Gain", 
      [this]() { return this->residualGain_; },
      [this](double gain)
      {
        if(gain != residualGain_)
        {
          tau_ext_hat_.setZero();
          integralTerm_.setZero();
        }
        residualGain_ = gain;
      }),
    mc_rtc::gui::ComboInput(
      "Estimation Mode", {"Momentum Observer", "Force Sensor Based"},
      [this]() { return estimation_method_; }, 
      [this](const std::string & v) {
        if(v.compare("Momentum Observer") == 0)
        {
          estimation_method_ = EstimationMethod::MomentumObserver;
        }
        else if(v.compare("Force Sensor Based") == 0)
        {
          estimation_method_ = EstimationMethod::ForceSensorBased;
        }
      }),
    mc_rtc::gui::ComboInput(
      "Torque measurement source", {"Commanded Torque", "Current Measurement", "Motor Torque Measurement", "Joint Torque Measurement"},
      [this]() { return tau_mes_src_; }, 
      [this](const std::string & v) {
        if(v.compare("Commanded Torque") == 0)
        {
          tau_mes_src_ = TorqueSourceType::CommandedTorque;
        }
        else if(v.compare("Current Measurement") == 0)
        {
          tau_mes_src_ = TorqueSourceType::CurrentMeasurement;
        }
        else if(v.compare("Motor Torque Measurement") == 0)
        {
          tau_mes_src_ = TorqueSourceType::MotorTorqueMeasurement;
        }
        else if(v.compare("Joint Torque Measurement") == 0)
        {
          tau_mes_src_ = TorqueSourceType::JointTorqueMeasurement;
        }
      })
    );
}

void ExternalForcesEstimator::addLog(mc_control::MCGlobalController & controller)
{
  controller.controller().logger().addLogEntry("ExternalForceEstimator_gain",
                                               [&, this]() { return this->residualGain_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_tauExtHat",
                                               [&, this]() { return this->tau_ext_hat_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_isActive",
                                               [&, this]() { return this->isActive_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_integralTerm",
                                               [&, this]() { return this->integralTerm_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_activeJoints",
                                               [&, this]() { return this->activeJoints_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_tauMomentumObserver",
                                               [&, this]() { return this->tau_momentum_observer_; });
}


Eigen::VectorXd ExternalForcesEstimator::momentumObserver(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());
  int dofNumber = realRobot.mb().nrDof();

  Eigen::VectorXd qdot(dofNumber), tau(dofNumber);
    
  switch(tau_mes_src_)
  {
    case TorqueSourceType::CommandedTorque:
      // Need friction model to finalize
      tau = rbd::sDofToVector(robot.mb(), robot.jointTorque());
      break;
    case TorqueSourceType::CurrentMeasurement:
      mc_rtc::log::error_and_throw<std::runtime_error>("Not implemented yet");
      // Need current to torque conversion, which requires motor constants and friction model
      break;
    case TorqueSourceType::MotorTorqueMeasurement:
      mc_rtc::log::error_and_throw<std::runtime_error>("Not implemented yet");
      // Need friction model to finalize + gear ratio for motor torque 
      // tau = Eigen::VectorXd::Map(realRobot.jointTorques().data(), realRobot.jointTorques().size())
      //       * robot.mb().joint(robot.mb().nrJoints() - 1).gearRatio();
      break;
    case TorqueSourceType::JointTorqueMeasurement:
      tau = Eigen::VectorXd::Map(realRobot.jointTorques().data(), realRobot.jointTorques().size());
      break;
  }

  rbd::ForwardDynamics fd = rbd::ForwardDynamics(realRobot.mb());
  fd.computeC(realRobot.mb(), realRobot.mbc());
  fd.computeH(realRobot.mb(), realRobot.mbc());

  rbd::Coriolis coriolis = rbd::Coriolis(realRobot.mb());
  Eigen::MatrixXd M = fd.H();
  if (tau_mes_src_ == TorqueSourceType::JointTorqueMeasurement)
  {
    // Removing rotor inertia effects
    M -= fd.HIr();
  }

  qdot = rbd::dofToVector(realRobot.mb(), realRobot.alpha());
  Eigen::VectorXd pt = M * qdot; // Momentum at current time

  Eigen::MatrixXd C = coriolis.coriolis(realRobot.mb(), realRobot.mbc());
  Eigen::VectorXd Cqdot_plus_g = fd.C();
  Eigen::VectorXd g = C*qdot - Cqdot_plus_g;

  Eigen::VectorXd tau_contact = ctl.controller().dynamicsConstraint->dynamicFunction().contactTorque();
  Eigen::VectorXd tau_ext_diff = forceSensorBasedEstimation(ctl);

  integralTerm_ += activeJoints_.cwiseProduct((tau + tau_ext_diff + tau_contact + C.transpose() * qdot - g + tau_momentum_observer_) * ctl.timestep());

  tau_momentum_observer_ = activeJoints_.cwiseProduct(residualGain_ * (pt - integralTerm_ + pZero_));

  return tau_momentum_observer_;
}

Eigen::VectorXd ExternalForcesEstimator::forceSensorBasedEstimation(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());
  Eigen::VectorXd tau_ext_ft_sensor = Eigen::VectorXd::Zero(realRobot.mb().nrDof());

  for(const auto & ft_sensor : realRobot.forceSensors())
  {
    // Transformation from parent body origin to sensor frame, used to place
    // the Jacobian at the exact sensor location rather than the body origin,
    // ensuring the moment arm is correct
    const sva::PTransformd & X_p_f = ft_sensor.X_p_f();
    auto jac = rbd::Jacobian(realRobot.mb(), ft_sensor.parentBody(), X_p_f.translation());

    // World-frame Jacobian (6 x path_dof), then expanded to full robot DoF
    // so J^T maps a world-frame wrench to all joint torques
    Eigen::MatrixXd shortJac = jac.jacobian(realRobot.mb(), realRobot.mbc());
    Eigen::MatrixXd fullJac = Eigen::MatrixXd::Zero(6, realRobot.mb().nrDof());
    jac.fullJacobian(realRobot.mb(), shortJac, fullJac);

    // wrenchWithoutGravity returns the wrench in the sensor (body) frame.
    // R.transpose() rotates it to the world frame to match the world-frame
    // Jacobian — virtual work requires both to be expressed in the same frame
    const Eigen::Matrix3d & R = realRobot.bodyPosW(ft_sensor.parentBody()).rotation();
    sva::ForceVecd w = ft_sensor.wrenchWithoutGravity(realRobot);
    w.force() = R.transpose() * w.force();
    w.couple() = R.transpose() * w.couple();

    // τ_ext += J^T * F: project the external wrench into joint torque space
    // and accumulate contributions from all sensors
    tau_ext_ft_sensor += fullJac.transpose() * w.vector();
  }

  Eigen::VectorXd tau_contact = ctl.controller().dynamicsConstraint->dynamicFunction().contactTorque();
  return tau_ext_ft_sensor - tau_contact;;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ExternalForcesEstimator", mc_plugin::ExternalForcesEstimator)
