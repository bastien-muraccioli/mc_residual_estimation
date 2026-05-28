#include "ExternalForcesEstimator.h"
#include <mc_control/GlobalPluginMacros.h>
#include <mc_control/mc_global_controller.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/ArrayLabel.h>

#include <RBDyn/Coriolis.h>
namespace mc_plugin
{

ExternalForcesEstimator::~ExternalForcesEstimator() = default;

void ExternalForcesEstimator::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  auto & robot = ctl.robot(ctl.robots()[0].name());
  nDof_ = robot.mb().nrDof();

  loadConfig(config);

  if(useContactConstraintCompensation_ && ctl.controller().dynamicsConstraint->backend() != mc_solver::QPSolver::Backend::TVM)
  {
    mc_rtc::log::warning(
    "[ExternalForcesEstimator] Contact-constraint torque compensation is only available with the TVM backend. " 
    "The current backend will ignore torques induced by contact constraints, "
    "which can lead to large errors in external force estimation when contacts are active.");
  }

  if(useActiveJointsMask_)
  {
    initializeActiveJoints(robot);
    activeJointsInitialized_ = true;
  }
  else
  {
    activeJoints_ = Eigen::VectorXd::Ones(nDof_);
  }
 
  // Initialize the momentum observer
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

  tau_ext_hat_ = Eigen::VectorXd::Zero(nDof_);
  tau_momentum_observer_ = Eigen::VectorXd::Zero(nDof_);
  tau_ext_diff_ = Eigen::VectorXd::Zero(nDof_);
  tau_contact_ = Eigen::VectorXd::Zero(nDof_);
  tau_ext_ft_sensor_ = Eigen::VectorXd::Zero(nDof_);
  integralTerm_ = Eigen::VectorXd::Zero(nDof_);

  addGui(controller);
  addLog(controller);
  addDatastoreCall(controller);
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

  if(robot.encoderVelocities().empty())
  {
    mc_rtc::log::warning("[ExternalForcesEstimator] Encoder velocities are not available, external forces estimation skipped.");
    return;
  }
  
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

  if(useActiveJointsMask_ && !activeJointsInitialized_)
  {
    initializeActiveJoints(robot);
    activeJointsInitialized_ = true;
  }
  else if(!useActiveJointsMask_ && activeJointsInitialized_)
  {
    activeJoints_ = Eigen::VectorXd::Ones(nDof_);
    activeJointsInitialized_ = false;
  }

  // Apply the active joint mask to the estimated external torque
  tau_ext_hat_ = tau_ext_hat_.cwiseProduct(activeJoints_);

  if(activeHasChanged_ != isActive_)
  {
    activeHasChanged_ = isActive_;
    if(!isActive_)
    {
      mc_rtc::log::info("[ExternalForcesEstimator] Estimation feedback deactivated, external torques set to zero.");
      controller.controller().robot().setExternalTorques(Eigen::VectorXd::Zero(nDof_));
      controller.controller().realRobot().setExternalTorques(Eigen::VectorXd::Zero(nDof_));
    }
  }

  if(isActive_)
  {
    controller.controller().robot().setExternalTorques(tau_ext_hat_);
    controller.controller().realRobot().setExternalTorques(tau_ext_hat_);
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
  // residual_gain: 10 # Higher is better, but lead to more noise in the estimation (recommended values are between 10 and dt/2)
  // torque_source_type: CommandedTorque # Options: JointTorqueMeasurement, CommandedTorque, EstimatedTorque
  // estimation_method: ForceSensorBased # Options: MomentumObserver, ForceSensorBased (MomentumObserver includes the use of the FT sensors)
  // use_active_joints_mask: false # If true, the mimic and grippers related joints will be masked out in the estimation
  // use_forces_from_ft_sensors: true # If true, the forces from the FT sensors will be used in the estimation
  // use_contact_constraint_compensation: false # If true, the torques induced by contact constraints will be subtracted from the torque source before computing the residual
  residualGain_ = config("residual_gain", 10.0);
  tau_mes_src_ = toTorqueSource(config("torque_source_type", std::string("CommandedTorque")));
  estimation_method_ = toEstimationMethod(config("estimation_method", std::string("ForceSensorBased")));
  useActiveJointsMask_ = config("use_active_joints_mask", false);
  useFTSensorMeasurements_ = config("use_forces_from_ft_sensors", true);
  useContactConstraintCompensation_ = config("use_contact_constraint_compensation", false);
}

void ExternalForcesEstimator::resetMomentumObserver()
{
  integralTerm_.setZero();
  tau_momentum_observer_.setZero();
}

void ExternalForcesEstimator::addGui(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot();
  std::vector<std::string> jointNames;
  bool robotIsFloatingBase = (robot.mb().nrJoints() > 0 && robot.mb().joint(0).type() == rbd::Joint::Free);
  if(robotIsFloatingBase)
  {
    jointNames.reserve(6 + robot.refJointOrder().size());
    jointNames.insert(jointNames.end(), {"rx", "ry", "rz", "x", "y", "z"});
    jointNames.insert(jointNames.end(), robot.refJointOrder().begin(), robot.refJointOrder().end());
  }
  else {
    jointNames = robot.refJointOrder();
  }

  ctl.controller().gui()->addElement({"Plugins", "External forces estimator"},
    mc_rtc::gui::Checkbox("Is estimation feedback active", isActive_),
    mc_rtc::gui::Checkbox("Use sensor measurements", useFTSensorMeasurements_),
    mc_rtc::gui::Checkbox("Active Gripper & Mimic joints mask", useActiveJointsMask_),
    mc_rtc::gui::Checkbox("Contact constraint compensation", useContactConstraintCompensation_),
    mc_rtc::gui::NumberInput(
      "Gain", 
      [this]() { return residualGain_; },
      [this](double gain)
      {
        if(gain != residualGain_)
        {
          resetMomentumObserver();
        }
        residualGain_ = gain;
      }),
    mc_rtc::gui::ComboInput(
      "Estimation Mode",
      std::vector<std::string>(
          estimationMethodNames.begin(),
          estimationMethodNames.end()),
      [this]() -> std::string
      {
        return toString(estimation_method_);
      },
      [this](const std::string & v)
      {
        estimation_method_ = toEstimationMethod(v);
      }),

    mc_rtc::gui::ComboInput(
      "Torque measurement source",
      std::vector<std::string>(
          torqueSourceNames.begin(),
          torqueSourceNames.end()),
      [this]() -> std::string
      {
        return toString(tau_mes_src_);
      },
      [this](const std::string & v)
      {
        tau_mes_src_ = toTorqueSource(v);
      }),
    mc_rtc::gui::ArrayLabel("Torque Ext Estimated", jointNames, [this]() { return tau_ext_hat_; }),
    mc_rtc::gui::ArrayLabel("Torque Ext from Momentum Observer", jointNames, [this]() { return tau_momentum_observer_; }),
    mc_rtc::gui::ArrayLabel("Torque Ext from Force Sensors", jointNames, [this]() { return tau_ext_ft_sensor_; }),
    mc_rtc::gui::ArrayLabel("Torque Ext from Contact Constraint", jointNames, [this]() { return tau_contact_; })
  );
}

void ExternalForcesEstimator::addLog(mc_control::MCGlobalController & controller)
{
  controller.controller().logger().addLogEntry("ExternalForceEstimator_gain",
                                               [&, this]() { return residualGain_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_tauExtHat",
                                               [&, this]() { return tau_ext_hat_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_isActive",
                                               [&, this]() { return isActive_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_integralTerm",
                                               [&, this]() { return integralTerm_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_activeJoints",
                                               [&, this]() { return activeJoints_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_tauMomentumObserver",
                                               [&, this]() { return tau_momentum_observer_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_tauExtDiff",
                                               [&, this]() { return tau_ext_diff_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_tauContact",
                                               [&, this]() { return tau_contact_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_tauExtFtSensor",
                                               [&, this]() { return tau_ext_ft_sensor_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_useFTSensorMeasurements",
                                               [&, this]() { return useFTSensorMeasurements_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_useActiveJointsMask",
                                               [&, this]() { return useActiveJointsMask_; });
  controller.controller().logger().addLogEntry("ExternalForceEstimator_useContactConstraintCompensation",
                                               [&, this]() { return useContactConstraintCompensation_; });
}

void ExternalForcesEstimator::addDatastoreCall(mc_control::MCGlobalController & controller)
{
  controller.controller().datastore().make_call("EF_Estimator::isActive", [this]() { return isActive_; });
  controller.controller().datastore().make_call("EF_Estimator::toggleActive", [this]() { isActive_ = !isActive_; });
  controller.controller().datastore().make_call("EF_Estimator::setGain",
                                         [this](double gain)
                                         {
                                           resetMomentumObserver();
                                           residualGain_ = gain;
                                         });
  controller.controller().datastore().make_call("EF_Estimator::getGain", [this]() { return residualGain_; });
  controller.controller().datastore().make_call("EF_Estimator::isUsingFTSensorMeasurements", [this]() { return useFTSensorMeasurements_; });
  controller.controller().datastore().make_call("EF_Estimator::toggleFTSensorMeasurements", [this]() { useFTSensorMeasurements_ = !useFTSensorMeasurements_; });
  controller.controller().datastore().make_call("EF_Estimator::isUsingActiveJointsMask", [this]() { return useActiveJointsMask_; });
  controller.controller().datastore().make_call("EF_Estimator::toggleActiveJointsMask", [this]() { useActiveJointsMask_ = !useActiveJointsMask_; });
  controller.controller().datastore().make_call("EF_Estimator::isUsingContactConstraintCompensation", [this]() { return useContactConstraintCompensation_; });
  controller.controller().datastore().make_call("EF_Estimator::toggleContactConstraintCompensation", [this]() { useContactConstraintCompensation_ = !useContactConstraintCompensation_; });
}

Eigen::VectorXd ExternalForcesEstimator::momentumObserver(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(nDof_);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(nDof_);

  Eigen::VectorXd tau_src;
  switch(tau_mes_src_)
  {
    case TorqueSourceType::CommandedTorque:
      // Need friction model to finalize
      tau_src = Eigen::VectorXd::Map(robot.jointTorques().data(), robot.jointTorques().size());
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
      tau_src = Eigen::VectorXd::Map(realRobot.jointTorques().data(), realRobot.jointTorques().size());
      break; 
  }
  tau.head(tau_src.size()) = tau_src;

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

  qdot = rbd::sDofToVector(realRobot.mb(), realRobot.alpha());
  Eigen::VectorXd pt = M * qdot; // Momentum at current time

  Eigen::MatrixXd C = coriolis.coriolis(realRobot.mb(), realRobot.mbc());
  Eigen::VectorXd Cqdot_plus_g = fd.C();
  Eigen::VectorXd g = -(C*qdot - Cqdot_plus_g);

  
  tau_ext_diff_ = forceSensorBasedEstimation(ctl);
  // tau_contact_ is updated in forceSensorBasedEstimation.
  integralTerm_ += (tau + tau_ext_diff_ + tau_contact_ + C.transpose() * qdot - g + tau_momentum_observer_) * ctl.timestep();
  tau_momentum_observer_ = residualGain_ * (pt - integralTerm_ + pZero_);

  return  tau_ext_diff_ + tau_momentum_observer_;
}

Eigen::VectorXd ExternalForcesEstimator::forceSensorBasedEstimation(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());
  tau_ext_ft_sensor_ = Eigen::VectorXd::Zero(nDof_);
  if(useFTSensorMeasurements_)
  {
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
      Eigen::MatrixXd fullJac = Eigen::MatrixXd::Zero(6, nDof_);
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
      tau_ext_ft_sensor_ += fullJac.transpose() * w.vector();
    }
  }

  if(!useContactConstraintCompensation_ || ctl.controller().dynamicsConstraint->backend() != mc_solver::QPSolver::Backend::TVM)
  {
    tau_contact_ = Eigen::VectorXd::Zero(nDof_);
  }
  else
  {
    tau_contact_ = ctl.controller().dynamicsConstraint->dynamicFunction().contactTorque();
  }

  return tau_ext_ft_sensor_ - tau_contact_;
}

std::string ExternalForcesEstimator::toString(TorqueSourceType src)
{
  return torqueSourceNames[static_cast<size_t>(src)];
}

TorqueSourceType ExternalForcesEstimator::toTorqueSource(const std::string & s)
{
  for(size_t i = 0; i < torqueSourceNames.size(); ++i)
  {
    if(s == torqueSourceNames[i])
    {
      return static_cast<TorqueSourceType>(i);
    }
  }

  mc_rtc::log::error_and_throw<std::runtime_error>(
      "[ExternalForceEstimator] Invalid torque source type: {}", s);
}

std::string ExternalForcesEstimator::toString(EstimationMethod method)
{
  return estimationMethodNames[static_cast<size_t>(method)];
}

EstimationMethod ExternalForcesEstimator::toEstimationMethod(const std::string & s)
{
  for(size_t i = 0; i < estimationMethodNames.size(); ++i)
  {
    if(s == estimationMethodNames[i])
    {
      return static_cast<EstimationMethod>(i);
    }
  }

  mc_rtc::log::error_and_throw<std::runtime_error>(
      "[ExternalForceEstimator] Invalid estimation method: {}", s);
}

void ExternalForcesEstimator::initializeActiveJoints(const mc_rbdyn::Robot & robot)
{

  bool robotIsFloatingBase = (robot.mb().nrJoints() > 0 && robot.mb().joint(0).type() == rbd::Joint::Free);
  
  // Collect gripper joints to exclude from estimation
  std::vector<std::string> activeGripperJoints;
  for(const auto & g : robot.grippers())
  {
    for(const auto & n : g.get().activeJoints())
    {
      activeGripperJoints.push_back(n);
    }
  }
  auto isActiveGripperJoint = [&](const std::string & jointName)
  {
    return std::find(activeGripperJoints.begin(), activeGripperJoints.end(), jointName)
           != activeGripperJoints.end();
  };

  // Initialize mask to zero over the full DoF vector
  activeJoints_ = Eigen::VectorXd::Zero(robot.mb().nrDof());

  // Floating base: first 6 DoFs are always estimated
  if(robotIsFloatingBase)
  {
    activeJoints_.head(6).setOnes();
  }

  // Walk the multibody joint list to stay in sync with the actual DoF vector
  // layout. pos tracks the current position in the DoF vector.
  // Joint 0 is either the floating base (Free, 6 DoF, already handled above)
  // or the fixed root (0 DoF), so we start at joint index 1 in both cases.
  int pos = robotIsFloatingBase ? 6 : 0;
  for(int ji = 1; ji < robot.mb().nrJoints(); ++ji)
  {
    const auto & j = robot.mb().joint(ji);
    if(j.dof() != 1)
    {
      // Multi-DoF or 0-DoF joints (fixed, mimic root) — advance pos and skip
      pos += j.dof();
      continue;
    }

    if(!j.isMimic() && !isActiveGripperJoint(j.name()))
    {
      activeJoints_(pos) = 1;
      mc_rtc::log::info("[ExternalForcesEstimator][initializeActiveJoints] Estimated joint (pos {}) -> {}", pos, j.name());
    }
    else
    {
      mc_rtc::log::info("[ExternalForcesEstimator][initializeActiveJoints] Joint {} excluded from estimation.", j.name());
    }

    pos++;
  }

  mc_rtc::log::info("[ExternalForcesEstimator][initializeActiveJoints] Active DoFs: {}/{}",
                    static_cast<int>(activeJoints_.sum()), robot.mb().nrDof());
}

} // namespace mc_plugin


EXPORT_MC_RTC_PLUGIN("ExternalForcesEstimator", mc_plugin::ExternalForcesEstimator)
