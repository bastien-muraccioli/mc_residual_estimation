/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <Eigen/src/Core/Matrix.h>
#include <string>

/**
 * @brief Source of joint torque measurements used by the momentum observer.
 *
 * - CommandedTorque:       Use the torque commanded by the QP solver (requires
 *                          a friction model to be accurate).
 * - CurrentMeasurement:    Derive torque from motor current (requires motor
 *                          constants and a friction model — not yet implemented).
 * - MotorTorqueMeasurement: Use motor-side torque scaled by the gear ratio
 *                          (requires a friction model — not yet implemented).
 * - JointTorqueMeasurement: Use joint-side torque directly from sensors
 *                          (most accurate when available; rotor inertia effects
 *                          are removed from the inertia matrix in this mode).
 */
enum class TorqueSourceType
{
  CommandedTorque,
  CurrentMeasurement,
  MotorTorqueMeasurement,
  JointTorqueMeasurement,
};

/**
 * @brief Algorithm used to estimate external joint torques.
 *
 * Both methods estimate torques not accounted for in the dynamic model.
 * Contact-constraint torques (model-based reaction forces, not measured) are
 * subtracted in both cases to avoid double-counting.
 *
 * - ForceSensorBased: Projects sensor wrenches into joint torque space via J^T
 *                     and subtracts contact-constraint torques:
 *                     @code
 *                       τ_ext = Σ J_s^T · R^T · F_s  -  τ_contact
 *                     @endcode
 *                     No integration, low latency, limited to sensor coverage.
 *
 * - MomentumObserver: Feeds force-sensor and contact-constraint torques as
 *                     known inputs into the observer, so the integral tracks
 *                     only the unexplained residual:
 *                     @code
 *                       integral += (τ + τ_FT - τ_contact + C^T·qdot - g + r) · dt
 *                       r         = K · (M·qdot - integral + p0)
 *                       τ_ext_hat = τ_FT + r
 *                     @endcode
 *                     The gain-related latency applies only to unmeasured
 *                     forces; forces captured by sensors pass through at full
 *                     bandwidth.
 */
enum class EstimationMethod
{
  MomentumObserver,
  ForceSensorBased,
};

namespace mc_plugin
{

  /**
 * @brief mc_rtc global plugin that estimates external joint torques acting on a
 *        robot and optionally feeds them back to the QP controller.
 *
 * ## Overview
 *
 * The plugin runs at every control cycle (before the QP solve) and produces an
 * estimate of the external torques `τ_ext` acting on the robot joints. Two
 * estimation strategies are available (see EstimationMethod):
 *
 *   1. **Momentum observer** — integrates the generalised momentum residual:
 *      @code
 *        integral += (τ + τ_ext_FT - τ_contact + C^T·qdot - g + r) · dt
 *        r = K · (M·qdot - integral + p0)
 *      @endcode
 *      where `K` is the residual gain, `p0` is the initial momentum, and `r`
 *      is the momentum observer output. Force-sensor torques `τ_ext_FT` are
 *      optionally fused to reduce bias.
 *
 *   2. **Force-sensor based** — computes `τ_ext = J^T · F_sensor - τ_contact`
 *      directly from the measured wrenches.
 *
 * ## Active-joint mask
 *
 * Gripper joints and mimic joints are excluded from the feedback path by an
 * optional binary mask (`useActiveJointsMask_`). The full-DoF dynamics are
 * always computed to preserve physical consistency; the mask is applied only
 * to the torques sent to `setExternalTorques`.
 *
 * ## Backend requirement
 *
 * Contact-constraint torque compensation (`τ_contact`) is only available with
 * the TVM QP backend. A warning is emitted at init if a different backend is
 * detected; the term is set to zero in that case.
 *
 * ## Configuration keys (mc_rtc YAML)
 * | Key                                   | Type   | Description                                                        |
 * |---------------------------------------|--------|--------------------------------------------------------------------|
 * | `residual_gain`                       | double | Observer gain K (higher = faster, noisier)                         |
 * | `torque_source_type`                  | string | One of the TorqueSourceType names                                  |
 * | `estimation_method`                   | string | One of the EstimationMethod names                                  |
 * | `use_active_joints_mask`              | bool   | Whether to mask out gripper/mimic joints in the feedback           |
 * | `use_forces_from_ft_sensors`          | bool   | Whether to fuse force sensor measurements into the observer        |
 * | `use_contact_constraint_compensation` | bool   | Whether to subtract contact-constraint torques from the estimation |
 *
 * ## Datastore interface
 * | Key                                                  | Type   | Description                                                      |
 * |------------------------------------------------------|--------|------------------------------------------------------------------|
 * | `EF_Estimator::isActive`                             | bool   | Whether feedback is applied                                      |
 * | `EF_Estimator::toggleActive`                         | void   | Toggle feedback on/off                                           |
 * | `EF_Estimator::setGain`                              | void   | Set gain and reset observer state                                |
 * | `EF_Estimator::getGain`                              | double | Get current gain value                                           |
 * | `EF_Estimator::isUsingFTSensorMeasurements`          | bool   | Whether force sensor measurements are fused into the observer    |
 * | `EF_Estimator::toggleFTSensorMeasurements`           | void   | Toggle fusion of force sensor measurements into the observer     |
 * | `EF_Estimator::isUsingActiveJointsMask`              | bool   | Whether the active-joint mask is applied to the output           |
 * | `EF_Estimator::toggleActiveJointsMask`               | void   | Toggle the application of the active-joint mask to the output    |
 * | `EF_Estimator::isUsingContactConstraintCompensation` | bool   | Whether contact-constraint torque compensation is applied        |
 * | `EF_Estimator::toggleContactConstraintCompensation`  | void   | Toggle the application of contact-constraint torque compensation |
 */
struct ExternalForcesEstimator : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~ExternalForcesEstimator() override;

private:

// ── Enum ↔ string conversion tables ────────────────────────────────────────
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

  
  // ── Initialisation helpers ──────────────────────────────────────────────────
  void loadConfig(const mc_rtc::Configuration & config);
  void addGui(mc_control::MCGlobalController & controller);
  void addLog(mc_control::MCGlobalController & controller);
  void addDatastoreCall(mc_control::MCGlobalController & controller);

  /**
   * @brief Build the active-joint binary mask.
   *
   * Walks the multibody joint list to stay in sync with the true DoF vector
   * layout. Sets mask entries to 1 for joints that should receive external
   * torque feedback, and 0 for:
   *   - Gripper actuated joints (handled by the gripper controller).
   *   - Mimic joints (kinematically driven, no independent dynamics).
   *   - Fixed joints (zero DoF).
   *
   * The floating base DoFs (if present) are always set to 1.
   *
   * @param robot The controller robot from which the multibody and gripper
   *              information are read.
   */
  void initializeActiveJoints(const mc_rbdyn::Robot & robot);
  void resetMomentumObserver();

  /**
   * @brief Generalised momentum observer for unmeasured external torques.
   *
   * Force-sensor and contact-constraint torques are fed as known inputs so the
   * integral accumulates only the unexplained residual. The gain-related
   * latency therefore applies only to unmeasured forces; sensor-captured forces
   * pass through at full bandwidth.
   *
   * Computed over the full DoF vector to preserve inertia coupling. The
   * active-joint mask is applied in `before()`, not here.
   *
   * @note Updates `tau_contact_` and `tau_ext_diff_` as side effects.
   * @return Full-DoF external torque estimate (unmasked).
   */
  Eigen::VectorXd momentumObserver(mc_control::MCGlobalController & controller);

   /**
   * @brief Force-sensor based external torque estimation.
   *
   * For each force sensor, computes the joint torques induced by the measured
   * wrench via the world-frame Jacobian transpose:
   * @code
   *   τ_FT = Σ_sensors  J_s^T · R^T · F_s
   * @endcode
   * then subtracts the contact-constraint torque obtained from the TVM
   * dynamics function (zero if the TVM backend is not active):
   * @code
   *   return τ_FT - τ_contact
   * @endcode
   *
   * @note Updates `tau_ext_ft_sensor_` and `tau_contact_` as side effects.
   *
   * @return Full-DoF external torque estimate (unmasked).
   */
  Eigen::VectorXd forceSensorBasedEstimation(mc_control::MCGlobalController & controller);

  // ── Configuration ───────────────────────────────────────────────────────────

  TorqueSourceType tau_mes_src_;   ///< Torque source used by the momentum observer.
  EstimationMethod estimation_method_; ///< Active estimation algorithm.
  double residualGain_;            ///< Observer gain K.

  // ── Runtime state ───────────────────────────────────────────────────────────

  int nDof_;                       ///< Full robot DoF count from robot.mb().nrDof().
  bool isActive_ = false;          ///< Whether estimated torques are fed back to the QP.
  bool activeHasChanged_ = true;   ///< Tracks isActive_ transitions to log deactivation once.
  bool useFTSensorMeasurements_ = true;  ///< Whether to use force sensor data.
  bool useActiveJointsMask_ = false;     /// If true, gripper and mimic joint torques are zeroed in the output.
  bool activeJointsInitialized_ = false;
  bool useContactConstraintCompensation_ = false; ///< Whether to subtract contact-constraint torques.

  // ── Observer state ──────────────────────────────────────────────────────────

  Eigen::VectorXd pZero_;          ///< Generalised momentum at initialisation time p(t0) = M(q0)·qdot0.
  Eigen::VectorXd integralTerm_;   ///< Running integral of the momentum observer (full DoF).

  // ── Torque signals ──────────────────────────────────────────────────────────

  /// Final external torque estimate sent to the controller (masked if useActiveJointsMask_).
  Eigen::VectorXd tau_ext_hat_;

  /// Momentum observer residual r = K·(M·qdot - integral + p0).
  Eigen::VectorXd tau_momentum_observer_;

  /// Force-sensor torque projection: Σ J_s^T · R^T · F_s.
  Eigen::VectorXd tau_ext_ft_sensor_;

  /// Net sensor-minus-contact torque fed into the observer: τ_FT - τ_contact.
  Eigen::VectorXd tau_ext_diff_;

  /// Contact-constraint torque from the TVM dynamics function (zero for non-TVM backends).
  Eigen::VectorXd tau_contact_;

  // ── Active-joint mask ───────────────────────────────────────────────────────

  /// Binary mask over the full DoF vector: 1 = include in feedback, 0 = exclude.
  /// Built by initializeActiveJoints(); applied to tau_ext_hat_ in before().
  Eigen::VectorXd activeJoints_;
};

} // namespace mc_plugin
