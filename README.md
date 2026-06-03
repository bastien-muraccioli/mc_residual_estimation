# ExternalForcesEstimator

An `mc_rtc` global plugin that estimates external joint torques and optionally feeds them back to the QP controller for compliant behaviour.

---

## Configuration

```yaml
residual_gain: 10                    # Observer gain K — higher = faster but noisier (recommended: 10 to dt/2)
torque_source_type: CommandedTorque  # CommandedTorque | JointTorqueMeasurement (¹)
estimation_method: ForceSensorBased  # ForceSensorBased | MomentumObserver
use_active_joints_mask: false        # Zero out gripper and mimic joints in the output
use_forces_from_ft_sensors: true     # Fuse F/T sensor wrenches into the observer
```

> ¹ `CurrentMeasurement` and `MotorTorqueMeasurement` are not yet implemented and fall back to `CommandedTorque`.

---

## Estimation Methods

**`ForceSensorBased`** — projects F/T sensor wrenches into joint space via the Jacobian transpose. Zero latency, limited to sensor coverage.

```
τ_ext = Σ  Jₛᵀ · Rᵀ · wₛ
```

**`MomentumObserver`** — integrates the generalised momentum residual. Handles unmeasured forces; F/T sensors are fused as known inputs so they pass through at full bandwidth.

```
integral += (τ + τ_FT + Cᵀ·q̇ − g + r) · dt
r         = K · (M·q̇ − integral + p₀)
τ_ext_hat = τ_FT + r
```

---

## Datastore Interface

| Key | Signature | Description |
|---|---|---|
| `EF_Estimator::isActive` | `() → bool` | Is feedback currently applied? |
| `EF_Estimator::toggleActive` | `() → void` | Toggle feedback on/off. |
| `EF_Estimator::setGain` | `(double) → void` | Set gain and reset observer state. |
| `EF_Estimator::getGain` | `() → double` | Get current gain. |
| `EF_Estimator::isUsingFTSensorMeasurements` | `() → bool` | Is F/T fusion enabled? |
| `EF_Estimator::toggleFTSensorMeasurements` | `() → void` | Toggle F/T fusion. |
| `EF_Estimator::isUsingActiveJointsMask` | `() → bool` | Is the active-joint mask applied? |
| `EF_Estimator::toggleActiveJointsMask` | `() → void` | Toggle the active-joint mask. |

---

## Notes

- The plugin runs **before** the QP solve on robot index 0.
- The **active-joint mask** excludes gripper, mimic, and fixed joints from the feedback torques while preserving full-DoF dynamics internally.
- The observer **resets** (integral zeroed) on any change to gain, estimation mode, or torque source.
- If encoder velocities are unavailable, estimation is skipped for that cycle with a warning.
- GUI panel available under **Plugins → External forces estimator** for live monitoring and runtime tuning.