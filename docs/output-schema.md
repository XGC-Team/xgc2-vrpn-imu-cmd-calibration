# Output contract

One run creates a timestamped directory containing the original bag, the ROS
parameter snapshot, the run manifest, and these generated files:

- `calibration.yaml`: versioned evidence asset. It preserves separate `C`, `I`
  and `M` semantics, raw estimates, observability, gates, and PASS/FAIL reasons.
- `estimator.yaml`: full estimator configuration when `--base-estimator-yaml`
  is supplied, otherwise a valid ROS parameter overlay. It contains only keys
  already consumed by `estimator_vrpn_px4_rotor_state`.
- `controller.yaml`: a small linkage overlay selecting
  `state_source: state_estimator`; calibration never tunes NMPC weights.
- `report.json`: machine-readable copy for CI, XGC2 Calibration Assets, or a
  Session workflow.

The runtime estimator field `imu_to_vrpn_marker_rpy` is the implementation's
folded `T_BM`; for this planar UGV flow its generated yaw is `T_CM.yaw`. The
tool reports `T_CI.yaw` separately and never adds an unsupported estimator key.
If `T_CI.yaw` or gyro scale is outside the accepted installation range, the
asset fails and `extrinsic_verified` remains false.

The controller does not consume sensor calibration values. It consumes the
estimator's `C`-frame state on `alg/state_estimator/state`, so the generated
controller overlay only prevents the unsafe `vrpn_direct` bypass.
