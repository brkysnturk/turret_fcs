# Changelog

All notable changes to the Turret FCS (Autonomous Air Defense Turret Fire Control System) project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.4.0] - 2026-10-24

v1.4 consolidates every post-v1.3 change: the repo-wide audit, P 0.22/0.11, the RL wrapper, and the pure-RL rescue with correct plant physics and a progress reward.

### Added
- **Pure RL (v1.4):** `turret_fcs/rl_controller_max.py` now emits the PPO action directly; the v1.3 `0.3 u_RL + 0.7 u_P` blend is removed. The v1.4 policy is `turret_ppo_v14.zip` / `turret_ppo_v14_policy.npz`, loaded through an 11-64-64-2 NumPy MLP so inference does not depend on SB3 or torch.
- **Progress reward and corrected plant:** `TurretEnv` now models a 1.20 deg backlash behind a command accumulator, keeps the 0.10 deg servo deadband separate, uses 1.50 deg per-frame pan/tilt steps, and trains with `r = 2.0*(e_prev^2 - e^2) - 0.05*e^2 - 0.02*|u|^2 - 0.08*|du|^2 + bullseye`. Webots motor velocity is 1.50 deg/frame instead of 1.35.
- **Pure RL Webots evidence:** profile 2 gives 1.93 deg pan RMS, 40.6 px, 36.3% lock, 92.5% locked, 4.4% reversal, 0.78 deg tilt; P gives 2.07 deg, 42.3 px, 39.8%, 82.2%, 0.9%, 1.34 deg. Zigzag and figure-8 improve similarly; see `docs/evidence/summary.md`.
- **RL velocity-observation guard:** the policy is trained with true velocity but the Kalman velocity is zeroed at runtime; noisy Kalman velocity saturated the policy on the Webots plant. Lead/HUD still use the Kalman estimate.
- **Webots flywheel idle and fixed settle:** flywheel pre-spins while the target is locked; firing sequence is `SPINUP/SLEW 0.25 s -> SETTLE 0.15 s -> FIRING 80 ms -> RECOVER 350 ms -> EVAL 1500 ms`.
- **Firmware flywheel idle:** STM32 keeps the flywheel energised in `LOCKED`/`FIRING` states, stops it in `TRACKING`, and retains the physical-fire safety switch (`ENABLE_PHYSICAL_FIRE`, default `0`).
- **LiDAR range gating hardening:** firing-time range freeze, 50 cm jump rejection, 3-frame/10 cm stability gate, 3-frame reacquire, and optional geometry check for jump candidates.
- **P:** single EMA, Kp pan/tilt 0.22/0.11, acceleration limits 120/80 deg/s^2, soft deadband 0.85/0.95 deg; legacy P remains selectable with `TURRET_P_CONTROLLER=legacy`.
- **Webots tuning harness:** `webots/tuning_params.json`, runtime Kp/max-output overrides and tuning metadata columns.

### Fixed
- **Target selection dropout hysteresis and ID-switch gate:** `is_tracking` became false after the first missing frame, so the relaxed 0.16 tracking confidence was never used during the 5-frame dropout tolerance. It now stays true while `lost_counter <= max_lost_frames`, and a ByteTrack ID that jumps farther than 250 px is rejected in favour of a spatially consistent candidate.
- **UART packet reassembly and per-axis residue:** the reader previously expected a full 10-byte telemetry packet in one `read` call, so split packets were discarded. A persistent RX buffer now resynchronizes on the 0x55 header, verifies checksums, and keeps partial packets. Quantization residue is reset per axis; an explicit COM port is tried before ST-Link auto-detection, and a missing pyserial install no longer crashes the reader.
- **Camera auto-detection:** the preferred name was `Logi C270`, which does not match the Windows device name `Logitech C270`; the substring is now `C270`.
- **Webots tuning defaults:** the fallback gains were the legacy 0.11/0.06, silently overriding P when `tuning_params.json` was absent. Defaults now come from `CONFIG.control.kp_pan_max/kp_tilt_max` (0.22/0.11).
- **HUD spin-up countdown:** the combat panel used the 1.0 s cold-start constant even when the idle 0.25 s spin-up was active. The actual duration is passed from `main.py` and `webots_main.py`; SETTLE gets its own panel state.
- **Telemetry semantics:** production and Webots logs wrote the P controller's filtered `actual_pan/tilt` as if it were the camera angular error. They now use the measured bbox error (`last_pan_err_deg/last_tilt_err_deg`), and `latency_ms` is the measured loop processing time rather than `dt * 1000`. Webots logs now use the real target confidence and percentage hit probability instead of hardcoded 0.90 and a 0-1 ratio.
- **Firmware cleanup:** removed unused `FCS_Comm_IsNewCommand/GetPan/GetTilt/GetState`, `Luna_IsDataReady`, and the redundant `PAN_DIR_INVERT/TILT_DIR_INVERT` double-inversion path. Direction inversion remains solely in the PC SSOT.
- **Pure-RL instability:** the first v1.4 attempts used 1.20 deg as the servo deadband, which swallowed the 1.50 deg step and killed motion; later attempts over-penalized the action derivative and saturated. The final model separates the 0.10 deg deadband from the 1.20 deg backlash, uses a progress reward, and zeroes the noisy Kalman velocity observation at runtime.

### Removed
- **P+RL blend:** `blend_with_p`, `blend_commands()`, and the `0.7 P + 0.3 RL` path are gone; RL is pure PPO.
- **Obsolete Webots analysis suite:** `bayesian_tune.py` (FF tuner for a disabled path), `compare_webots_controllers.py`, `generate_10_shot_performance_analysis.py`, `plot_fcs_analysis.m`, and their old PNG/CSV outputs. Current validation lives in `analysis/p_profile_sweep.py`, `analysis/p_controller_validation.py`, `analysis/rl_webots_eval.py`, and `analysis/rl_diagnose.py`.
- **Dead artifacts:** `legacy/hsv` C++ prototype, empty pip build/cache directories, `.pytest_cache`, `egg-info`, YOLO training curve/confusion images, and the superseded architecture PNGs. Production weights (`best.pt`, `yolov8n.pt`) and the final evidence figures remain.
- **GitHub hygiene:** deleted the stale `back_up` tag and the local `backup_before_email_rewrite` / `backup_main_before_squash` branches. Release tags v1.0-v1.3 and all four releases remain.

### Changed
- **Single controller per mode, no rollback shims:** P is now the production `turret_fcs/p_controller.py` (`PController`) and RL is now `turret_fcs/rl_controller.py` (`RLController`). The legacy P/RL controllers, the `p_controller_factory` / `rl_controller_factory` selectors, the `p_controller_impl` / `rl_controller_impl` config fields, the `TURRET_P_CONTROLLER` / `TURRET_RL_CONTROLLER` environment switches, the `kp_pan_max` / `kp_tilt_max` aliases, and the v1.3 `turret_ppo_final` model are deleted. Rollback to v1.3 now means checking out the `v1.3` tag.
- **Dead control config removed:** `velocity_ff_*`, `ff_min_speed_*`, `ff_full_speed_deg_s`, `ff_slew_rate_deg_s`, `ff_error_ramp_deg`, `ff_stop_*`, `p_pan_ema_alpha`, `p_pan_slew_deg`, `rl_pan_ema_alpha`, and `rl_pan_slew_deg` were never read by the production controllers; `alpha_filter` now matches the P controller (0.85).
- **Install reliability:** `setup_env.bat` now picks `py -3.12/3.11/3.10` or a valid `python`, rebuilds a broken `venv`, installs the CPU-only PyTorch wheel, and verifies imports. `requirements.txt` resolves again: stable-baselines3 2.4.1 with gymnasium 1.0.0 and numpy 1.26.4 (the previous SB3 2.3.2 / gymnasium 1.0.0 / numpy 2.1.3 pins could not resolve). `requirements.txt`, `pyproject.toml`, and `setup_env.bat` now agree on Python 3.10-3.12.
- **`main.bat` fails clearly:** it now stops with instructions when `venv` is missing instead of falling back to a system Python that may lack dependencies, and it no longer pauses in CI.
- **pytest collection:** `testpaths = ["tests"]` and the Webots controller path are configured, so a fresh clone's `pytest` run no longer tries to import Webots controller modules.
- **Test suite now 38 tests:** added target-selector hysteresis/ID-gate and UART packet/residue regression coverage; the obsolete P-blend test was removed. CI remains green across Python 3.10-3.12 and Windows.
- **Test suite consolidated from 67 to 33 tests:** duplicated controller/Kalman/LOS/HUD/firmware tests were merged into `tests/test_kalman.py`, `tests/test_p_controller.py`, `tests/test_aux_modules.py`, and `tests/test_rl_controller.py`. Removed tests for dead velocity feedforward and the legacy RL smoothing path, plus per-file bootstrap duplication. Coverage of the active paths is unchanged; the suite runs in about 4 seconds.
- **Field rollback to 0.22/0.11 after real-turret resonance:** Kp 0.30 resonated on the physical turret even though Webots, with its 1.2 deg backlash model, had ranked it best. The default is now pan 0.22 / tilt 0.11, which sits inside the stable band of every Webots profile while leaving margin for real gear backlash and potentiometer noise. `webots/tuning_params.json` follows the same values. If it still chatters on hardware, drop to 0.18/0.09; 0.30 and above must not be used on the real turret.
- **P gain swept across six motion profiles:** the single-profile sweep was not enough to prove stability under abrupt direction changes. A 6-profile by 8-gain sweep (`analysis/p_profile_sweep.py`) shows no single universal Kp: standard sine holds to 0.35 and collapses at 0.39, figure-8 holds to 0.32, fast sine is flat across 0.20-0.32, zigzag holds to 0.27 (89.6% lock) and limit-cycles at 0.30+, and step/hold is unstable even at 0.20 (25% command saturation) with 0% lock at 0.30. Limit cycles are detected by command reversal rate (8%+ vs under 2% in stable runs) and command saturation (50%+ vs near 0).
- **Profile-aware gain caps:** `webots_tuning.py` caps pan gain per profile (standard/figure-8 0.32, fast sine 0.25, zigzag 0.27, step/hold 0.15), with tests covering each profile.
- **P gains swept in Webots and raised to 0.30/0.13:** the initial 0.20/0.12 defaults were conservative. A profile-2 sweep with identical 90 s runs gives pan RMS error and command reversal rates of 0.20: 2.25 deg / 0.8%, 0.28: 2.13 / 1.0%, 0.30: 2.12 / 1.2%, 0.35: 2.10 / 2.7%, 0.39: 3.53 / 8.2%, 0.42: 4.19 / 8.1%, 0.50: 4.63 / 8.1%. The stability cliff sits between 0.35 and 0.39, so 0.30/0.13 is the new default: it has the lowest late-phase error and the highest lock time (90.9%) at 1.2% reversal. The 0.30 run reproduced exactly. Tilt 0.13 beats 0.10 (tilt error 1.76 vs 1.83 deg).
- **Tuning limits no longer clamp silently:** the Webots tuner previously limited `kp_pan` to 0.20, so every sweep above that value silently ran at 0.20. Limits now allow the swept range, clamping prints a `[TUNE UYARI]` line, and the fast profile has its own limit. Added regression tests for gain passthrough, clamp warnings, and profile limits.
- **P controller redesigned as P (legacy kept selectable):** new `turret_fcs/p_controller_max.py` removes the double EMA and the fixed 0.20 deg/frame pan slew, which capped pan speed at 6 deg/s. It uses one EMA (0.85), pan/tilt Kp of 0.30/0.13 with a smooth 1.6x gain increase above 5 deg, and command acceleration limits of 120/80 deg/s^2 instead of a hard speed cap. Ki, Kd, and velocity feedforward remain absent; the soft 0.85/0.95 deg deadband is preserved. `p_controller_factory.py` selects the implementation, defaults to `max`, and supports rollback with `TURRET_P_CONTROLLER=legacy`. `webots/tuning_params.json` baseline is 0.30/0.13 with FF 0.0. Test suite was 67 tests at that stage.
- **Kalman filter redesigned from scratch (CV + robust single-frame gate):** state is `[x, y, vx, vy]` with one process noise (`sigma_a=300 px/s^2`), one measurement noise (`r_var=64 px^2`), and Joseph-form covariance. Removed adaptive Q, sigma rate limiting, reversal state overwrite/P inflation, acceleration clamp, 3-frame measurement slope, output EMA, reacquire holdoff, distance-based deadband, and velocity clamp. A single >150 px bbox jump inflates R 100x; three consecutive jumps reinitialize the track (position from measurement, velocity zero). Ego now comes only from a 30 ms command-step servo model because STM feedback is quantized and jumpy; lead uses `bbox + v * lead_time` instead of adding the filter position lag. Real-log world-LOS validation improved correlation 0.44 -> 0.83 and sign agreement 66% -> 77% (92% above 120 px/s); Webots ground truth confirms 87-92% sign agreement. Test suite was 46 tests at that stage.
- **Fire timing documentation:** idle flywheel spin-up reduced to 0.25 s; separate 0.15 s settle stage added before solenoid release.
- **Kalman velocity-confidence gate:** two consecutive measurement-direction frames are required for valid pan/tilt velocity; reversal, high innovation, or inconsistent direction sets velocity to zero, and FCS suppresses P FF/RL velocity on that frame.
- **P-only experiment switch:** `velocity_ff_enabled=False` disables P pan/tilt FF while leaving Kp active; the Kalman gate was loosened to a 10 px/s minimum and accepts low-innovation sign changes so velocity is not permanently zeroed.
- **CI repair:** pinned runtime versions could not resolve together (stable-baselines3 2.4 requires numpy<2, lapx has no Linux wheel for every matrix python). CI now installs `requirements-ci.txt` (headless OpenCV, numpy 1.26, SB3 2.4) plus CPU torch, and the Windows job creates its own venv instead of running `setup_env.bat`. A clean venv run passes all 38 tests.
- **LOS calibration from real log:** alpha-beta tuned to 0.70/0.20 by grid search (mean residual 0.62° -> 0.49°); the tilt max of 31.5° was traced to the turret angle itself, not a LOS offset error.
- **Velocity clamp widened:** `max_vel_px_s` raised from 500 to 700 because large YOLO box jumps saturated the clamp (217 frames clipped); the 3-frame slope plus acceleration limit still suppress spikes.
- **Kalman reversal fix:** measurement velocity is EMA-smoothed; on a confirmed reversal the velocity state is set to the measured new direction and Q is boosted for two frames. Synthetic tests show one false-free reversal and a 1-frame true-reversal response.
- **Single ego source:** ego compensation always uses the smooth 45 ms servo model; UART pose deltas no longer switch the ego source frame by frame.
- **Kalman reversal consistency:** reversal covariance inflation is configurable (`reversal_p_inflate`), cross-covariance terms are zeroed on a reversal so `P` stays symmetric, and the lead gate now ramps smoothly between `lead_gate_low_px_s` (40) and `lead_gate_high_px_s` (60).
- **HUD lead always blue:** removed the gray `LEAD?` confidence state; the lead marker and label are always drawn in blue.
- **Velocity-confidence gate removed:** with FF disabled, the gate only zeroed HUD/RL velocity, so `velocity_valid_pan/tilt`, its two-frame direction check, and the FCS zeroing logic were deleted.
- **HUD lead EMA removed:** `pred_center` follows the Kalman prediction directly, removing the 0.70/0.30 display lag; the firing lead offset was already EMA-free.
- **Lead gate removed and deadband softened:** the separate lead threshold is gone (lead uses full velocity), the hard 50 px/s velocity deadband is replaced by a soft 30–60 px/s band, and measurement velocity now comes from a 3-frame slope instead of a single-frame difference plus EMA.
- **Reversal EMA removed:** the separate `reversal_ema_alpha` was nearly identical to `velocity_ema_alpha` (0.55 vs 0.50); output velocity now uses a single EMA.
- **Webots balloon suite:** eight selectable motion profiles (static, slow/standard/fast sine, step-hold, zigzag, tilt-only, figure-8), profile selection through `webots/balloon_profile.json` or `WEBOTS_BALLOON_PROFILE`, and a 30 cm balloon collision sphere/visual scale.
- **Webots realism knobs:** deterministic 3 px bbox jitter, 5 mm balloon tremor, and an adjustable 40–120 ms observation delay (`WEBOTS_BBOX_JITTER`, `WEBOTS_BALLOON_TREMOR`, `WEBOTS_OBS_DELAY_S`).
- **World LOS prototype:** alpha-beta `LosTracker` from turret angle plus camera error; pan/tilt angle and angular rate are logged as four telemetry columns but not wired into control.
- **HUD debug lead:** lead marker is back as a diagnostic, blue when Kalman pan/tilt velocity is valid and gray with `LEAD?` when invalid; it is visual-only because P FF is disabled.
- **Test suite:** consolidated from 40 to 41 behavior tests; added Kalman velocity-confidence, LOS-prototype, LOS-telemetry, soft-deadband, and full-lead coverage.
- **System identification labeling:** `analysis/system_identification.py` is now explicitly marked as a synthetic nominal-model analysis; it does not fit real telemetry or provide measured actuator `tau`/backlash.
- **Portability hardening:** dependencies are exactly pinned in `requirements.txt` and `pyproject.toml`, `setup_env.bat` enforces Python 3.9-3.12, `weights/yolov8n.pt` is tracked as the fallback detector, and CI now runs a clean Windows `setup_env.bat` install plus import/test smoke.

### Fixed
- **Webots ballistic range source:** virtual ballistics now uses the FCS-frozen `last_valid_target_dist_m` instead of raw LiDAR `dist_cm`, preventing 230 cm wall-range shots.
- **RL feedforward double compensation:** Webots velocity FF is no longer added on top of the RL policy.
- **Camera capture throughput:** Windows camera capture now prefers MSMF with a DSHOW fallback; on the C270 test bench this restored 720p MJPG capture from ~7 FPS to ~30 FPS.
- **RL frame-rate scaling:** RL per-frame action limits are scaled by `dt / 0.03333` (capped by the controller output limit), preserving angular speed when the real loop runs below 30 FPS.
- **P frame-rate scaling:** P step limits now use the actual `dt` and the 1.5 deg firmware command cap, preserving angular rate below 30 FPS while remaining unchanged at the nominal rate.
- **Tilt FF resonance hardening:** FF is gated by the deadband/error ramp, tilt uses its own lower gain and higher minimum speed, and FF changes are slew-rate limited.
- **Ego-motion source:** ego compensation uses one smooth servo model instead of switching between UART pose deltas and the command observer.
- **HUD target label:** the box label is now a plain `TARGET`; confidence, `LOCKED`, `CENTERED`, and distance suffixes were removed.
- **Legacy cleanup:** removed the v1.2 6-byte telemetry fallback, the unused Webots raw-frame compatibility method, and the RL model archive.
- **Kalman robustness:** `sigma_a_max` reduced to 100, residual threshold raised to 70 px, sigma growth rate-limited, outlier residuals inflate R, reacquisition holds Q at minimum for two frames, and velocity/acceleration are clamped.
- **P FF stop gate:** a sharp target-speed drop now scales FF down to 25% to suppress the one-shot overshoot when a target stops.
- **P pan output damping:** P pan now passes through EMA and slew limiting before the firmware step cap; `p_pan_ema_alpha=0.65`, `p_pan_slew_deg=0.20`.
- **P FF/tilt speed tuning:** pan FF gain reduced to 0.60 and `Kp_pan` raised to 0.11 to trade a little tracking speed for less overshoot; tilt FF gain raised to 0.55 and minimum tilt FF speed lowered to 1.5 deg/s to remove the slow tilt response.
- **Tilt gain:** `Kp_tilt` raised from 0.05 to 0.06 for additional tilt responsiveness after the FF change.
- **RL pan output damping:** RL pan EMA tightened to 0.50 and slew to 0.10 deg per frame for the remaining low-amplitude pan resonance; action limits are 0.80/0.50.
- **Direction-reversal detector:** when the measurement-derived world velocity opposes the filter estimate above 60 px/s, the Kalman resets raw and smoothed velocity to zero for that frame so the old direction is never displayed; the next frame starts from the measured direction.
- **Import cleanup:** removed package/direct-script import fallbacks; the production entry point is `python -m turret_fcs.main`. The optional `pyserial` guard remains.

---

## [v1.3] - 2026-10-21

### Added
- **Single Source of Truth (SSOT) Architecture (`turret_fcs/config.py`):**
  - Consolidated all optical, actuator, controller, kalman, lock, ballistics, and path configurations into a lightweight, pure-math leaf module.
  - Eliminated circular dependencies and heavy imports from config.
  - Added `ActuatorConfig` with exact hardware step limits (0.55 deg pan, 0.22 deg tilt) and servo deadband (0.10 deg).
- **Telemetry v2 Specification & Sidecar Metadata (`turret_fcs/telemetry.py`):**
  - Expanded telemetry schema from 25 columns to 40 columns including `schema_version`, `frame_id`, `latency_ms`, `vx_raw`, `vy_raw`, `innov_px`, `sigma_a`, `kp_eff_pan`, `kp_eff_tilt`, `db_eff_pan`, `db_eff_tilt`, `servo_pan_fb`, `servo_tilt_fb`, `sensor_temp_c`, and `lock_thr`.
  - Automatic JSON metadata sidecar (`.meta.json`) recording system configuration, git commit SHA, session timestamps, and full post-session metrics (FPS, lock ratio, tracking error median/IQR).
  - Tactical event sidecar (`_events.csv`) logging all state transitions, locks, fire authorizations, and mode switches with millisecond timestamps.
- **Redesigned Linear-Quadratic Regulator (LQR) Reward Formulation for RL:**
  - Formulated continuous state-action penalty reward: `r = -(e_norm^2 + lambda_u * u_norm^2 + mu_du * du_norm^2) + bullseye_bonus`.
  - Bayesian hyperparameter optimization with Optuna TPE over 20 fixed seeds using robust Median + IQR scoring.
  - Retrained 250,000 steps PPO policy (`turret_ppo_final.zip`) achieving +59.0% tracking accuracy improvement over baseline P controller in benchmark tests.
- **Automated Pytest Test Suite & Continuous Integration:**
  - Created 19 automated test cases in `tests/` covering SSOT consistency, path resolution, controller deadband behavior, Joseph form Kalman stability, FCSState firmware enum parity, aim-point slew feedforward, and full FCSManager smoke pipelines.
  - Added GitHub Actions workflow (`.github/workflows/ci.yml`) matrix-testing Python 3.10, 3.11, and 3.12.
- **Exact 2D Gaussian Hit Probability (`turret_fcs/lock_manager.py`):**
  - Closed-form Marcum Q / Non-central chi-squared CDF (`scipy.stats.ncx2.cdf`) replacing heuristic attenuation.
  - Dynamic target angular radius computed directly from YOLO bounding box dimensions (`bw, bh`) and camera focal lengths.
  - Explicit separation of projectile flight time (`t_flight = d / v_proj`) and 80 ms transport delay with realistic weapon dispersion (`sigma_weapon = 1.25 deg`).
  - Extended dynamic hysteresis lock window (1.60° - 3.00° entry, 2.20° - 3.60° exit) with 10-frame stability filter.
- **Ballistic Firing Synchronization, Slew Feedforward & Lock Grace Period:**
  - Extended flywheel spin-up to 1.0s (`spinup_time_s = 1.00`), integrated direct aim-point slew feedforward (`u = delta_slew + u_P`) eliminating steady-state ramp lag, and ensuring servo achieves full super-elevation angle before solenoid release.
  - Added 0.75s post-firing lock grace period (`post_fire_grace_duration_frames = 22`), preventing false lock drops as turret returns to boresight.
- **Combat Declutter & Encapsulated Tactical HUD (`turret_fcs/turret_hud.py`):**
  - Encapsulated status banner, footer legend, and event logs into `TurretHUD` methods following SRP.
  - Automatically hides status panels, event logs, and FPS counters during fire authorization and firing cycle.
  - English tactical post-firing engagement feedback (`ENGAGEMENT COMPLETE / ENGAGEMENT SUCCESSFUL`).
- **HUD Color Discipline Decoupling:**
  - Decoupled tactical status banner colors from aiming reticle and target bounding boxes.
  - Reticle, target bounding box, and `TARGET` label remain strictly high-contrast Red during lock and firing sequences, while only the status banner updates through stage colors.

### Changed
- Synchronized Webots digital twin camera FOV (48.8 deg horizontal, 28.6 deg vertical) directly to `CONFIG.optics`.
- Replaced all raw numerical E-stop literals (3, 0) with `FCSState.ESTOP` and `FCSState.TRACKING`.
- Aligned solenoid actuation timing across documentation, config, and firmware to 80 ms (0.08 s).
- Clarified benchmark lock duration metrics: 1,061 frames corresponds to 13.16 seconds at 80.6 FPS.

- **Webots v1.3 Digital Twin Refactoring & High-Fidelity HAL Architecture (`webots/`):**
  - Replaced 8 legacy duplicated controller copies (`webots_fcs_manager`, `webots_pid_controller`, `webots_kalman_filter`, `webots_lock_manager`, `webots_rl_controller`, `webots_patrol_generator`, `webots_target_selector`, `webots_turret_hud`) with a single thin hardware abstraction layer importing production `turret_fcs` modules directly.
  - Implemented high-fidelity plant dynamics in `webots_hardware_interface.py`: 80 ms transport delay queue (`collections.deque(maxlen=3)`), 45 ms 1st-order servo lag (`tau = 0.045 s`), 1.20° mechanical backlash hysteresis, and 0.10° STM32 UART quantization.
  - Aligned physical world (`webots_turret_fcs.wbt`) parameters with v1.3 SSOT: camera FOV 48.8° (0.8517 rad), motor velocity 0.707 rad/s (40.5 deg/s rate limit), and mechanical gimbal limits (Pan ±75°, Tilt ±25°).
  - Implemented plant-accurate anti-windup incremental motor command integration (`cmd = curr + delta`) matching STM32 firmware behavior.
  - Synchronized firing sequence to pure simulation time (`sim_t = robot.getTime()`) with 1.0s spin-up, 80 ms solenoid pulse, 350 ms recover, and 1.5s EVAL window with 0.75s grace period; enforced lock requirement on `[Y]` fire key.
  - Connected 3D virtual referee directly to `KAFA_BLOGU` 3D orientation matrix eliminating coordinate projection ambiguity.
  - Added selectable field-aligned target flight profiles (0°/s static, 2-4°/s slow hover, 5-7°/s standard, 8-11°/s agile) to `balloon_mover.py`.
  - Upgraded Webots telemetry logger to Schema v2 with synchronized ground truth target coordinates and proper `fcs_state` logging.
  - Refactored kinematic mission and controller comparison scripts to test production `FCSManager` with explicit synthetic labeling.

### Fixed
- Fixed broken image references in `README.md` pointing to root instead of `docs/assets/`.
- Deleted duplicate analysis graphic `analysis/system_identification_analysis.png`.
- Updated `CameraThread.read()` to support optional `return_id=True` and zero-latency hardware frame IDs.
- Connected LiDAR temperature reading `temp_c` into real-time telemetry logging instead of discarding.

---

## [v1.2.0] - 2026-10-10

### Added
- Cumulative step accumulator in patrol generator (`min_dispatch_deg = 0.95 deg`) to prevent servo deadband stalling.
- Dynamic Kalman velocity deadband expanding from 45 px/s to 120 px/s based on ego-motion speed and target proximity.
- Tracking confidence hysteresis with dynamic thresholding (0.28 acquisition, 0.16 tracking persistence).
- Asymmetric elevation damping and vertical zero-jerk EMA filter to eliminate stick-slip limit cycles under 300g barrel inertia.
- Smith predictor action memory in PPO environment for sim-to-real latency alignment.
- Consolidated telemetry inspection suite under `analysis/inspect_logs.py`.

---

## [v1.1.0] - 2026-09-20

### Added
- Type-1 Proportional control backbone with soft deadband for chattering-free servo response.
- Range gating protection filtering out background wall LiDAR reflections for errors > 3.0 deg.
- 11-dimensional action history PPO reinforcement learning agent with domain randomization.
- Reverse sliding-window binary UART parser for 115200 baud PC-STM32 telemetry.
- Multi-layer fail-safe architecture (300 ms STM32 heartbeat watchdog, 350 ms camera freeze guard, latched E-stop, 80 ms edge-triggered solenoid latch).
- Lissajous curve parametric air surveillance patrol mode (Pan: +/- 35 deg, Tilt: +/- 8 deg).
