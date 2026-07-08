# Go1 Examples

This folder contains optimization and system identification examples for the Unitree Go1 quadruped.
Python interfaces are provided in the [python/](python/README.md) folder.

## System Identification

Two-stage simulation-to-solver pipeline for identifying friction and inertial parameters of a single Go1 leg (3 DOF, fixed base).

**Physical setup**: trunk is C-clamped to a rigid table (truly fixed base). Each leg is identified independently using its 3-DOF per-leg URDF (`go1_FR/FL/RR/RL.urdf`, `nv=3`). Inactive legs are not in the model and require no torques.

**Timestamp convention**: each tool auto-generates a timestamp (`YYYYMMDD_HHMM`) and prints it to stdout. Downstream tools take explicit file paths as arguments — giving you full control over which run feeds which stage.

---

### Stage 1: Friction Parameter Identification

Identifies per-joint Coulomb friction **Fc**, viscous damping **Fv**, and armature inertia **Ia** (9 parameters total).

The friction model per joint is:

```math
\tau_f(\dot{q}, \ddot{q}) = F_c \cdot \text{sign}(\dot{q}) + F_v \cdot \dot{q} + I_a \cdot \ddot{q}
```

> On real hardware, add an offset term `β` to absorb torque sensor bias. Enable via `include_offset_input = true` in `TestFrictionParametersIdentification.cpp`.

#### Step 1 — Generate trajectory data

All 3 joints excited simultaneously with incommensurate sinusoidal frequencies to maximize regressor rank.

```bash
cd Examples/Unitree_Go1/python
python3 sysid_trajectory_generator.py --mode friction --leg FR
# Prints: "Saved to: ...  [timestamp: 20250707_1430]"
```

| Joint | Center (rad) | Amplitude (rad) | Frequency (Hz) | Phase (rad) |
|-------|-------------|-----------------|----------------|-------------|
| Hip   | 0.0         | 0.3             | 0.5            | 0           |
| Thigh | 1.9         | 0.4             | 1.0            | π/3         |
| Calf  | −1.85       | 0.5             | 1.5            | 2π/3        |

Outputs (`full_params_data/friction/FR/`):
```
q_downsampled_<ts>.csv
q_d_downsampled_<ts>.csv
q_dd_downsampled_<ts>.csv
tau_downsampled_<ts>.csv        ← motor command (not net torque)
```

#### Step 2 — Run the C++ solver

Pass the data timestamp from Step 1. The solver auto-generates its own output timestamp.

```bash
cd /workspaces/RAPTOR/build
./Go1_SysidFriction_test FR 20250707_1430
#                         ^leg ^data_ts from Step 1
```

Outputs (`full_params_data/friction/FR/`):
```
friction_parameters_solution_<ts>.csv   ← [Fc(3), Fv(3), Ia(3)] = 9 values, one per row
friction_estimate_tau_<ts>.csv          ← reconstructed torque (training-set validation)
```

CSV layout:
```
rows 0–2:  Fc  per joint (hip, thigh, calf)
rows 3–5:  Fv  per joint
rows 6–8:  Ia  per joint
```

#### Step 3 — Validate with IDC comparison

```bash
cd Examples/Unitree_Go1/python
python3 sysid_comparison.py --leg FR --mode friction \
    --friction-csv full_params_data/friction/FR/friction_parameters_solution_<ts>.csv
```

Three controllers are simulated on a **new** trajectory (different frequencies from ID data):

| Controller | Friction used | Expected result |
|------------|--------------|-----------------|
| True IDC   | Hardcoded true Fc/Fv/Ia | Best-case baseline |
| Estimated IDC | Loaded from `--friction-csv` | Should match True if solver converged |
| Noisy IDC  | True ± 20% (random signs) | Lower-bound reference |

Repeat Steps 1–2 with `--leg FL`, `--leg RR`, `--leg RL` for each leg.

---

### Stage 2: Inertial Parameter Identification

Identifies the full 30 inertial parameters `[m, mcx, mcy, mcz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]` per link × 3 links. Requires Stage 1 friction results as input.

The Go1 FR leg has **rank 17 out of 30** identifiable parameters — 13 structural zeros arise from the hip x-axis and y-axis joint geometry. Tikhonov ridge regularization automatically pins the unidentifiable directions to URDF values. Identification uses Log-Cholesky reparameterization (Rucker & Wensing 2022) so all optimizer values are physically consistent — no LMI constraints needed.

Two solvers available in `SystemIdentification/ParametersIdentification/`:
- **IIDD** (`TestEndEffectorParametersIdentification`) — requires estimated joint acceleration `q̈`
- **Momentum** (`TestEndEffectorParametersIdentificationMomentum`) — avoids `q̈`, preferred for hardware

#### Step 1 — Design exciting trajectory

Minimizes the condition number of the joint torque regressor. Run once per leg, offline.

```bash
cd /workspaces/RAPTOR/build
./Go1_exciting_traj 1 FR
```

Output (`Examples/Unitree_Go1/SystemIdentification/ExcitingTrajectories/data/FR`):
```
exciting-trajectory-1.csv   ← [time, q_des(3), qd_des(3), tau_ff(3)]
```

#### Step 2 — Generate simulation data

Replays the exciting trajectory in simulation via PD control with true friction active. Acceleration is computed by central difference and filtered.

```bash
cd Examples/Unitree_Go1/python
python3 sysid_trajectory_generator.py --mode inertial --leg FR --run 1
# Prints: "Saved to: ...  [timestamp: 20250707_1445]"
```

Outputs (`full_params_data/inertial/FR/`):
```
traj_data_<ts>.csv       ← [time, q(3), qd(3), tau(3)]
acceleration_<ts>.csv    ← q̈(3), filtered
```

#### Step 3 — Run the C++ solver

Pass the inertial data timestamp and the friction timestamp from Stage 1.

```bash
cd /workspaces/RAPTOR/build
./Go1_SysidInertial_test FR 20250707_1445 20250707_1431
#                         ^leg ^data_ts    ^friction_ts from Stage 1 Step 2
```

Output (`full_params_data/inertial/FR/`):
```
inertial_parameters_solution_<ts>.csv   ← 30 values, comma-delimited, one row
                                           [m₀,mcx₀,…,Izz₀, m₁,…,Izz₁, m₂,…,Izz₂]
                                           (hip, thigh, calf links)
```

#### Step 4 — Validate with IDC comparison

```bash
cd Examples/Unitree_Go1/python

# Pure inertial — all controllers use true friction (isolates inertial quality)
python3 sysid_comparison.py --leg FR --mode inertial \
    --inertial-csv full_params_data/inertial/FR/inertial_parameters_solution_<ts>.csv

# Combined — esti/noise controllers use identified friction too
python3 sysid_comparison.py --leg FR --mode inertial \
    --inertial-csv full_params_data/inertial/FR/inertial_parameters_solution_<ts>.csv \
    --friction-csv full_params_data/friction/FR/friction_parameters_solution_<ts>.csv
```

| Controller | Inertia | Friction (no `--friction-csv`) | Friction (with `--friction-csv`) |
|------------|---------|-------------------------------|----------------------------------|
| True IDC   | URDF + `Ia_true` | `Fc_true` | `Fc_true` |
| Estimated IDC | `phi_estimated` + `Ia_estimated` | `Fc_true` | `Fc_estimated` |
| Noisy IDC  | URDF ± 20% + `Ia_noise` | `Fc_true` | `Fc_noise` (±20%) |

---

### Quick Reference

| Binary | Args | Reads | Writes |
|--------|------|-------|--------|
| `Go1_SysidFriction_test` | `<leg> <data_ts>` | `friction/<leg>/*_<data_ts>.csv` | `friction_parameters_solution_<ts>.csv` |
| `Go1_exciting_traj` | `1 <leg>` | `go1_<leg>.urdf` | `exciting-trajectory-1.csv` |
| `Go1_SysidInertial_test` | `<leg> <data_ts> <friction_ts>` | `inertial/*_<data_ts>.csv` + friction solution | `inertial_parameters_solution_<ts>.csv` |

Full data tree:
```
full_params_data/
    friction/
        FR/                                           ← also FL/ RR/ RL/
            q_downsampled_<ts>.csv
            q_d_downsampled_<ts>.csv
            q_dd_downsampled_<ts>.csv
            tau_downsampled_<ts>.csv
            friction_parameters_solution_<ts>.csv     ← [Fc(3), Fv(3), Ia(3)]
    inertial/
        FR/
            exciting-trajectory-1.csv                 ← fixed (no timestamp)
            traj_data_<ts>.csv
            acceleration_<ts>.csv
            inertial_parameters_solution_<ts>.csv     ← 30 params, comma-delimited
```
