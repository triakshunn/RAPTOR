# Go1 Examples

This folder contains optimization and system identification examples for the Unitree Go1 quadruped.
Python interfaces are provided in the [python/](python/README.md) folder.

## System Identification

The system identification pipeline identifies dynamic parameters of a single Go1 leg (3 DOF, fixed base). Two stages run sequentially:

1. **Friction ID** — identifies `Fc` (Coulomb), `Fv` (viscous damping), `Ia` (armature inertia) per joint
2. **Inertial ID** — identifies the 10 inertial parameters per link `[m, mcx, mcy, mcz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]`

**Physical setup**: trunk is C-clamped to a rigid table (truly fixed base). Each leg is identified independently using its 3-DOF per-leg URDF (`go1_FR/FL/RR/RL.urdf`, `nv=3`). Inactive legs are not in the model and require no torques.

---

### Stage 1: Friction Parameter Identification (Simulation)

The friction model per joint is:

```math
\tau_f(\dot{q}, \ddot{q}) = F_c \cdot \text{sign}(\dot{q}) + F_v \cdot \dot{q} + I_a \cdot \ddot{q}
```

> On real hardware, add an offset term `β` to absorb torque sensor bias. Enable via `include_offset_input = true` in `TestFrictionParametersIdentification.cpp`.

#### Step 1 — Generate trajectory data (Python simulation)

All 3 joints of the target leg are excited **simultaneously** with sinusoidal frequencies to maximize friction regressor rank. A PD controller tracks the desired trajectory while friction is added in the simulator.

```bash
cd Examples/Unitree_Go1/python
python3 sysid_trajectory_generator.py --mode friction --leg FR
```

This writes 4 CSV files (one per signal, `N_samples × 3`) to:
```
SystemIdentification/ParametersIdentification/full_params_data/friction/FR/
    q_downsampled.csv
    q_d_downsampled.csv
    q_dd_downsampled.csv
    tau_downsampled.csv
```

**Trajectory design** (simultaneous, all 3 joints):

| Joint | Center (rad) | Amplitude (rad) | Frequency (Hz) | Phase (rad) |
|-------|-------------|-----------------|---------------|-------------|
| Hip   | 0.0         | 0.3             | 0.5           | 0           |
| Thigh | 1.9         | 0.4             | 1.0           | π/3         |
| Calf  | −1.85       | 0.5             | 1.5           | 2π/3        |

Frequencies are incommensurate (no harmonic relationships) to prevent rank deficiency in the regressor.

#### Step 2 — Run the C++ solver

```bash
cd /workspaces/RAPTOR/build
./Go1_SysidFriction_test 1    # "1" matches the *_1.csv file suffix
```

**Output** (`full_params_data/friction/FR/`):
```
friction_parameters_solution_1.csv   ← [Fc(3), Fv(3), Ia(3)] = 9 values
friction_estimate_tau_1.csv          ← reconstructed torque for validation
```

CSV layout:
```
row 0–2:   Fc  per joint (hip, thigh, calf)
row 3–5:   Fv  per joint
row 6–8:   Ia  per joint
```

#### Step 3 — Verify

Check `friction_parameters_solution_1.csv` against the true parameters injected in simulation:

```python
# Expected (from sysid_trajectory_generator.py constants)
FC_TRUE = [0.8, 0.8, 0.8]   # N·m
FV_TRUE = [0.5, 0.5, 0.5]   # N·m·s/rad
IA_TRUE = [0.03, 0.03, 0.03] # kg·m²
```

Run all four legs by repeating Steps 1–2 with `--leg FL`, `--leg RR`, `--leg RL`.

---

### Exciting Trajectories (for Inertial ID)

The [ExcitingTrajectories](SystemIdentification/ExcitingTrajectories/) folder contains `Go1_RegressorExample.cpp`, which designs a trajectory that minimizes the condition number of the joint torque regressor for a single leg. Run once per leg offline:

```bash
./Go1_exciting_traj 1 FR    # outputs data/FR/exciting-trajectory-1.csv
```

---

### Stage 2: Inertial Parameter Identification

> **WIP.** See `knowledge_base.md` §"FEATURE (TODO)" for the full-30-parameter + Tikhonov design.

The Go1 FR leg has **rank 17 out of 30** identifiable parameters (13 structural zeros from hip x-axis and y-axis joint geometry). Unidentifiable parameters are held at URDF values. Identification uses Log-Cholesky reparameterization (Rucker & Wensing 2022) so any optimizer value is physically consistent — no LMI constraints needed.

Two solvers available (both in `SystemIdentification/ParametersIdentification/`):
- **IIDD** (`TestEndEffectorParametersIdentification`) — requires estimated joint acceleration
- **Momentum** (`TestEndEffectorParametersIdentificationMomentum`) — avoids `q̈`, preferred for hardware uncertainty analysis





