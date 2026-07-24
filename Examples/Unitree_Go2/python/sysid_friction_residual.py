#!/usr/bin/env python
"""
sysid_residual_sim.py — sim-side friction torque residual, to compare against the hardware
residual from deploy_reference_traj_friction.py.

Same reference and tau_ff as sysid_eval_trajectory_generator_real.py (imported), same FF+PD
law as the Go2 onboard controller (kp=60, kd=3), simulated through go1_dynamics.integrate
against a known ground-truth plant. Saves tau_fb (feedback effort = model residual) in the
same file layout the hardware deploy writes, so the two are directly comparable.

Run:  python sysid_residual_sim.py
"""
import os
from datetime import datetime
import numpy as np
import pinocchio as pin

from go1_dynamics import integrate
import sysid_eval_trajectory_generator_real as refgen   # build_reference + compute_tau_ff (mirror)

# ============================== CONFIG ==============================
KP, KD = 60.0, 3.0     # match deploy_reference_traj_friction.py
V_MOVE = 0.05          # moving-velocity mask; rank on moving samples (matches hardware scorer)

# SIM plant ground truth — the "true" robot friction. Keep in sync with sysid_trajectory_generator.py.
FC_TRUE     = np.array([0.20, 0.25, 0.35])    # Coulomb  [N·m]
FV_TRUE     = np.array([0.04, 0.05, 0.06])    # viscous  [N·m·s/rad]
IA_TRUE     = np.array([0.015, 0.020, 0.030]) # armature [kg·m²]
OFFSET_TRUE = np.array([0.05, -0.08, 0.10])   # bias     [N·m]

# Identified friction params to evaluate as feedforward (from your sim friction-ID output).
# Default = the ground truth -> "perfect ID" floor residual. REPLACE with your ID solution.
CAND_FC     = np.array([0.20, 0.25, 0.3502])
CAND_FV     = np.array([0.0399, 0.0499, 0.0596])
CAND_IA     = np.array([0.01504, 0.0200, 0.030])
CAND_OFFSET = np.array([0.0499, -0.0799, 0.1000])

OUT_DIR = os.path.abspath(os.path.join(
    refgen.current_dir, 
    f"../SystemIdentification/ParametersIdentification/full_params_data/friction/{refgen.LEG}/sim_residual/"))
# ===================================================================


def make_reference_fn(ts, q, qd, qdd, tau_ff):
    """traj_fn(t) -> (q, qd, qdd, tau_ff); interpolated so integrate() can sample any t."""
    def traj_fn(t):
        tc = np.clip(t, ts[0], ts[-1])
        return (np.array([np.interp(tc, ts, q[:, j])      for j in range(3)]),
                np.array([np.interp(tc, ts, qd[:, j])     for j in range(3)]),
                np.array([np.interp(tc, ts, qdd[:, j])    for j in range(3)]),
                np.array([np.interp(tc, ts, tau_ff[:, j]) for j in range(3)]))
    return traj_fn


def make_ffpd_controller(kp, kd):
    """tau_cmd = tau_ff + kp(q_ref - q) + kd(qd_ref - qd) — the Go2 onboard FF+PD law."""
    Kp, Kd = np.full(3, kp), np.full(3, kd)
    def ctrl_fn(q, v, qd, qd_d, qd_dd, tau_ff):
        return tau_ff + Kp * (qd - q) + Kd * (qd_d - v)
    return ctrl_fn


def central_difference_4th_order(t, velocity):
    dt = np.diff(t).mean()
    n = velocity.shape[0]
    a = np.zeros_like(velocity)
    for i in range(2, n - 2):
        a[i] = (-velocity[i+2] + 8*velocity[i+1] - 8*velocity[i-1] + velocity[i-2]) / (12*dt)
    return a[2:-2], dt


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    run_ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    # 1. reference — identical to the real pipeline
    ts, q, qd, qdd = refgen.build_reference()

    # 2. tau_ff — identical formula to the real pipeline (rnea + tanh friction + Ia + offset),
    #    using the identified candidate. refgen.model keeps armature=0 (Ia added manually).
    refgen.model.armature[:] = 0.0
    tau_ff = refgen.compute_tau_ff(q, qd, qdd, CAND_FC, CAND_FV, CAND_IA, CAND_OFFSET)
    refgen.assert_safe(q, qd, tau_ff)

    # 3. plant — fresh model from the same URDF; true reflected inertia enters as armature
    #    (integrate sets it), Coulomb/viscous/offset applied inside integrate.
    plant = pin.buildModelFromUrdf(refgen.URDF)
    assert plant.nv == 3
    plant.friction[:] = 0.0; plant.damping[:] = 0.0; plant.armature[:] = 0.0

    # 4. simulate FF+PD tracking against the known ground-truth plant
    traj_fn = make_reference_fn(ts, q, qd, qdd, tau_ff)
    ctrl_fn = make_ffpd_controller(KP, KD)
    x0 = np.concatenate([q[0], np.zeros(3)])          # reference starts at rest
    qs, vs, taus = integrate(plant, ts, x0, traj_fn, ctrl_fn,
                             FC_TRUE, FV_TRUE, IA_TRUE, OFFSET_TRUE)

    # 5. residual — the same tau_fb metric the hardware scorer uses
    accs, _ = central_difference_4th_order(ts, vs)
    sl = slice(2, -2)
    t_c, qs_c, vs_c, taus_c = ts[sl], qs[sl], vs[sl], taus[sl]
    q_c, qd_c, qdd_c, tau_ff_c = q[sl], qd[sl], qdd[sl], tau_ff[sl]
    tau_fb = KP * (q_c - qs_c) + KD * (qd_c - vs_c)

    moving   = np.abs(qd_c) >= V_MOVE
    rms_all  = np.sqrt(np.mean(tau_fb ** 2, axis=0))
    rms_move = np.sqrt(np.array([np.mean(tau_fb[moving[:, j], j] ** 2) for j in range(3)]))
    print(f"RMS|tau_fb| per joint (all)    = {rms_all}")
    print(f"RMS|tau_fb| per joint (moving) = {rms_move}   total = {np.linalg.norm(rms_move):.6f}")

    # 6. save — same layout as deploy_reference_traj_friction.py for a one-to-one diff
    tcol = t_c.reshape(-1, 1)
    np.savetxt(f"{OUT_DIR}/traj_data_{run_ts}.csv",
               np.hstack([tcol, qs_c, vs_c, accs, taus_c]))                 # [t, q, qd, qdd, tau]
    np.savetxt(f"{OUT_DIR}/traj_data_{run_ts}_debug.csv",
               np.hstack([tcol, q_c, qd_c, qdd_c, tau_ff_c, tau_fb]))       # [t, q_ref, qd_ref, qdd_ref, tau_ff, tau_fb]
    with open(f"{OUT_DIR}/friction_residual_scores.csv", 'a') as f:
        f.write(f"{run_ts} {rms_move[0]:.6f} {rms_move[1]:.6f} {rms_move[2]:.6f} "
                f"{np.linalg.norm(rms_move):.6f}\n")
    print(f"Saved sim residual to {OUT_DIR}")


if __name__ == "__main__":
    main()