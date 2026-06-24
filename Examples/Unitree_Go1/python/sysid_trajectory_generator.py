import numpy as np
import pinocchio as pin
import scipy.io as sio
from scipy.signal import butter, filtfilt
import sys
import os
from pinocchio.visualize import MeshcatVisualizer
import matplotlib.pyplot as plt
import time
import argparse
import pinocchio as pin

from go1_dynamics import integrate


'''
Go1 torque limits doc: https://pmc.ncbi.nlm.nih.gov/articles/PMC11207842/pdf/sensors-24-03825.pdf
Go1 Kp:Kd torque values taken from here: https://arxiv.org/pdf/2304.09834
'''

# ===== Constants =====
# Per-leg neutral pose [hip, thigh, calf]
LEG_NEUTRAL = np.array([0.0, 1.9, -1.85])

# Friction mode: simultaneous excitation of all 3 joints
# Frequencies chosen to be incommensurate (avoid harmonic relationships)
FRICTION_AMPS   = np.array([0.7, 1.5, 0.8])       # rad — hip, thigh, calf
FRICTION_FREQS  = np.array([0.5, 1.0, 1.5])       # Hz
FRICTION_PHASES = np.array([0.0, np.pi/3, 2*np.pi/3])  # rad — spread phases

DT_SIM = 1e-2   # simulation timestep (10 ms)
T_SIM  = 10.0    # simulation duration (s)

KP_FRICTION, KD_FRICTION   = 60.0, 3.0    # PD gains, friction mode
KP_INERTIAL, KD_INERTIAL   = 60.0, 3.0  # PD gains, inertial mode 

TAU_LIMITS = np.array([23.7, 23.7, 23.7])  # N·m — hip, thigh, calf (35.5 suggested by AI tho)
V_LIMIT    = 30.0                            # rad/s

# Ground-truth friction (simulation only — not used for hardware)
FC_TRUE = np.array([0.5, 0.8, 0.6])
FV_TRUE = np.array([0.3, 0.5, 0.4])
IA_TRUE = np.array([0.02, 0.03, 0.02])

sys.path.append("/workspaces/RAPTOR/build/lib")


def make_friction_traj_fn():
    """
    All 3 joints excited simultaneously with different frequencies and phases.
    Returns a trajectory function traj_fn(t) → (q_d, qd_d, qdd_d).
    """
    def traj_fn(t):
        q_d   = LEG_NEUTRAL.copy()
        qd_d  = np.zeros(3)
        qdd_d = np.zeros(3)

        for j in range(3):
            w = 2 * np.pi * FRICTION_FREQS[j]
            phi = FRICTION_PHASES[j]

            q_d[j]   += FRICTION_AMPS[j] * np.sin(w * t + phi)
            qd_d[j]   += FRICTION_AMPS[j] * w * np.cos(w * t + phi)
            qdd_d[j]  -= FRICTION_AMPS[j] * w**2 * np.sin(w * t + phi)

        return q_d, qd_d, qdd_d

    return traj_fn

def make_inertial_traj_fn(csv_path):
    """
    Load exciting trajectory from CSV (output of Go1_RegressorExample.cpp).
    CSV columns: [t, q0, q1, q2, v0, v1, v2, qdd0, qdd1, qdd2, tau0, tau1, tau2]
    Returns a trajectory function that interpolates the CSV data.
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"Exciting trajectory CSV not found: {csv_path}")

    data  = np.loadtxt(csv_path)
    t_vec = data[:, 0]

    def traj_fn(t):
        # Clip t to valid range
        t_c = np.clip(t, t_vec[0], t_vec[-1])

        # Interpolate position, velocity, acceleration (columns 1-9)
        q_d   = np.array([np.interp(t_c, t_vec, data[:, 1+j]) for j in range(3)])
        qd_d  = np.array([np.interp(t_c, t_vec, data[:, 4+j]) for j in range(3)])
        qdd_d = np.array([np.interp(t_c, t_vec, data[:, 7+j]) for j in range(3)])

        return q_d, qd_d, qdd_d

    return traj_fn

def make_pd_controller(kp,kd):
    Kp = np.full(3, kp) # 3 since number of 3 joints in a leg of Go1
    Kd = np.full(3, kd)

    def ctrl_fn(q, v, qd, qd_d, qd_dd):
        return Kp * (qd - q) + Kd * (qd_d - v)

    return ctrl_fn

def verify_trajectory_safety(traj_fn, model, ctrl_fn):

    """
    Check if the trajectory violates position, velocity, or torque limits before simulation.
    """
    q_min = model.lowerPositionLimit
    q_max = model.upperPositionLimit
    margin = 0.01  # rad safety margin

    # Sample 1000 points across simulation duration
    for t in np.linspace(0, T_SIM, 1000):
        q_d, qd_d, qdd_d = traj_fn(t)

        # Position check
        if np.any(q_d < q_min + margin) or np.any(q_d > q_max - margin):
            viol_lower = np.where(q_d < q_min + margin)[0]
            viol_upper = np.where(q_d > q_max - margin)[0]
            msg = f"Position limit violation at t={t:.3f}s:"
            if len(viol_lower) > 0:
                msg += f" joints {viol_lower} below limit"
            if len(viol_upper) > 0:
                msg += f" joints {viol_upper} above limit"
            raise ValueError(msg)

        # Velocity check
        if np.any(np.abs(qd_d) > V_LIMIT):
            viol = np.where(np.abs(qd_d) > V_LIMIT)[0]
            raise ValueError(f"Velocity limit violation at t={t:.3f}s: joints {viol}")

        # Torque check (assuming perfect tracking: q=q_d, v=q_d_d)
        tau = ctrl_fn(q_d, qd_d, q_d, qd_d, qdd_d)
        if np.any(np.abs(tau) > TAU_LIMITS):
            viol = np.where(np.abs(tau) > TAU_LIMITS)[0]
            raise ValueError(f"Torque limit violation at t={t:.3f}s: joints {viol}, tau={tau[viol]}")

    print("✓ Joint limit safety verification passed.")

def ff_controller(nv, q, v, qd, qd_d, qd_dd, active_joint_idx,
               model_ctrl=None, data_ctrl=None):
    """
    WIP: Not working currently, hopefully will not be needed
    Inverse Dynamics Controller (Computed Torque).
    Uses model-based feedforward + PD feedback + friction compensation.
    If model_ctrl is None, falls back to pure PD.

    Also note this is little different as defined in the paper "System Identification for Constrained Robots", since they find it combined, but we take out the IDC friction part seperately
    """
    e   = qd   - q    # position error
    e_d = qd_d - v    # velocity error

    # PD correction (added on top of feedforward)
    Kp = np.ones(nv) * 40.0 
    Kd = np.ones(nv) * 1.0

    Kp[active_joint_idx] = 20.0
    Kd[active_joint_idx] = 0.5

    if model_ctrl is None:
        # Fallback: pure PD
        return Kp * e + Kd * e_d

    #pin.computeAllTerms(model_ctrl, data_ctrl, q, v)
    # Desired acceleration with PD correction
    a_des = qd_dd + Kd * e_d + Kp * e ## using feedback linearization here

    # Model-based feedforward: RNEA(q, v, a_des)
    tau_ff = pin.rnea(model_ctrl, data_ctrl, q, v, a_des) ### what is data_ctrl here? this is just needed for the function

    return tau_ff 

def central_difference_4th_order(t, velocity):
    """
    Apply a 4th order central difference method to estimate acceleration 
    from velocity data.

    Parameters:
        t (numpy.ndarray): A 1D array of time values. Assumes uniform time steps.
        velocity (numpy.ndarray): A 2D array of velocity values with shape (n, m),
                                  where n is the number of time steps and m is the 
                                  number of velocity components.

    Returns:
        tuple:
            - numpy.ndarray: A 2D array of estimated accelerations with shape 
                             (n-4, m), corresponding to the interior points 
                             where the 4th order central difference is applied.
            - float: The average time step size (dt) computed from the time array.

    Notes:
        - The function assumes uniform time steps in the input time array.
        - Boundary points (first two and last two rows) are excluded from the 
          output as they do not have enough neighbors for a 4th order difference.
        - Boundary accelerations are padded with NaN values internally but are 
          not included in the returned acceleration array.
    """
    dt = np.diff(t).mean()  # Assuming uniform time steps
    n = velocity.shape[0]
    acceleration = np.zeros((n, velocity.shape[1]))

    # Apply 4th order central difference formula for interior points
    for i in range(2, n - 2):
        acceleration[i, :] = (
            -velocity[i + 2, :]
            + 8 * velocity[i + 1, :]
            - 8 * velocity[i - 1, :]
            + velocity[i - 2, :]
        ) / (12 * dt)

    # For boundary points, pad with NaN/0
    acceleration[:2] = np.nan  # Not enough points for 4th order
    acceleration[-2:] = np.nan

    return acceleration[2:-2], dt

def butterworth_lowpass_filter(data, cutoff, fs, order=4):
    """
    Apply a Butterworth low-pass filter to the input data.
    This function applies a digital Butterworth low-pass filter to the given 
    multi-dimensional data. The filter is applied independently to each column 
    of the input data.
    Parameters:
        data (numpy.ndarray): A 2D array where each column represents a signal 
            to be filtered.
        cutoff (float): The cutoff frequency of the low-pass filter in Hz.
        fs (float): The sampling frequency of the input data in Hz.
        order (int, optional): The order of the Butterworth filter. Default is 4.
    Returns:
        numpy.ndarray: The filtered data with the same shape as the input data.
    Notes:
        - The function uses zero-phase filtering via `scipy.signal.filtfilt` 
          to avoid phase distortion.
        - Ensure that the cutoff frequency is less than half the sampling 
          frequency (Nyquist frequency) to avoid aliasing.
    """
    b, a = butter(order, cutoff, btype='low', analog=False, fs = fs)
    data_filtered = data
    
    for i in range(data.shape[1]):
        data_filtered[:, i] = filtfilt(b, a, data[:, i])
    return data_filtered

def make_grid(title, actual, desired, ts_out,leg, ylabel):
    """Helper: 4x3 grid plot for one signal type across all 12 joints."""
    fig, axes = plt.subplots(1, 3, figsize=(14, 10), sharex=True)
    fig.suptitle(title, fontsize=13)
    for j in range(3):
        jidx = j
        ax   = axes[j]
        ax.plot(ts_out, actual[:, jidx],
                color='red',
                lw=2.0 ,
                label='actual')
        ax.plot(ts_out, desired[:, jidx],
                color='orange', lw=1.0, ls='--', label='desired')
        ax.set_title(
            f"{leg} {joint_names[j]}  (j{jidx})"
            + (""),
            fontsize=8,
            fontweight='bold',
            color='red')
        ax.grid(True, alpha=0.4)
        if j == 0:
            ax.set_ylabel(ylabel, fontsize=8)
        ax.set_xlabel("Time (s)", fontsize=8)
        if j == 0:
            ax.legend(fontsize=7)
    plt.tight_layout()
    return fig

def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description='Go1 Per-Leg SysID Trajectory Generator (Friction & Inertial Modes)'
    )
    parser.add_argument(
        '--mode',
        type=str,
        choices=['friction', 'inertial'],
        required=True,
        help='Mode: friction (simultaneous all 3 joints) or inertial (replay exciting traj)'
    )
    parser.add_argument(
        '--leg',
        type=str,
        default='FR',
        choices=['FR', 'FL', 'RR', 'RL'],
        help='Which leg: FR (default), FL, RR, or RL'
    )
    parser.add_argument(
        '--run',
        type=int,
        default=1,
        help='Run ID for exciting-trajectory-<run>.csv (inertial mode only)'
    )
    return parser.parse_args()


def main():
    
    """Main simulation pipeline."""
    args = parse_args()
    leg=args.leg

    print(f"Mode: {args.mode}, Leg: {args.leg}")
    if args.mode == 'inertial':
        print(f"  Using exciting trajectory run {args.run}")


    # ===== Load per-leg URDF (nv=3, fixed base) =====
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.abspath(
        os.path.join(current_dir, f"../../../Robots/unitree-go1/go1_{args.leg}.urdf")
    )

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()

    assert model.nv == 3, f"Expected nv=3, got {model.nv}"
    print(f"✓ Loaded URDF: nv={model.nv}")
    
     # ===== Build trajectory function =====
    if args.mode == 'friction':
        traj_fn = make_friction_traj_fn()
    else:  # inertial
        csv_path = os.path.abspath(
            os.path.join(
                current_dir,
                f"../SystemIdentification/ExcitingTrajectories/data/{args.leg}/exciting-trajectory-{args.run}.csv"
            )
        )
        traj_fn = make_inertial_traj_fn(csv_path)

     # ===== Build controller (all 3 joints always active) =====
    if args.mode == 'friction':
        ctrl_fn = make_pd_controller(KP_FRICTION, KD_FRICTION)
    else:
        ctrl_fn = make_pd_controller(KP_INERTIAL, KD_INERTIAL)

    # ===== Safety check =====
    print("Running safety verification...")
    verify_trajectory_safety(traj_fn, model, ctrl_fn)

     # ===== Simulate =====
    print(f"Simulating {T_SIM}s with dt={DT_SIM}s...")
    q0 = LEG_NEUTRAL.copy()
    v0 = np.zeros(3)
    x0 = np.concatenate([q0, v0])

    ts = np.arange(0, T_SIM, DT_SIM)

    # All 3 joints are active (list [0,1,2])
    qs, vs, taus = integrate(
        model, ts, x0, traj_fn, ctrl_fn,
        [0, 1, 2], FC_TRUE, FV_TRUE, IA_TRUE
    )
    print(f"✓ Simulation complete: {len(ts)} timesteps")

    # ===== Post-process: acceleration + trim =====
    print("Computing acceleration...")
    accs, dt_acc = central_difference_4th_order(ts, vs)

    # Trim 2 samples from each boundary (lost in central difference)
    qs_out   = qs[2:-2]
    vs_out   = vs[2:-2]
    taus_out = taus[2:-2]
    accs_out = accs  # already trimmed by central_difference_4th_order

    print(f"Output shape: {qs_out.shape[0]} samples × {qs_out.shape[1]} DOF")

    # Optional: filter if desired (typically not needed for simulation)
    # fs = 1 / dt_acc
    # accs_out = butterworth_lowpass_filter(accs_out, cutoff=30, fs=fs)

    #### removing near zero velocity values to avoid chattering
    if args.mode == 'friction':
        v_threshold = 0.01
        keep_mask = np.ones(len(vs_out), dtype=bool)
        for j in range(3):
            keep_mask &= np.abs(vs_out[:, j]) >= v_threshold
        qs_out   = qs_out[keep_mask]
        vs_out   = vs_out[keep_mask]
        taus_out = taus_out[keep_mask]
        accs_out = accs_out[keep_mask]
        ts_out   = ts[2:-2][keep_mask]  # also trim time vector

    else:
        ts_out = ts[2:-2]
    # ===== Save CSVs =====
    out_dir = os.path.abspath(
        os.path.join(
            current_dir,
            f"../SystemIdentification/ParametersIdentification/full_params_data/{args.mode}/{args.leg}/"
        )
    )

    os.makedirs(out_dir, exist_ok=True)

    if args.mode == 'inertial':
        np.savetxt(out_dir + f"q_downsampled_{args.run}.csv",   qs_out,   delimiter=" ")
        np.savetxt(out_dir + f"q_d_downsampled_{args.run}.csv",  vs_out,   delimiter=" ")
        np.savetxt(out_dir + f"q_dd_downsampled_{args.run}.csv", accs_out, delimiter=" ")
        np.savetxt(out_dir + f"tau_downsampled_{args.run}.csv",  taus_out, delimiter=" ")
    
    else:
        np.savetxt(os.path.join(out_dir, "q_downsampled.csv"),    qs_out,   delimiter=" ")
        np.savetxt(os.path.join(out_dir, "q_d_downsampled.csv"),  vs_out,   delimiter=" ")
        np.savetxt(os.path.join(out_dir, "q_dd_downsampled.csv"), accs_out, delimiter=" ")
        np.savetxt(os.path.join(out_dir, "tau_downsampled.csv"),  taus_out, delimiter=" ")

    # ── Static sanity check plots: all 12 joints ──────────────────────────

    joint_names = ["Hip", "Thigh", "Calf"]    # 3 joints per leg, columns

    # Pre-compute desired position, velocity, acceleration for all 3 joints
    print("Pre-computing desired trajectories for all joints...")
    qd_des_all  = np.array([traj_fn(t)[0] for t in ts_out])   # (N, 3)
    vd_des_all  = np.array([traj_fn(t)[1] for t in ts_out])   # (N, 3)
    add_des_all = np.array([traj_fn(t)[2] for t in ts_out])   # (N, 3)

    def make_grid(title, actual, desired, ts_out,leg, ylabel):
        """Helper: 4x3 grid plot for one signal type across all 12 joints."""
        fig, axes = plt.subplots(1, 3, figsize=(14, 10), sharex=True)
        fig.suptitle(title, fontsize=13)
        for j in range(3):
            jidx = j
            ax   = axes[j]
            ax.plot(ts_out, actual[:, jidx],
                    color='red',
                    lw=2.0 ,
                    label='actual')
            ax.plot(ts_out, desired[:, jidx],
                    color='orange', lw=1.0, ls='--', label='desired')
            ax.set_title(
                f"{leg} {joint_names[j]}  (j{jidx})"
                + (""),
                fontsize=8,
                fontweight='bold',
                color='red')
            ax.grid(True, alpha=0.4)
            if j == 0:
                ax.set_ylabel(ylabel, fontsize=8)
            ax.set_xlabel("Time (s)", fontsize=8)
            if j == 0:
                ax.legend(fontsize=7)
        plt.tight_layout()
        return fig

    make_grid("Position — Actual vs Desired (all 12 joints)",
              qs_out, qd_des_all,ts_out, leg, "rad")

    make_grid("Velocity — Actual vs Desired (all 12 joints)",
              vs_out, vd_des_all,ts_out, leg, "rad/s")

    make_grid("Acceleration — Estimated vs Desired (all 12 joints)",
              accs_out, add_des_all, ts_out, leg, "rad/s²")

    plt.show()
    print("Plots complete.")
    
    
if __name__ == "__main__":
    main()


######### MESHCAT Implementation ##################
    # # Start Meshcat viewer
    # viz = MeshcatVisualizer(model_vis, collision_model, visual_model)
    # viz.initViewer(open=True)   # opens browser tab automatically
    # viz.loadViewerModel()

    # # Set up live matplotlib figure
    # plt.ion()  # interactive mode — allows live updates
    # fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=False)
    # fig.suptitle(f"Live Replay — Joint {active_joint}", fontsize=13)

    # # Pre-compute accs on the full ts_sim grid for indexing
    # # 5x speed math:
    # # 5s simulation @ 10kHz = 50,000 steps. 
    # # To play in 1s (5x speed) at a 50Hz refresh rate:
    # # We update every 1000 steps and pause for 0.02s (20ms) between frames.
    # playback_speed = 1.0
    # hz = 50.0  # target display update rate
    # decimate = int((1.0 / dt) / hz * playback_speed)  # = 200 steps
    # dt_pause = 1.0 / hz  # = 0.02 seconds

    # # Pre-align time vector for clipped data
    # ts_clipped = ts_sim[2:-2]
    
    # # Pre-compute desired trajectory vectors to avoid slow list comprehensions in the loop
    # print("Pre-computing desired trajectory for plotting...")
    # qd_des = np.array([traj_fn(t)[0][active_joint] for t in ts_clipped])

    # # Setup plots
    # plt.ion()
    # fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    # fig.suptitle(f"Live Replay — Joint {active_joint} (5x Speed)", fontsize=13)
    
    # # Pre-initialize line handles for fast plotting
    # line_q_act, = axes[0].plot([], [], color='blue', label='q actual')
    # line_q_des, = axes[0].plot([], [], color='orange', linestyle='--', label='q desired')
    # axes[0].set_ylabel("Position (rad)")
    # axes[0].legend(loc='upper right', fontsize=8)
    # axes[0].grid(True)
    
    # line_v_act, = axes[1].plot([], [], color='green', label='velocity')
    # axes[1].set_ylabel("Velocity (rad/s)")
    # axes[1].legend(loc='upper right', fontsize=8)
    # axes[1].grid(True)
    
    # line_a_act, = axes[2].plot([], [], color='red', label='acceleration')
    # axes[2].set_ylabel("Acceleration (rad/s²)")
    # axes[2].set_xlabel("Time (s)")
    # axes[2].legend(loc='upper right', fontsize=8)
    # axes[2].grid(True)
    
    # # Pre-set limits to avoid auto-scaling compute time
    # axes[0].set_xlim(ts_clipped[0], ts_clipped[-1])
    # axes[0].set_ylim(np.min(qs_clipped[:, active_joint]) - 0.1, np.max(qs_clipped[:, active_joint]) + 0.1)
    # axes[1].set_ylim(np.min(vs_clipped[:, active_joint]) - 0.5, np.max(vs_clipped[:, active_joint]) + 0.5)
    # axes[2].set_ylim(np.min(accs_filtered[:, active_joint]) - 5.0, np.max(accs_filtered[:, active_joint]) + 5.0)

    # print("Replaying simulation in Meshcat + live plots...")
    # for i in range(0, len(qs_clipped), decimate):
    #     # 1. Update Meshcat
    #     # viz.display(qs_clipped[i])
        
    #     # 2. Update plot lines efficiently (no cla() called)
    #     t_now = ts_clipped[:i+1]
    #     line_q_act.set_data(t_now, qs_clipped[:i+1, active_joint])
    #     line_q_des.set_data(t_now, qd_des[:i+1])
        
    #     line_v_act.set_data(t_now, vs_clipped[:i+1, active_joint])
        
    #     line_a_act.set_data(t_now, accs_filtered[:i+1, active_joint])
        
    #     # 3. Draw and pause
    #     fig.canvas.draw()
    #     fig.canvas.flush_events()
    #     plt.pause(dt_pause)

    # plt.ioff()
    # plt.show()
    # print("Replay complete.")

    
    # # initialization for system identification
    
    # # motor friction parameters
    # # [ --- static friction                 ---
    # #   --- viscous friction (damping)      ---
    # #   --- transmission inertia (armature) ---
    # #   --- offset                          ---]
    # # in this particular example, motor friction parameters are disabled (set to zero)
    # friction_parameters = np.zeros(28)

    # sysid_solver = end_effector_sysid_nanobind.EndEffectorIdentificationPybindWrapper(
    #     urdf_filename, 
    #     friction_parameters,
    #     'Second',            # format of time data, 'Second' or 'Nanosecond'
    #     True                 # display information or not
    # )
    
    # sysid_solver.add_trajectory_file(
    #     "traj_data.csv",
    #     "acceleration_filtered.csv")

    # sysid_solver.set_ipopt_parameters(
    #     1e-14,          # tol
    #     5.0,            # max_wall_time
    #     5,              # print_level
    #     500,            # max_iter
    #     "adaptive",     # mu_strategy
    #     "ma86",         # linear_solver
    #     False           # gradient_check
    # )

    # theta_solution = sysid_solver.optimize()
    
    # print("end effector inertial parameters:")
    # print("solution:\n", theta_solution)
    # print("groundtruth:\n", model.inertias[-1].toDynamicParameters())
    ##############################################
