import collections
import numpy as np
import pinocchio as pin
import scipy.io as sio
from scipy.signal import butter, filtfilt
import sys
import os
from pinocchio.visualize import MeshcatVisualizer
import matplotlib.pyplot as plt
import time
import copy
from datetime import datetime


from go1_dynamics import integrate

'''

Possible food for thought: Currently my gains are high enough for the data collection controller , that it is able to follow the trajectory with unknown friction dynamics closely, and I use
that for my optimization proble. What if it cannot by reducing high gains? will that effect? My answer no, since the optimization problem needs tau(data) to be able to track ideal trajectory
Mathematically, the opti problem is independent of gains, but oif gains are low enough to not be able to track the trajectory, exciting trajectory will be bad, and we will not get good results.

'''

sys.path.append("/workspaces/RAPTOR/build/lib")
# import end_effector_sysid_nanobind

# def desired_trajectory(t):
#     """
#     Computes the desired trajectory for a 7-DOF system based on sinusoidal functions.

#     Parameters:
#     t (float): The time variable.

#     Returns:
#     tuple: A tuple containing:
#         - qd (numpy.ndarray): The desired joint positions, a 7-element array where each element is sin(t).
#         - qd_d (numpy.ndarray): The desired joint velocities, a 7-element array where each element is cos(t).
#         - qd_dd (numpy.ndarray): The desired joint accelerations, a 7-element array where each element is -sin(t).
        
#     Notes:
#         This trajectory is not an exciting one, so the results may not be good on hardware.
#     """
#     qd = np.sin(t) * np.ones(7)
#     qd_d = np.cos(t) * np.ones(7)
#     qd_dd = -np.sin(t) * np.ones(7)
#     return qd, qd_d, qd_dd


def verify_trajectory_safety(traj_fn, ctrl_fn, ts, model, margin=0.05):
    """
    Checks if the desired trajectory violates joint position limits at any simulated time step.
    """
    q_min = model.lowerPositionLimit
    q_max = model.upperPositionLimit
    
    # Go1 hardware limits
    TAU_LIMIT_HIP_THIGH = 23.7    # N·m
    TAU_LIMIT_CALF      = 23.7   # N·m
    V_LIMIT             = 30.0    # rad/s

    # Build per-joint torque limit array (12 joints: 4 legs × [hip, thigh, calf])
    tau_limit = np.array([
        TAU_LIMIT_HIP_THIGH, TAU_LIMIT_HIP_THIGH, TAU_LIMIT_CALF,  # FR
        TAU_LIMIT_HIP_THIGH, TAU_LIMIT_HIP_THIGH, TAU_LIMIT_CALF,  # FL
        TAU_LIMIT_HIP_THIGH, TAU_LIMIT_HIP_THIGH, TAU_LIMIT_CALF,  # RR
        TAU_LIMIT_HIP_THIGH, TAU_LIMIT_HIP_THIGH, TAU_LIMIT_CALF,  # RL
    ])
    
    # Sample 1000 points evenly across the simulation duration for efficiency
    ts_sample = np.linspace(ts[0], ts[-1], 1000)
    for t in ts_sample:
        qd, qd_d, qd_dd = traj_fn(t)
        
        # Check position limits
        if np.any(qd < q_min + margin) or np.any(qd > q_max - margin):
            lower_violations = np.where(qd < q_min + margin)[0]
            upper_violations = np.where(qd > q_max - margin)[0]
            
            error_msg = f"Trajectory joint limit violation at t={t:.4f}s!\n"
            if len(lower_violations) > 0:
                error_msg += f"  Lower limit violated on joint(s) {lower_violations}:\n"
                error_msg += f"    qd: {qd[lower_violations]}\n"
                error_msg += f"    limits: {q_min[lower_violations]}\n"
            if len(upper_violations) > 0:
                error_msg += f"  Upper limit violated on joint(s) {upper_violations}:\n"
                error_msg += f"    qd: {qd[upper_violations]}\n"
                error_msg += f"    limits: {q_max[upper_violations]}\n"
            raise ValueError(error_msg)

        # --- Velocity check ---
        if np.any(np.abs(qd_d) > V_LIMIT):
            viol = np.where(np.abs(qd_d) > V_LIMIT)[0]
            raise ValueError(
                f"Trajectory velocity limit violation at t={t:.4f}s!\n"
                f"  Joint(s) {viol}: |qd_d|={np.abs(qd_d[viol])} > {V_LIMIT} rad/s"
            )
    
        # --- Torque check (uses controller evaluated at desired state, i.e. zero tracking error) ---
        if ctrl_fn is not None:
            # Evaluate torque assuming perfect tracking (q=qd, v=qd_d) → pure feedforward torque
            tau = ctrl_fn(qd, qd_d, qd, qd_d, qd_dd)
            if np.any(np.abs(tau) > tau_limit):
                viol = np.where(np.abs(tau) > tau_limit)[0]
                raise ValueError(
                    f"Trajectory torque limit violation at t={t:.4f}s!\n"
                    f"  Joint(s) {viol}: |tau|={np.abs(tau[viol])} > limit={tau_limit[viol]} N·m"
                )
    print("✓ Joint limit safety verification passed.")


def desired_trajectory_leg(t, leg_offset, nq, q_nominal):
    """Excite all 3 joints of one leg simultaneously. Replacement of desired_trajectory_friction."""
    centers    = [0.0,  1.9,   -1.85]
    amplitudes = [0.3,  0.4,    0.5]
    freqs      = [0.5,  1.0,    1.5]       # Hz — incommensurate to avoid rank deficiency
    phases     = [0.0,  np.pi/3, 2*np.pi/3]

    qd    = np.copy(q_nominal)
    qd_d  = np.zeros(nq)
    qd_dd = np.zeros(nq)
    for k in range(3):
        w = 2 * np.pi * freqs[k]
       qd[leg_offset + k]    = centers[k] + amplitudes[k] * np.sin(w * t + phases[k])
        qd_d[leg_offset + k]  = amplitudes[k] * w * np.cos(w * t + phases[k])
        qd_dd[leg_offset + k] = -amplitudes[k] * w**2 * np.sin(w * t + phases[k])
    return qd, qd_d, qd_dd


# def desired_trajectory_friction(t, local_joint_idx):
#     """
#     Generate exciting trajectory for active_joint_idx while keeping others frozen.
#     Returns full vectors of size nq.
#     """
    
#     # Center and amplitude parameters for the Go1 Front Right (FR) leg joints
#     # 0 = Hip, 1 = Thigh, 2 = Calf
#     centers = [0.1, 1.1, -2.5]
#     amplitudes = [0.8, 1.0, 0.2] ## changed from the original generated trajectory
    
#     # Map the active joint to the 3-joint leg configuration
#     c = centers[local_joint_idx]
#     A = amplitudes[local_joint_idx]
    
#     # Multi-frequency sinusoid to excite Coulomb (Fc), Viscous (Fv), and Armature (Ia)
#     qd_joint    = c + A * (0.5*np.sin(0.3*t) + 0.3*np.sin(3.0*t) + 0.1*np.sin(10.0*t))
#     qd_d_joint  = A * (0.5*0.3*np.cos(0.3*t) + 0.3*3.0*np.cos(3.0*t) + 0.1*10.0*np.cos(10.0*t))
#     qd_dd_joint = A * (-0.5*0.09*np.sin(0.3*t) - 0.3*9.0*np.sin(3.0*t) - 0.1*100.0*np.sin(10.0*t))
    
#     return qd_joint, qd_d_joint, qd_dd_joint

# def desired_trajectory_full(t, active_joint_idx, nq, q_nominal):
#     """
#     Computes desired joint positions, velocities, and accelerations for all nq joints.
#     Only the active_joint_idx joint executes the exciting sinusoidal trajectory.
#     All other joints remain frozen at their corresponding values in q_nominal.
#     """
#     qd = np.copy(q_nominal)
#     qd_d = np.zeros(nq)
#     qd_dd = np.zeros(nq)    
    
#     # Get the sinusoidal trajectory for the active joint
#     # (using local_idx = active_joint_idx % 3 to map to Go1 leg joint configs)
#     qd_active, qd_d_active, qd_dd_active = desired_trajectory_friction(t, active_joint_idx % 3)

#     qd[active_joint_idx] = qd_active
#     qd_d[active_joint_idx] = qd_d_active
#     qd_dd[active_joint_idx] = qd_dd_active

#     return qd, qd_d, qd_dd

# def controller(q, v, qd, qd_d, qd_dd):
#     """
#     Computes the control torque for a system using a PD control law.

#     Parameters:
#         q (float or array-like): Current position of the system.
#         v (float or array-like): Current velocity of the system.
#         qd (float or array-like): Desired position of the system.
#         qd_d (float or array-like): Desired velocity of the system.
#         qd_dd (float or array-like): Desired acceleration of the system (not used in this implementation).

#     Returns:
#         float or array-like: Control torque to be applied to the system.
#     """
    
#     kp = 1000
#     kd = 100
#     tau = kp * (qd - q) + kd * (qd_d - v)
#     return tau

# def controller(nv, q, v, qd, qd_d, qd_dd, active_joint_idx):
#     kp = np.ones(nv) * 500.0   # high stiffness for frozen joints
#     kd = np.ones(nv) * 50.0
    
#     kp[active_joint_idx] = 200.0  # moderate for active joint
#     kd[active_joint_idx] = 20.0
    
#     tau = kp * (qd - q) + kd * (qd_d - v)
#     return tau

def controller(nv, q, v, qd, qd_d, qd_dd, active_joint_idx,
               model_ctrl=None, data_ctrl=None,
               Fc_ctrl=None, Fv_ctrl=None):
    """
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

    # Desired acceleration with PD correction
    a_des = qd_dd + Kd * e_d + Kp * e ### what is the logic behind this? why cant just be qd_dd here? 

    # Model-based feedforward: RNEA(q, v, a_des)
    tau_ff = pin.rnea(model_ctrl, data_ctrl, q, v, a_des) ### what is data_ctrl here? this is just needed for the function ### claude says to change this to q,qdes,qddes etc. (still dont get it)

    # Friction feedforward compensation
    tau_fric_comp = Fc_ctrl * np.sign(v) + Fv_ctrl * v ### where is armature included in the model? (Not needed, since it comes in the integrator dynamics)

    return tau_ff + tau_fric_comp ### Plus? In the paper friction comp is on left and Torque on right? No makes sense since this gets subtracted in the integrator dynamics


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
    data_filtered = data.copy()
    
    for i in range(data.shape[1]):
        data_filtered[:, i] = filtfilt(b, a, data[:, i])
    return data_filtered



def compute_tracking_metrics(qs_actual, qd_desired_all, active_joint):
    """
    Compute position tracking error metrics for the active joint only.
    Parameters
    ----------
    qs_actual      : (N, nv)  actual joint positions
    qd_desired_all : (N, nv)  desired joint positions
    active_joint   : int      joint index being evaluated
    Returns
    -------
    dict with keys: 'rmse', 'max_error', 'mean_error', 'error_ts'
    """
    error_ts   = np.abs(qs_actual[:, active_joint] - qd_desired_all[:, active_joint])
    rmse       = np.sqrt(np.mean(error_ts ** 2))
    max_error  = np.max(error_ts)
    mean_error = np.mean(error_ts)
    return {
        'rmse':       rmse,
        'max_error':  max_error,
        'mean_error': mean_error,
        'error_ts':   error_ts,
    }


def print_metrics_table(results):
    """
    Print a formatted summary table of tracking error metrics for all simulated joints.
    Parameters
    ----------
    results : list of dict, each with keys:
        'joint_name', 'joint_idx',
        'true_rmse', 'true_max', 'true_mean',
        'esti_rmse', 'esti_max', 'esti_mean',
        'noise_rmse', 'noise_max', 'noise_mean'
    """
    sep = '─' * 78
    print(f"\n{'═'*78}")
    print(f"  IDC Position Tracking Error — Active Joint Summary")
    print(f"{'═'*78}")
    print(f"  {'Joint':<20} {'Controller':<14} {'RMSE (rad)':<14} {'Max |e| (rad)':<16} {'Mean |e| (rad)'}")
    print(f"  {sep}")
    for r in results:
        jname = f"j{r['joint_idx']} {r['joint_name']}"
        print(f"  {jname:<20} {'True (IDC)':<14} {r['true_rmse']:<14.6f} {r['true_max']:<16.6f} {r['true_mean']:.6f}")
        print(f"  {'':<20} {'Estimated':<14} {r['esti_rmse']:<14.6f} {r['esti_max']:<16.6f} {r['esti_mean']:.6f}")
        print(f"  {'':<20} {'Noisy':<14} {r['noise_rmse']:<14.6f} {r['noise_max']:<16.6f} {r['noise_mean']:.6f}")
        print(f"  {sep}")
    print(f"{'═'*78}\n")

def plot_active_joint_position(ts_clipped, qs_true_clipped, qs_esti_clipped, qs_noise_clipped,
                                qd_des_all, active_joint, joint_label,
                                metrics_true, metrics_esti, metrics_noise):
    """
    Standalone 2-subplot position-tracking figure for a single active joint.
    Colour scheme (distinct, not overlapping):
        True IDC  → crimson       (solid, thick)
        Estimated → dodgerblue    (solid, medium)
        Desired   → darkorange    (dashed)
        Noisy     → black         (solid, medium)

    Bottom subplot shows per-timestep absolute position error for both controllers.
    """
    fig, (ax_pos, ax_err) = plt.subplots(
        2, 1, figsize=(10, 6), sharex=True,
        gridspec_kw={'height_ratios': [3, 1]}
    )
    fig.suptitle(
        f"Position Tracking — {joint_label}  (j{active_joint})",
        fontsize=13, fontweight='bold'
    )
    # Position subplot
    ax_pos.plot(ts_clipped, qd_des_all[:, active_joint],
                color='darkorange', lw=1.5, ls='--', label='Desired', zorder=1)
    ax_pos.plot(ts_clipped, qs_true_clipped[:, active_joint],
                color='crimson', lw=2.0, ls='-',
                label=f'True IDC  (RMSE = {metrics_true["rmse"]:.5f} rad)', zorder=3)
    ax_pos.plot(ts_clipped, qs_esti_clipped[:, active_joint],
                color='dodgerblue', lw=1.5, ls='-',
                label=f'Estimated (RMSE = {metrics_esti["rmse"]:.5f} rad)', zorder=2)
    ax_pos.plot(ts_clipped, qs_noise_clipped[:, active_joint],
                color='black', lw=1.5, ls='-',
                label=f'Noisy     (RMSE = {metrics_noise["rmse"]:.5f} rad)', zorder=2)
    ax_pos.set_ylabel("Position (rad)", fontsize=10)
    ax_pos.legend(fontsize=9, loc='upper right')
    ax_pos.grid(True, alpha=0.35)
    # Error subplot
    ax_err.plot(ts_clipped, metrics_true['error_ts'],
                color='crimson', lw=1.2, ls='-', label='|e| True IDC')
    ax_err.plot(ts_clipped, metrics_esti['error_ts'],
                color='dodgerblue', lw=1.2, ls='-', label='|e| Estimated')
    ax_err.plot(ts_clipped, metrics_noise['error_ts'],
                color='black', lw=1.2, ls='-', label='|e| Noisy')
    ax_err.set_ylabel("|Error| (rad)", fontsize=9)
    ax_err.set_xlabel("Time (s)", fontsize=10)
    ax_err.legend(fontsize=8, loc='upper right')
    ax_err.grid(True, alpha=0.35)
    plt.tight_layout()
    return fig

def make_grid(title, true_data, esti_data, noise_data, desired, ylabel, active_joint, ts_clipped, leg_names, joint_names):
    """4×3 grid comparing true (solid red/blue) vs estimated (dashed) vs noisy (dashed) for all 12 joints."""
    fig, axes = plt.subplots(4, 3, figsize=(14, 10), sharex=True)
    fig.suptitle(title, fontsize=13)
    for leg in range(4):
        for j in range(3):
            jidx = leg * 3 + j
            ax   = axes[leg, j]
            is_active = (jidx == active_joint)
            
            # Desired — darkorange dotted (drawn first, behind others)
            ax.plot(ts_clipped, desired[:, jidx],
                    color='darkorange', lw=1.0, ls=':', label='Desired', zorder=1)
            # True IDC — crimson solid
            ax.plot(ts_clipped, true_data[:, jidx],
                    color='crimson' if is_active else 'firebrick',
                    lw=2.0 if is_active else 1.0, ls='-',
                    label='True IDC', zorder=3)
            # Estimated — dodgerblue dashed
            ax.plot(ts_clipped, esti_data[:, jidx],
                    color='dodgerblue' if is_active else 'steelblue',
                    lw=1.5, ls='--', label='Estimated', zorder=2)
            # Noisy — black solid
            ax.plot(ts_clipped, noise_data[:, jidx],
                    color='black' if is_active else 'dimgray',
                    lw=1.5, ls='-', label='Noisy', zorder=2)

            ax.set_title(
                f"{leg_names[leg]} {joint_names[j]}  (j{jidx})"
                + ("  ← ACTIVE" if is_active else ""),
                fontsize=8,
                fontweight='bold' if is_active else 'normal',
                color='red' if is_active else 'black')
            ax.grid(True, alpha=0.4)
            if j == 0:
                ax.set_ylabel(ylabel, fontsize=8)
            if leg == 3:    
                ax.set_xlabel("Time (s)", fontsize=8)
            if leg == 0 and j == 0:
                ax.legend(fontsize=7)
    plt.tight_layout()
    return fig

def main():
    # initialization for simulation and data collection
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_filename = os.path.abspath(os.path.join(current_dir, "../../../Robots/unitree-go1/go1.urdf"))
    model = pin.buildModelFromUrdf(urdf_filename)
    model_esti = copy.deepcopy(model)      # independent copy — has its own armature field
    model_noise=copy.deepcopy(model)
    # ### added for vis
    # model_vis, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_filename)
    # data_vis = model_vis.createData()

    q_lower = model.lowerPositionLimit   # shape (nq,) = (12,) for full Go1
    q_upper = model.upperPositionLimit   # shape (nq,) = (12,) for full Go1

    if model.nq == 12:
        q_nominal = np.array([0.0, 0.5, -1.0] * 4) ### changed from the trajectories. 
    else:
        # Fallback for Kinova (7 joints) or other models
        q_nominal = np.zeros(model.nq)

   
    q0 = np.copy(q_nominal)  # Set initial state to the nominal pose
    v0 = np.zeros(model.nv)

    dt = 1e-4 # 0.1 ms data measurement loop
    ts_sim = np.arange(0, 5, dt) # 5 seconds simulation

    # Realistic values for Go1 leg joints (adjust to your liking)
    Fc_true = np.zeros(model.nv)
    Fv_true = np.zeros(model.nv)
    Ia_true = np.zeros(model.nv)

    Fc_estimated = np.zeros(model.nv)
    Fv_estimated = np.zeros(model.nv)
    Ia_estimated = np.zeros(model.nv)

    Fc_noise = np.zeros(model.nv)
    Fv_noise = np.zeros(model.nv)
    Ia_noise = np.zeros(model.nv)
    
    # Set non-zero only for the 3 FR leg joints (indices 0, 1, 2 for FR hip, thigh, calf)
    Fc_true[0:3] = [0.5, 0.8, 0.6]   # Coulomb friction (N·m)
    Fv_true[0:3] = [0.3, 0.5, 0.4]   # Viscous damping (N·m·s/rad)
    Ia_true[0:3] = [0.02, 0.03, 0.02] # Armature inertia (kg·m²)

    # # Set non-zero only for the 3 FR leg joints (indices 0, 1, 2 for FR hip, thigh, calf) (Change these to values calculated from the optimization problem)
    # Fc_estimated[0:3] = [0.46, 0.77, 0.55]   # Coulomb friction (N·m)
    # Fv_estimated[0:3] = [0.36, 0.51, 0.47]   # Viscous damping (N·m·s/rad)
    # Ia_estimated[0:3] = [0.02, 0.03, 0.02] # Armature inertia (kg·m²)
    #### Upper are high gains estimated params(200_20 with original (farther) init conditions)


    # # Set non-zero only for the 3 FR leg joints (indices 0, 1, 2 for FR hip, thigh, calf) (Change these to values calculated from the optimization problem)
    # Fc_estimated[0:3] = [0.46, 0.76, 0.55]   # Coulomb friction (N·m)
    # Fv_estimated[0:3] = [0.37, 0.53, 0.48]   # Viscous damping (N·m·s/rad)
    # Ia_estimated[0:3] = [0.02, 0.03, 0.02] # Armature inertia (kg·m²)
    # ### Upper are high gains estimated params (200_20 with closer init positions)

    # # # Set non-zero only for the 3 FR leg joints (indices 0, 1, 2 for FR hip, thigh, calf) (Change these to values calculated from the optimization problem)
    # # Fc_estimated[0:3] = [0.35, 0.67, 0.43]   # Coulomb friction (N·m)
    # # Fv_estimated[0:3] = [0.52, 0.54, 0.65]   # Viscous damping (N·m·s/rad)
    # # Ia_estimated[0:3] = [0.02, 0.03, 0.02] # Armature inertia (kg·m²)
    # # #### Upper are low gains estimated params

    # # Set non-zero only for the 3 FR leg joints (indices 0, 1, 2 for FR hip, thigh, calf) (Change these to values calculated from the optimization problem)
    # Fc_estimated[0:3] = [0.37, 0.62, 0.44]   # Coulomb friction (N·m)
    # Fv_estimated[0:3] = [0.53, 0.63, 0.64]   # Viscous damping (N·m·s/rad)
    # Ia_estimated[0:3] = [0.02, 0.03, 0.02] # Armature inertia (kg·m²)
    # #### Upper are series 60_3 gain results 

    # # Set non-zero only for the 3 FR leg joints (indices 0, 1, 2 for FR hip, thigh, calf) (Change these to values calculated from the optimization problem)
    # Fc_estimated[0:3] = [0.37, 0.62, 0.44]   # Coulomb friction (N·m)
    # Fv_estimated[0:3] = [0.51, 0.63, 0.65]   # Viscous damping (N·m·s/rad)
    # Ia_estimated[0:3] = [0.02, 0.03, 0.02] # Armature inertia (kg·m²)
    # #### Upper are parallel 60_3 gain results (Not a lot of effect then?)

        # Set non-zero only for the 3 FR leg joints (indices 0, 1, 2 for FR hip, thigh, calf) (Change these to values calculated from the optimization problem)
    Fc_estimated[0:3] = [0.50, 0.80, 0.60]   # Coulomb friction (N·m)
    Fv_estimated[0:3] = [0.30, 0.50, 0.40]   # Viscous damping (N·m·s/rad)
    Ia_estimated[0:3] = [0.02, 0.03, 0.02] # Armature inertia (kg·m²)
    #### Upper are parallel 60_3 gain results with filtering near zero vel
    
    ## Noisy friction parameters for result validation (20% noise)
    Fc_noise[0:3] = Fc_true[0:3]+[Fc_true[0]*0.20,-Fc_true[1]*0.2,Fc_true[2]*0.2]   # Coulomb friction (N·m)
    Fv_noise[0:3] = Fv_true[0:3]+[-Fv_true[0]*0.20,-Fv_true[1]*0.20,Fv_true[2]*0.2]   # Viscous damping (N·m·s/rad)
    Ia_noise[0:3] = Ia_true[0:3]+[Ia_true[0]*0.20,Ia_true[1]*0.20,-Ia_true[2]*0.2] # Armature inertia (kg·m²)


    print("Noisy friction parameters:", Fc_noise, Fv_noise, Ia_noise)

    '''
    Noisy friction parameters: [0.6  0.64 0.72 0.   0.   0.   0.   0.   0.   0.   0.   0.  ] 
    [0.24 0.4  0.48 0.   0.   0.   0.   0.   0.   0.   0.   0.  ] 
    [0.024 0.036 0.016 0.    0.    0.    0.    0.    0.    0.    0.    0.   ]
    '''

     # ── Labels ───────────────────────────────────────────────────────────────
    leg_names   = ["FR", "FL", "RR", "RL"]
    joint_names = ["Hip", "Thigh", "Calf"]

     # ── Storage for metrics table and last-run grid data ──────────────────────
    metrics_summary = []
    last_run = {}

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.abspath(os.path.join(current_dir, f"../SystemIdentification/ParametersIdentification/friction_results/run_{timestamp}/")) + "/"
    os.makedirs(output_dir, exist_ok=True)
    
    for active_joint in [0,1,2]:

        leg_idx     = active_joint // 3   # always 0 (FR)
        local_idx   = active_joint % 3
        joint_label = f"{leg_names[leg_idx]} {joint_names[local_idx]}"
        print(f"\n{'─'*60}")
        print(f"  Simulating active_joint = {active_joint}  ({joint_label})")
        print(f"{'─'*60}")
    
    # Wrap the trajectory function using a lambda so it accepts only time 't'
        traj_fn = lambda t: desired_trajectory_full(t, active_joint, model.nq, q_nominal) ### IMP: we are passing active joint as lambda func, so if ever this func called outside the for loop, will take last value of active joint inside

    # Run the safety verification BEFORE simulating
        verify_trajectory_safety(traj_fn, ts_sim, model, margin=0.05)

    # ctrl_fn = lambda q, v, qd, qd_d, qd_dd: controller(model.nv, q, v, qd, qd_d, qd_dd, active_joint)
    
        # True controller — uses true friction params
        data_true = model.createData() ### what is this function?? Used for RNEA calculation
        ctrl_fn_true = lambda q, v, qd, qd_d, qd_dd: controller(
            model.nv, q, v, qd, qd_d, qd_dd, active_joint,
            model_ctrl=model, data_ctrl=data_true, 
            Fc_ctrl=Fc_true, Fv_ctrl=Fv_true)

        # Estimated controller — uses identified friction params
        model_esti.armature = Ia_estimated 
        data_esti = model_esti.createData()
        ctrl_fn_esti = lambda q, v, qd, qd_d, qd_dd: controller(
            model.nv, q, v, qd, qd_d, qd_dd, active_joint,
            model_ctrl=model_esti, data_ctrl=data_esti, 
            Fc_ctrl=Fc_estimated, Fv_ctrl=Fv_estimated)

        # Noisy controller — uses noisy friction params
        model_noise.armature = Ia_noise 
        data_noise = model_noise.createData()
        ctrl_fn_noise = lambda q, v, qd, qd_d, qd_dd: controller(
            model.nv, q, v, qd, qd_d, qd_dd, active_joint,
            model_ctrl=model_noise, data_ctrl=data_noise, 
            Fc_ctrl=Fc_noise, Fv_ctrl=Fv_noise)
    

        # simulate the robot dynamics using ode solver
        # track the desired trajectory using the controller (ground truth)
        qs_true, vs_true, taus_true = integrate(model, ts_sim, np.concatenate([q0, v0]), traj_fn, ctrl_fn_true, active_joint, Fc_true, Fv_true, Ia_true)
        
        ### track the desired trajectory using the controller (estimate)
        qs_esti, vs_esti, taus_esti = integrate(model, ts_sim, np.concatenate([q0, v0]), traj_fn, ctrl_fn_esti, active_joint, Fc_true, Fv_true, Ia_true)

        ### track the desired trajectory using the controller (noisy)
        qs_noise, vs_noise, taus_noise = integrate(model, ts_sim, np.concatenate([q0, v0]), traj_fn, ctrl_fn_noise, active_joint, Fc_true, Fv_true, Ia_true)

        # estimate acceleration using central difference method on velocity data
        accs_true, dt = central_difference_4th_order(ts_sim, vs_true)
        accs_esti, dt = central_difference_4th_order(ts_sim, vs_esti)
        accs_noise, dt = central_difference_4th_order(ts_sim, vs_noise)
        fs = 1 / dt
        print(f"Completed acc cal")
        
        # # filter acceleration data using a Butterworth low-pass filter
        # cutoff = 20 # Hz
        # accs_filtered = butterworth_lowpass_filter(accs, cutoff, fs)
        accs_true_filtered = accs_true # filtering is not needed in simulation, but really important for hardware data
        accs_esti_filtered = accs_esti # filtering is not needed in simulation, but really important for hardware data
        accs_noise_filtered = accs_noise # filtering is not needed in simulation, but really important for hardware data

    
        # # save the simulation results as a text file
        # traj_data = np.concatenate([ts_sim[:,None], qs, vs, taus], axis=1)
        # traj_data_clipped = traj_data[2:-2, :]
        
        
        qs_true_clipped   = qs_true[2:-2]    # trim boundary points lost to central difference
        vs_true_clipped   = vs_true[2:-2]
        taus_true_clipped = taus_true[2:-2]
        qs_esti_clipped   = qs_esti[2:-2]    # trim boundary points lost to central difference
        vs_esti_clipped   = vs_esti[2:-2]
        taus_esti_clipped = taus_esti[2:-2]
        qs_noise_clipped   = qs_noise[2:-2]    # trim boundary points lost to central difference
        vs_noise_clipped   = vs_noise[2:-2]
        taus_noise_clipped = taus_noise[2:-2]

        # print(f"q_s clipped is {qs_clipped}")
        # print(f"v_s clipped is {vs_clipped}")
        # print(f"q_dd_clipped is {accs_filtered}")
        # print(f"tau_clipped is {taus_clipped}")

        np.savetxt(output_dir + f"q_true_{active_joint}.csv",   qs_true_clipped,   delimiter=" ")
        np.savetxt(output_dir + f"q_d_true_{active_joint}.csv",  vs_true_clipped,   delimiter=" ")
        np.savetxt(output_dir + f"q_dd_true_{active_joint}.csv", accs_true_filtered, delimiter=" ")
        np.savetxt(output_dir + f"tau_true_{active_joint}.csv",  taus_true_clipped, delimiter=" ")
        np.savetxt(output_dir + f"q_esti_{active_joint}.csv",   qs_esti_clipped,   delimiter=" ")
        np.savetxt(output_dir + f"q_d_esti_{active_joint}.csv",  vs_esti_clipped,   delimiter=" ")
        np.savetxt(output_dir + f"q_dd_esti_{active_joint}.csv", accs_esti_filtered, delimiter=" ")
        np.savetxt(output_dir + f"tau_esti_{active_joint}.csv",  taus_esti_clipped, delimiter=" ")
        np.savetxt(output_dir + f"q_noise_{active_joint}.csv",   qs_noise_clipped,   delimiter=" ")
        np.savetxt(output_dir + f"q_d_noise_{active_joint}.csv",  vs_noise_clipped,   delimiter=" ")
        np.savetxt(output_dir + f"q_dd_noise_{active_joint}.csv", accs_noise_filtered, delimiter=" ")
        np.savetxt(output_dir + f"tau_noise_{active_joint}.csv",  taus_noise_clipped, delimiter=" ")

        # ── Static sanity check plots: all 12 joints ──────────────────────────
        ts_clipped = ts_sim[2:-2]

        # Pre-compute desired position, velocity, acceleration for all 12 joints
        print("Pre-computing desired trajectories for all joints...")
        qd_des_all  = np.array([traj_fn(t)[0] for t in ts_clipped])   # (N, 12)
        vd_des_all  = np.array([traj_fn(t)[1] for t in ts_clipped])   # (N, 12)
        add_des_all = np.array([traj_fn(t)[2] for t in ts_clipped])   # (N, 12)

        # Tracking error metrics
        metrics_true = compute_tracking_metrics(qs_true_clipped, qd_des_all, active_joint)
        metrics_esti = compute_tracking_metrics(qs_esti_clipped, qd_des_all, active_joint)
        metrics_noise = compute_tracking_metrics(qs_noise_clipped, qd_des_all, active_joint)

        metrics_summary.append({
            'joint_idx':  active_joint,
            'joint_name': joint_label,
            'true_rmse':  metrics_true['rmse'],
            'true_max':   metrics_true['max_error'],
            'true_mean':  metrics_true['mean_error'],
            'esti_rmse':  metrics_esti['rmse'],
            'esti_max':   metrics_esti['max_error'],
            'esti_mean':  metrics_esti['mean_error'],
            'noise_rmse': metrics_noise['rmse'],
            'noise_max':  metrics_noise['max_error'],
            'noise_mean': metrics_noise['mean_error'],
        })

        # Focused single-joint position plot
        plot_active_joint_position(
            ts_clipped, qs_true_clipped, qs_esti_clipped, qs_noise_clipped,
            qd_des_all, active_joint, joint_label,
            metrics_true, metrics_esti, metrics_noise)

        make_grid("Position — True vs Estimated vs Noisy (all 12 joints)",
              qs_true_clipped, qs_esti_clipped, qs_noise_clipped,
              qd_des_all, "rad", active_joint, ts_clipped, leg_names, joint_names)
              


    # ── Print consolidated metrics table ──────────────────────────────────────
    print_metrics_table(metrics_summary)

    plt.show()
    print("Plots complete.")


    # make_grid("Position — Actual vs Desired (all 12 joints)",
    #           qs_clipped, qd_des_all, "rad", active_joint)

    # make_grid("Velocity — Actual vs Desired (all 12 joints)",
    #           vs_clipped, vd_des_all, "rad/s", active_joint)

    # make_grid("Acceleration — Estimated vs Desired (all 12 joints)",
    #           accs_filtered, add_des_all, "rad/s²", active_joint)

    # plt.show()
    # print("Plots complete.")
    

if __name__ == "__main__":
    main()
