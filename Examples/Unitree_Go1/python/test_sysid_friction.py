import numpy as np
import pinocchio as pin
import scipy.io as sio
from scipy.signal import butter, filtfilt
import sys
import os

from go1_dynamics import integrate

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

def verify_trajectory_safety(traj_fn, ts, model, margin=0.05):
    """
    Checks if the desired trajectory violates joint position limits at any simulated time step.
    """
    q_min = model.lowerPositionLimit
    q_max = model.upperPositionLimit
    
    # Sample 1000 points evenly across the simulation duration for efficiency
    ts_sample = np.linspace(ts[0], ts[-1], 1000)
    for t in ts_sample:
        qd, _, _ = traj_fn(t)
        
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
            
    print("✓ Joint limit safety verification passed.")



def desired_trajectory_friction(t, local_joint_idx):
    """
    Generate exciting trajectory for active_joint_idx while keeping others frozen.
    Returns full vectors of size nq.
    """
    
    # Center and amplitude parameters for the Go1 Front Right (FR) leg joints
    # 0 = Hip, 1 = Thigh, 2 = Calf
    centers = [0.0, 1.9, -1.85]
    amplitudes = [0.7, 1.5, 0.8]
    
    # Map the active joint to the 3-joint leg configuration
    c = centers[local_joint_idx]
    A = amplitudes[local_joint_idx]
    
    # Multi-frequency sinusoid to excite Coulomb (Fc), Viscous (Fv), and Armature (Ia)
    qd_joint    = c + A * (0.5*np.sin(0.5*t) + 0.3*np.sin(2.0*t) + 0.1*np.sin(7.0*t))
    qd_d_joint  = A * (0.5*0.5*np.cos(0.5*t) + 0.3*2.0*np.cos(2.0*t) + 0.1*7.0*np.cos(7.0*t))
    qd_dd_joint = A * (-0.5*0.25*np.sin(0.5*t) - 0.3*4.0*np.sin(2.0*t) - 0.1*49.0*np.sin(7.0*t))
    
    return qd_joint, qd_d_joint, qd_dd_joint

def desired_trajectory_full(t, active_joint_idx, nq, q_nominal):
    """
    Computes desired joint positions, velocities, and accelerations for all nq joints.
    Only the active_joint_idx joint executes the exciting sinusoidal trajectory.
    All other joints remain frozen at their corresponding values in q_nominal.
    """
    qd = np.copy(q_nominal)
    qd_d = np.zeros(nq)
    qd_dd = np.zeros(nq)    
    
    # Get the sinusoidal trajectory for the active joint
    # (using local_idx = active_joint_idx % 3 to map to Go1 leg joint configs)
    qd_active, qd_d_active, qd_dd_active = desired_trajectory_friction(t, active_joint_idx % 3)

    qd[active_joint_idx] = qd_active
    qd_d[active_joint_idx] = qd_d_active
    qd_dd[active_joint_idx] = qd_dd_active

    return qd, qd_d, qd_dd

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

def controller(nv, q, v, qd, qd_d, qd_dd, active_joint_idx):
    kp = np.ones(nv) * 500.0   # high stiffness for frozen joints
    kd = np.ones(nv) * 50.0
    
    kp[active_joint_idx] = 200.0  # moderate for active joint
    kd[active_joint_idx] = 20.0
    
    tau = kp * (qd - q) + kd * (qd_d - v)
    return tau

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

def main():
    # initialization for simulation and data collection
    urdf_filename = "/Users/akshunn/ROAHM Lab/RAPTOR/Robots/unitree-go1/go1.urdf"
    model = pin.buildModelFromUrdf(urdf_filename)

    q_lower = model.lowerPositionLimit   # shape (nq,) = (12,) for full Go1
    q_upper = model.upperPositionLimit   # shape (nq,) = (12,) for full Go1

    if model.nq == 12:
        q_nominal = np.array([0.0, 0.9, -1.8] * 4)
    else:
        # Fallback for Kinova (7 joints) or other models
        q_nominal = np.zeros(model.nq)

   
    q0 = np.copy(q_nominal)  # Set initial state to the nominal pose
    v0 = np.zeros(model.nv)

    dt = 1e-4 # 0.1 ms data measurement loop
    ts_sim = np.arange(0, 5, dt) # 5 seconds simulation

    # Choose which joint to excite (e.g. 1 for FR hip, 2 for thigh, and 3 for calf)
    active_joint = 1

    # Realistic values for Go1 leg joints (adjust to your liking)
    Fc_true = np.zeros(model.nv)
    Fv_true = np.zeros(model.nv)
    Ia_true = np.zeros(model.nv)
    # Set non-zero only for the 3 FR leg joints (indices 0, 1, 2 for FR hip, thigh, calf)
    Fc_true[0:3] = [0.5, 0.8, 0.6]   # Coulomb friction (N·m)
    Fv_true[0:3] = [0.3, 0.5, 0.4]   # Viscous damping (N·m·s/rad)
    Ia_true[0:3] = [0.02, 0.03, 0.02] # Armature inertia (kg·m²)
    
    # Wrap the trajectory function using a lambda so it accepts only time 't'
    traj_fn = lambda t: desired_trajectory_full(t, active_joint, model.nq, q_nominal)

    ctrl_fn = lambda q, v, qd, qd_d, qd_dd: controller(model.nv, q, v, qd, qd_d, qd_dd, active_joint)
    
    # Run the safety verification BEFORE simulating
    verify_trajectory_safety(traj_fn, ts_sim, model, margin=0.05)
    
    # simulate the robot dynamics using ode solver
    # track the desired trajectory using the controller
    qs, vs, taus = integrate(model, ts_sim, np.concatenate([q0, v0]), traj_fn, ctrl_fn, active_joint, Fc_true, Fv_true, Ia_true)

    
    # estimate acceleration using central difference method on velocity data
    accs, dt = central_difference_4th_order(ts_sim, vs)
    fs = 1 / dt
    print(f"Completed acc cal")
    # # filter acceleration data using a Butterworth low-pass filter
    # cutoff = 20 # Hz
    # accs_filtered = butterworth_lowpass_filter(accs, cutoff, fs)
    accs_filtered = accs # filtering is not needed in simulation, but really important for hardware data
    
    # # save the simulation results as a text file
    # traj_data = np.concatenate([ts_sim[:,None], qs, vs, taus], axis=1)
    # traj_data_clipped = traj_data[2:-2, :]
    
    output_dir = "/Users/akshunn/ROAHM Lab/RAPTOR/Examples/Unitree_Go1/SystemIdentification/ParametersIdentification/friction_data"
    qs_clipped   = qs[2:-2]    # trim boundary points lost to central difference
    vs_clipped   = vs[2:-2]
    taus_clipped = taus[2:-2]
    
    print(f"q_s clipped is {qs_clipped}")
    print(f"v_s clipped is {vs_clipped}")
    print(f"q_dd_clipped is {accs_filtered}")
    print(f"tau_clipped is {taus_clipped}")

    os.makedirs(output_dir, exist_ok=True)
    np.savetxt(output_dir + "q_downsampled_1.csv",   qs_clipped,   delimiter=" ")
    np.savetxt(output_dir + "q_d_downsampled_1.csv",  vs_clipped,   delimiter=" ")
    np.savetxt(output_dir + "q_dd_downsampled_1.csv", accs_filtered, delimiter=" ")
    np.savetxt(output_dir + "tau_downsampled_1.csv",  taus_clipped, delimiter=" ")
    
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

if __name__ == "__main__":
    main()
