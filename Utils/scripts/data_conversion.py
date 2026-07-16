import numpy as np

ts = "20260713_165102"
friction_dir = "/workspaces/RAPTOR/Examples/Unitree_Go2/SystemIdentification/ParametersIdentification/full_params_data/friction/FR/"
inertial_dir = "/workspaces/RAPTOR/Examples/Unitree_Go2/SystemIdentification/ParametersIdentification/full_params_data/inertial/FR/"

# Load raw 13-col files
D_fr = np.loadtxt(friction_dir + f"traj_data_{ts}.csv")
D_in = np.loadtxt(inertial_dir + f"traj_data_{ts}.csv")

# --- Friction: split into 4 separate files (cols 1-3=q, 4-6=qd, 7-9=qdd, 10-12=tau, 0=time) ---
np.savetxt(friction_dir + f"q_downsampled_{ts}.csv",    D_fr[:, 1:4])
np.savetxt(friction_dir + f"q_d_downsampled_{ts}.csv",  D_fr[:, 4:7])
np.savetxt(friction_dir + f"q_dd_downsampled_{ts}.csv", D_fr[:, 7:10])
np.savetxt(friction_dir + f"tau_downsampled_{ts}.csv",  D_fr[:, 10:13])

# --- Inertial: traj_data = [t, q, qd, tau], acceleration = [qdd] ---
traj = np.hstack([D_in[:, 0:1], D_in[:, 1:4], D_in[:, 4:7], D_in[:, 10:13]])
np.savetxt(inertial_dir + f"traj_data_{ts}.csv",     traj)
np.savetxt(inertial_dir + f"acceleration_{ts}.csv",  D_in[:, 7:10])
