import numpy as np

tau_meas = np.loadtxt("/workspaces/RAPTOR/Examples/Unitree_Go2/SystemIdentification/ParametersIdentification/full_params_data/friction/FR/tau_downsampled_20260713_165102.csv")           # (N, 3)
tau_est  = np.loadtxt("/workspaces/RAPTOR/Examples/Unitree_Go2/SystemIdentification/ParametersIdentification/full_params_data/friction/FR/friction_estimate_tau_20260714_1742.csv")            # (N, 3)

residual = tau_meas - tau_est
print("std per joint [Nm]:", np.std(residual, axis=0))
print("overall std [Nm]:", np.std(residual))