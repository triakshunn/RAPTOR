import argparse
import os
import numpy as np
from scipy.signal import butter, filtfilt, welch
import matplotlib.pyplot as plt

'''
Usage:
python3 data_cleaning.py --mode friction  --leg FR --raw-ts 20260713_165102 --cutoff 25 --order 4
'''

RAW_COLS = {
    "t":   0,
    "q":   slice(1, 4),
    "qd":  slice(4, 7),
    # cols 7:10 = raw "qdd" — DISCARDED. Verified this is a naive forward-diff
    # of qd baked in at logging time, not a sensor value. Never trust it.
    "tau": slice(10, 13),
}


JOINT_NAMES = ["Hip", "Thigh", "Calf"]


def plot_psd_with_candidates(t, signal, joint_names, candidate_cutoffs):
   """
   signal: (N, 3) RAW array (e.g. raw qd or raw tau, before any filtering).
   Plots PSD per joint on log-y with vertical lines at each candidate cutoff.
   Look for where the big low-freq peaks end and the spectrum flattens into
   a noise floor — that's the knee. Pick a cutoff just above it.
   """
   fs = 1.0 / np.diff(t).mean()
   fig, axes = plt.subplots(1, signal.shape[1], figsize=(14, 4), sharey=True)

   for j in range(signal.shape[1]):
       f, pxx = welch(signal[:, j], fs=fs, nperseg=min(1024, len(signal)))
       f, pxx = f[1:], pxx[1:] ## removing DC element
       axes[j].loglog(f, pxx) ### added logcurve for better readability
       for c in candidate_cutoffs:
           axes[j].axvline(c, ls='--', alpha=0.4, label=f'{c} Hz')
       axes[j].set_title(joint_names[j])
       axes[j].set_xlabel("Hz")
       axes[j].grid(True, which='both', alpha=0.2)
       if j == 0:
           axes[j].set_ylabel("PSD")
           axes[j].legend(fontsize=7)

   plt.tight_layout()
   plt.show()

def load_raw(mode, leg, raw_ts, base_dir):
    """Load the 13-col raw hardware log: [t, q(3), qd(3), qdd_raw(3)-discarded, tau(3)]."""
    path = os.path.join(base_dir, "full_params_data", mode, leg, "physical",
                         f"traj_data_{raw_ts}.csv")
    if not os.path.exists(path):
        raise FileNotFoundError(f"Raw physical data not found: {path}")

    raw = np.loadtxt(path)  # space-delimited, matches existing files
    t   = raw[:, RAW_COLS["t"]]
    q   = raw[:, RAW_COLS["q"]]
    qd  = raw[:, RAW_COLS["qd"]]
    tau = raw[:, RAW_COLS["tau"]]
    return t, q, qd, tau


def clean_signals(t, q, qd, tau, cutoff, order):
    """
    q, qd, tau are real (noisy) sensor channels -> Butterworth-filter directly.
    qdd is NOT a sensor channel on Go2 -> derive it from filtered qd via
    4th-order central difference (never differentiate a raw/unfiltered signal).
    """
    dt_raw = np.diff(t).mean()
    fs = 1.0 / dt_raw

    q_f   = butterworth_lowpass_filter(q.copy(),   cutoff, fs, order)
    qd_f  = butterworth_lowpass_filter(qd.copy(),  cutoff, fs, order)
    tau_f = butterworth_lowpass_filter(tau.copy(), cutoff, fs, order)

    # central-diff trims 2 samples off each end -> everything else must match
    qdd_f, dt_acc = central_difference_4th_order(t, qd_f)
    fs_acc = 1.0 / dt_acc

    # differentiation re-injects some high-freq noise even from a filtered
    # input -> one more Butterworth pass on qdd only, same cutoff/order
    qdd_f = butterworth_lowpass_filter(qdd_f, cutoff, fs_acc, order)

    t_out   = t[2:-2]
    q_out   = q_f[2:-2]
    qd_out  = qd_f[2:-2]
    tau_out = tau_f[2:-2]

    return t_out, q_out, qd_out, qdd_f, tau_out


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

def apply_near_zero_mask(t, q, qd, qdd, tau, v_threshold):
    """
    Friction-mode only. Drop samples where ANY joint's |qd| < threshold —
    Coulomb sign(qd) is undefined/chattering right at zero-crossing and
    corrupts the friction fit if left in.
    """
    keep_mask = np.ones(len(qd), dtype=bool)
    for j in range(qd.shape[1]):
        keep_mask &= np.abs(qd[:, j]) >= v_threshold

    return t[keep_mask], q[keep_mask], qd[keep_mask], qdd[keep_mask], tau[keep_mask]
     

def save_outputs(mode, leg, raw_ts, base_dir, t, q, qd, qdd, tau):
    out_dir = os.path.join(base_dir, "full_params_data", mode, leg, "physical")
    out_ts = f"filtered_{raw_ts}"   # -> "..._filtered_20260713_165102.csv"

    if mode == "friction":
        np.savetxt(os.path.join(out_dir, f"q_downsampled_{out_ts}.csv"),    q)
        np.savetxt(os.path.join(out_dir, f"q_d_downsampled_{out_ts}.csv"),  qd)
        np.savetxt(os.path.join(out_dir, f"q_dd_downsampled_{out_ts}.csv"), qdd)
        np.savetxt(os.path.join(out_dir, f"tau_downsampled_{out_ts}.csv"),  tau)
    else:  # inertial
        traj_data = np.column_stack([t, q, qd, tau])       # 10 cols, matches momentum solver's traj_data format
        np.savetxt(os.path.join(out_dir, f"traj_data_{out_ts}.csv"), traj_data)
        np.savetxt(os.path.join(out_dir, f"acceleration_{out_ts}.csv"), qdd)  # kept for symmetry w/ existing split; momentum solver doesn't read this file

    print(f"Saved filtered data to: {out_dir}  [timestamp: {out_ts}]")


def parse_args():
    parser = argparse.ArgumentParser(description="Clean raw Go2 physical SysID data")
    parser.add_argument("--mode", required=True, choices=["friction", "inertial"])
    parser.add_argument("--leg", default="FR", choices=["FR", "FL", "RR", "RL"])
    parser.add_argument("--raw-ts", required=True, help="timestamp of raw traj_data file, e.g. 20260713_165102")
    parser.add_argument("--cutoff", type=float, default=30.0, help="Butterworth cutoff freq (Hz)")
    parser.add_argument("--order", type=int, default=4, help="Butterworth filter order")
    parser.add_argument("--v-threshold", type=float, default=0.01, help="near-zero velocity cutoff (rad/s), friction mode only") # increase to 0.1
    parser.add_argument("--inspect", action="store_true",
                        help="plot PSD of raw qd/tau and exit, instead of running the filter pipeline")
    parser.add_argument("--inspect-cutoffs", type=float, nargs="+", default=[5, 10, 20, 30],
                        help="candidate cutoff freqs (Hz) to overlay on the PSD plot")
    return parser.parse_args()


def main():
    args = parse_args()
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..",
                                              "SystemIdentification", "ParametersIdentification"))

    t, q, qd, tau = load_raw(args.mode, args.leg, args.raw_ts, base_dir)
    print(f"Loaded {len(t)} raw samples for {args.mode}/{args.leg}")

    if args.inspect:
        print("Inspecting raw qd spectrum...")
        plot_psd_with_candidates(t, qd, JOINT_NAMES, args.inspect_cutoffs)
        print("Inspecting raw tau spectrum...")
        plot_psd_with_candidates(t, tau, JOINT_NAMES, args.inspect_cutoffs)
        return

    t, q, qd, qdd, tau = clean_signals(t, q, qd, tau, args.cutoff, args.order)
    print(f"After filtering + central-diff: {len(t)} samples")

    if args.mode == "friction":
        t, q, qd, qdd, tau = apply_near_zero_mask(t, q, qd, qdd, tau, args.v_threshold)
        print(f"After near-zero-velocity mask: {len(t)} samples")

    save_outputs(args.mode, args.leg, args.raw_ts, base_dir, t, q, qd, qdd, tau)

if __name__ == "__main__":
    main()