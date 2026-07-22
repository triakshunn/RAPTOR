from datetime import datetime
import pinocchio as pin
import numpy as np
import os

# =========================================================================
# CONFIG
# =========================================================================
LEG   = "FR"
DT     = 2e-3      # 500 Hz -- match low-level API rate
T      = 10.0      # trajectory duration [s]
T_RAMP = 1.5       # raised-cosine fade in/out so motion starts & ends at rest [s]

# joint order: [hip, thigh, calf]. Position limits are READ FROM THE URDF (below).
V_LIMIT   = np.array([30.1, 30.1, 15.70])   # velocity limit  (calf = 15.70)
TAU_LIMIT = np.array([23.7, 23.7, 45.43])   # torque limit    (calf = 45.43, NOT 23.7)

VEL_FRAC   = 0.9       # allow up to this fraction of velocity limit
TAU_FRAC   = 0.9       # allow up to this fraction of torque limit

# friction reference: all 3 joints excited simultaneously, incommensurate freqs
LEG_NEUTRAL = np.array([0.0,  0.9, -1.8])           # safe center posture (VERIFY on rig)
FRIC_AMPS   = np.array([0.6,  0.6,  0.4])           # amplitude per joint [rad]
FRIC_FREQS  = np.array([0.4,  0.55, 0.75])          # frequency per joint [Hz] (incommensurate)
FRIC_PHASES = np.array([0.0,  np.pi/3, 2*np.pi/3])  # phase spread [rad]

V_EPS = 0.05   # <- uncomment + swap np.sign->np.tanh(qd_d/V_EPS) if hardware
               #    chatters at velocity reversals (deviates from the ID model near v=0)

# candidate sets from your sysid: (Fc[3], Fv[3], Ia[3], offset[3])
# REPLACE with your real sysid outputs; first row = sim ground truth (offset unknown -> 0).
CANDIDATES = [
    (np.array([0.3652423773531451, 0.2364152260794431, 1.140607085939648]), np.array([3.575466976656432e-08, 0.08202481232307103,  0.3441476383237332]),
     np.array([-9.03871858798432e-09,0.01127771706432524,-2.435735081752974e-09 ]), np.array([ -0.07685077247093838,0.1539473396331599,  0.04653294071444773]))
    # (Fc, Fv, Ia, offset),
]
#### originally 20Hz low pass filtered accs
# 0.4080525450028091
# 0.2007566802819017
# 1.135346766598562
# 3.675646327116685e-08
# 0.09815763674297992
# 0.3482658755202964
# -5.93741941423508e-09
# 3.172047647014212e-07
# 2.15074008565167e-08
# -0.1556567711708887
# 0.1320802107276932
# 0.04595943862115547

##### above but armature changed manually 
# 0.4080525450028091
# 0.2007566802819017
# 1.135346766598562
# 3.675646327116685e-08
# 0.09815763674297992
# 0.3482658755202964
# 0.004330
# 0.004330
# 0.015911 
# -0.1556567711708887
# 0.1320802107276932
# 0.04595943862115547

#### these are for 5 Hz low pass filter
# 0.410356727277358
# 0.1767314280616502
# 1.117212579904389
# 1.684944248587256e-08
# 0.1132109155749055
# 0.366120012122075
# -7.961861147954e-09
# 0.01375702352930705
# 5.069596726668393e-09
# -0.1549753095084204
# 0.1165642145073084
# 0.02629400980547109

#### these are using curve matching data

# 0.3652423773531451
# 0.2364152260794431
# 1.140607085939648
# 3.575466976656432e-08
# 0.08202481232307103
# 0.3441476383237332
# -9.03871858798432e-09
# 0.01127771706432524
# -2.435735081752974e-09
# -0.07685077247093838
# 0.1539473396331599
# 0.04653294071444773

# =========================================================================
# MODEL  (armature added manually -> leave model.armature = 0, like the C++)
# =========================================================================
current_dir = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.abspath(os.path.join(current_dir, f"../../../Robots/unitree-go2/go2_{LEG}.urdf"))
OUT_DIR = os.path.abspath(os.path.join(
    current_dir, f"../SystemIdentification/ParametersIdentification/full_params_data/friction/{LEG}/physical/"))

model = pin.buildModelFromUrdf(URDF)
data  = model.createData()
assert model.nv == 3, f"Expected nv=3, got {model.nv}"
model.friction[:] = 0.0; model.damping[:] = 0.0; model.armature[:] = 0.0

Q_MIN = model.lowerPositionLimit   # read from URDF (nq=3)
Q_MAX = model.upperPositionLimit
POS_MARGIN = 0.10 * (Q_MAX - Q_MIN)   # stay 10% of each joint's range off both limits
# =========================================================================
# 1. KINEMATIC REFERENCE  (candidate-independent)
# =========================================================================
def raised_cosine(t):
    """Smooth window: 0 at the ends, 1 in the middle. Returns (w, wdot, wddot)."""
    if t < T_RAMP:                       # ramp up
        s = t / T_RAMP
        w   = 0.5 * (1 - np.cos(np.pi * s))
        wd  = 0.5 * (np.pi / T_RAMP)    * np.sin(np.pi * s)
        wdd = 0.5 * (np.pi / T_RAMP)**2 * np.cos(np.pi * s)
    elif t > T - T_RAMP:                 # ramp down (mirror)
        s = (T - t) / T_RAMP
        w   =  0.5 * (1 - np.cos(np.pi * s))
        wd  = -0.5 * (np.pi / T_RAMP)    * np.sin(np.pi * s)
        wdd =  0.5 * (np.pi / T_RAMP)**2 * np.cos(np.pi * s)
    else:                                # flat middle
        w, wd, wdd = 1.0, 0.0, 0.0
    return w, wd, wdd

def build_reference():
    ts = np.linspace(0.0, T, int(round(T / DT)) + 1)   # include t=T so it ends exactly at rest
    q   = np.zeros((len(ts), 3))
    qd  = np.zeros((len(ts), 3))
    qdd = np.zeros((len(ts), 3))
    wf = 2 * np.pi * FRIC_FREQS
    for i, t in enumerate(ts):
        w, wd, wdd = raised_cosine(t)
        u   =  np.sin(wf * t + FRIC_PHASES)
        ud  =  wf    * np.cos(wf * t + FRIC_PHASES)
        udd = -wf**2 * np.sin(wf * t + FRIC_PHASES)
        q[i]   = LEG_NEUTRAL + FRIC_AMPS * (w * u)
        qd[i]  =               FRIC_AMPS * (wd * u + w * ud)          # exact derivative
        qdd[i] =               FRIC_AMPS * (wdd * u + 2 * wd * ud + w * udd)
    return ts, q, qd, qdd

# =========================================================================
# 2. tau_ff  (candidate-dependent) -- matches TestFrictionParametersIdentification.cpp
# =========================================================================
def compute_tau_ff(q, qd, qdd, Fc, Fv, Ia, offset):
    tau = np.zeros_like(q)
    for i in range(len(q)):
        tau_rb   = pin.rnea(model, data, q[i], qd[i], qdd[i])         # inertia+gravity+Coriolis
        tau_fric = ((Fc * np.tanh(qd[i]/V_EPS)) + Fv * qd[i]                  # Coulomb + viscous
                    + Ia * qdd[i] + offset)                          # armature + bias
        tau[i]   = tau_rb + tau_fric
    return tau

# =========================================================================
# SAFETY GATE  (limits read from model, like verify_trajectory_safety)
# =========================================================================
def assert_safe(q, qd, tau):
    assert (q >= Q_MIN + POS_MARGIN).all(), "position below lower limit"
    assert (q <= Q_MAX - POS_MARGIN).all(), "position above upper limit"
    assert (np.abs(qd)  <= VEL_FRAC * V_LIMIT).all(),   "velocity limit exceeded"
    assert (np.abs(tau) <= TAU_FRAC * TAU_LIMIT).all(), "torque limit exceeded"
    assert np.allclose(qd[0],  0.0, atol=1e-6), "does not start at rest"
    assert np.allclose(qd[-1], 0.0, atol=1e-6), "does not end at rest"



# =========================================================================
# 3. GENERATE + WRITE  (one CSV per candidate, shared kinematics)
# =========================================================================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    run_ts = datetime.now().strftime("%Y%m%d_%H%M")
    ts, q, qd, qdd = build_reference()
    for k, (Fc, Fv, Ia, offset) in enumerate(CANDIDATES):
        tau_ff = compute_tau_ff(q, qd, qdd, Fc, Fv, Ia, offset)
        assert_safe(q, qd, tau_ff)
        out  = np.hstack([ts[:, None], q, qd, qdd, tau_ff])          # 13 columns
        path = os.path.join(OUT_DIR, f"reference_{LEG}_cand{k}_{run_ts}.csv")
        np.savetxt(path, out, delimiter=" ")
        print(f"✓ {path}  ({out.shape[0]} rows, "
              f"peak|qd|={np.abs(qd).max(0)}, peak|tau|={np.abs(tau_ff).max(0)})")

if __name__ == "__main__":
    main()
