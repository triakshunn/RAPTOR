"""
Compute and verify the Go1 prone (lying flat) joint positions.

Prone = robot body flat on ground, legs fully folded.
This is NOT the standard stand pose. In prone:
  - Hip abduction (roll): 0.0 rad  (legs straight out sideways)
  - Thigh (pitch): max forward/up  ~ 3.14 rad (folded back toward body)  
  - Calf (pitch): max flex         ~ -2.8 rad (folded under thigh)

We verify each candidate by:
  1. Checking all joints are within URDF limits
  2. Computing forward kinematics to see foot positions relative to base
  3. Checking that foot Z is near base Z (legs tucked in)

Joint ordering in Pinocchio (matches URDF active joints):
  Index 0: 1_FR_hip_joint   (abduction, limits: -0.863 to +0.863)
  Index 1: 1_FR_thigh_joint (pitch,      limits: -0.686 to +4.501)
  Index 2: 1_FR_calf_joint  (pitch,      limits: -2.818 to -0.888)
  Index 3: 2_FL_hip_joint
  Index 4: 2_FL_thigh_joint
  Index 5: 2_FL_calf_joint
  Index 6: 3_RR_hip_joint
  Index 7: 3_RR_thigh_joint
  Index 8: 3_RR_calf_joint
  Index 9: 4_RL_hip_joint
  Index 10: 4_RL_thigh_joint
  Index 11: 4_RL_calf_joint
"""

import numpy as np
import pinocchio as pin
import os

# ── Load model ─────────────────────────────────────────────────────────────
URDF_PATH = os.path.join(os.path.dirname(__file__),
                         "../../../Robots/unitree-go1/go1.urdf")
model = pin.buildModelFromUrdf(URDF_PATH)
data  = model.createData()

print(f"Model: nq={model.nq}, nv={model.nv}")
print("\nJoint names (active):")
for i in range(1, model.njoints):  # skip universe
    j = model.joints[i]
    name = model.names[i]
    idx = j.idx_q  # position in q vector
    if model.nqs[i] == 1:  # only revolute/prismatic (skip fixed)
        lo = model.lowerPositionLimit[idx]
        hi = model.upperPositionLimit[idx]
        print(f"  q[{idx:2d}] {name:30s}  limits: [{lo:7.3f}, {hi:7.3f}]")

# ── URDF limits from the file ───────────────────────────────────────────────
# Hip  (abduction): -0.863 to  0.863
# Thigh (pitch):   -0.686 to  4.501
# Calf  (pitch):   -2.818 to -0.888

# ── Pose candidates ────────────────────────────────────────────────────────
poses = {
    "zeros (URDF default)": np.zeros(model.nq),

    # Standard STAND pose (from Unitree SDK high-level defaults)
    "stand": np.array([
        0.0,  0.9, -1.8,   # FR
        0.0,  0.9, -1.8,   # FL
        0.0,  0.9, -1.8,   # RR
        0.0,  0.9, -1.8,   # RL
    ]),

    # PRONE (body flat, legs folded tight):
    # hip=0, thigh pushed forward (high value), calf folded back (negative)
    # Thigh at ~3.5 rad swings leg forward/up; calf at -2.8 tucks foot back
    "prone_v1 (thigh=3.5, calf=-2.8)": np.array([
        0.0,  3.5, -2.8,
        0.0,  3.5, -2.8,
        0.0,  3.5, -2.8,
        0.0,  3.5, -2.8,
    ]),

    # Prone with thigh at max (4.5) and calf at near-max negative (-2.8)
    "prone_v2 (thigh=4.0, calf=-2.818)": np.array([
        0.0,  4.0, -2.818,
        0.0,  4.0, -2.818,
        0.0,  4.0, -2.818,
        0.0,  4.0, -2.818,
    ]),

    # Unitree-reported default prone (from legged_gym / unitree_ros references)
    # hip=0, thigh=1.2, calf=-2.721  → body ~3cm above ground
    "prone_v3 (thigh=1.2, calf=-2.7)": np.array([
        0.0,  1.2, -2.7,
        0.0,  1.2, -2.7,
        0.0,  1.2, -2.7,
        0.0,  1.2, -2.7,
    ]),
    
    # Prone from legged_gym default_dof_pos for Go1 (commonly cited)
    "prone_legged_gym (thigh=0.9, calf=-1.8)": np.array([
        0.0,  0.9, -1.8,  # same as stand — not prone, just stand
        0.0,  0.9, -1.8,
        0.0,  0.9, -1.8,
        0.0,  0.9, -1.8,
    ]),
}

# ── Foot frame names ────────────────────────────────────────────────────────
foot_frames = {
    "FR_foot": "1_FR_foot",
    "FL_foot": "2_FL_foot",
    "RR_foot": "3_RR_foot",
    "RL_foot": "4_RL_foot",
}

def get_frame_id(model, name):
    for i, f in enumerate(model.frames):
        if f.name == name:
            return i
    return None

# Pre-find frame IDs
frame_ids = {}
for key, fname in foot_frames.items():
    fid = get_frame_id(model, fname)
    if fid is None:
        # try partial match
        for i, f in enumerate(model.frames):
            if fname.lower() in f.name.lower():
                fid = i
                break
    frame_ids[key] = fid

print(f"\nFoot frame IDs: {frame_ids}")

# ── Evaluate each pose ─────────────────────────────────────────────────────
print("\n" + "="*80)
for pose_name, q_candidate in poses.items():
    print(f"\n{'─'*60}")
    print(f"POSE: {pose_name}")
    print(f"  q = {np.round(q_candidate, 4)}")

    # Check limits
    lo = model.lowerPositionLimit
    hi = model.upperPositionLimit
    violations = []
    for i in range(model.nq):
        if q_candidate[i] < lo[i] - 1e-4: ### shouldnt this be positive??? (like + 1e-4)
            violations.append(f"q[{i}]={q_candidate[i]:.3f} < lower={lo[i]:.3f}")
        if q_candidate[i] > hi[i] + 1e-4: ### shouldnt this be negative??? (like - 1e-4)
            violations.append(f"q[{i}]={q_candidate[i]:.3f} > upper={hi[i]:.3f}")
    if violations:
        print(f"  ⚠ LIMIT VIOLATIONS:")
        for v in violations:
            print(f"      {v}")
    else:
        print(f"  ✓ All joints within limits")

    # Forward kinematics → foot heights
    pin.forwardKinematics(model, data, q_candidate)
    pin.updateFramePlacements(model, data)

    foot_z = {}
    for key, fid in frame_ids.items():
        if fid is not None:
            pos = data.oMf[fid].translation  ### so fid is 1_FR_foot right? so this is coordinates of feet
            foot_z[key] = pos[2]
        else:
            foot_z[key] = float('nan')

    print(f"  Foot Z positions (relative to world/base origin):")
    for k, z in foot_z.items():
        print(f"      {k}: z = {z:.4f} m")

    avg_foot_z = np.nanmean(list(foot_z.values()))
    print(f"  Mean foot Z: {avg_foot_z:.4f} m  (body at Z=0 → {'legs DOWN' if avg_foot_z < 0 else 'legs UP/FOLDED'})")

print("\n" + "="*80)
print("\nSUMMARY:")
print("For PRONE (body lying flat, legs tucked under):")
print("  The foot Z should be NEAR 0 (at body level) → legs folded to body height.")
print("  For STAND: foot Z should be ~ -0.31 m (total leg length ≈ 0.33 m down).")
print("\nRecommended prone q_nominal for RAPTOR (12 joints):")
q_prone = np.array([0.0, 3.5, -2.818] * 4)
print(f"  q_prone = {q_prone}")
