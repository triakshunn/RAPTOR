# Go2 FR — Friction Candidate Hardware Evaluation

**Metric:** RMS of the feedback torque `tau_fb = kp·(q_ref − q) + kd·(qd_ref − qd)` over
moving-velocity samples (`|qd_ref| ≥ 0.05`), per joint and combined. Each candidate replays
the **identical** reference trajectory; only the feedforward `tau_ff` (the friction/armature
model) differs. **Lower feedback effort ⇒ the model carried more of the true torque ⇒ better.**
Units: N·m. Fixed gains across all runs.

## Results

| Candidate (method)                    | Run   | RMS hip | RMS thigh | RMS calf | **RMS total** |
|---------------------------------------|-------|--------:|----------:|---------:|--------------:|
| 25 Hz low-pass                        | 1621  | **0.4494** | **0.6521** | 0.7972 | 1.1237 |
| 25 Hz low-pass + armature (from URDF) | 1633  | 0.4572  | 0.6668    | **0.7691** | **1.1158** |
| 5 Hz low-pass                         | 1637  | 0.4599  | 0.6821    | 0.7977 | 1.1459 |
| Curve matching                        | 1653  | 0.5300  | 0.6994    | 0.8234 | 1.2034 |
| 25 Hz Low pass sim                    | ----   | 0.007386 |  0.010182 |  0.013019 | 0.018104


Best per column in **bold**.

## Observations

1. **Best overall: 1633** (20 Hz low-pass + URDF armature), total **1.1158** — but only ~0.7%
   ahead of 1621 (1.1237). That gap is within plausible hardware run-to-run noise.

2. **Armature from URDF helps the calf specifically** (0.7972 → 0.7691, ~3.5% better), while
   slightly worsening hip and thigh. This is exactly expected: the calf has the largest gear
   ratio and reflected inertia, so its `Ia·qdd` term is the most significant; hip/thigh armature
   is ~0, so injecting a nonzero URDF value there is a small mismatch.

3. **Heavier filtering hurts:** 5 Hz low-pass (1637) is worse than 20 Hz (1621) on every joint —
   over-smoothing the identification data degrades the fitted parameters.

4. **Curve matching is worst on every joint** — the optimization-based friction sets beat the
   constant-velocity curve-fit approach here.

5. **Calf dominates the residual** across all candidates (highest RMS everywhere). The biggest
   remaining model error lives on the calf regardless of method — consistent with it being the
   hardest joint (largest friction, gear ratio, and armature contribution).

## Caveats

- Total spread across all four candidates is only ~7%; the 1621↔1633 difference (~0.7%) is
  likely inside single-run noise. To declare a winner between the top two, repeat each 2–3×
  and confirm the ranking is stable.
- Each row is a single hardware run.
