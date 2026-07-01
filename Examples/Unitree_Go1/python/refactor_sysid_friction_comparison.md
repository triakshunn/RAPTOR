# Refactor Notes: `sysid_friction_comparison.py`

**File**: `Examples/Unitree_Go1/python/sysid_friction_comparison.py`

Goal: fix bugs and refactor from old per-joint loop to new per-leg simultaneous excitation, matching the updated `sysid_trajectory_generator.py` pipeline.

---

## Bugs (fix first)

### Bug 1 — Hard crash: wrong `verify_trajectory_safety` call (L560)

Function signature (L50): `verify_trajectory_safety(traj_fn, ctrl_fn, ts, model, margin=0.05)`

Called as: `verify_trajectory_safety(traj_fn, ts_sim, model, margin=0.05)` — `ctrl_fn` is missing, `ts_sim` lands on `ctrl_fn`, `model` lands on `ts`, and the 4th positional `model` is absent → **TypeError at runtime**.

Fix: pass `ctrl_fn_true` as second arg (or `None` to skip torque check):
```diff
-        verify_trajectory_safety(traj_fn, ts_sim, model, margin=0.05)
+    verify_trajectory_safety(traj_fn, ctrl_fn_true, ts_sim, model, margin=0.05)
```

### Bug 2 — Shallow copy in `butterworth_lowpass_filter` (L289)

```diff
-    data_filtered = data
+    data_filtered = data.copy()
```
Without `.copy()`, the function modifies the input array in-place and `data_filtered` is just an alias.

---

## Design Issues (refactor)

| # | Line | Issue |
|---|------|-------|
| D1 | L547 | `for active_joint in [0,1,2]` — old one-joint-at-a-time loop; new pipeline excites all 3 simultaneously |
| D2 | L113–153 | `desired_trajectory_friction` + `desired_trajectory_full` — per-joint structure; replace with `desired_trajectory_leg` |
| D3 | L454 | `q_nominal = [0.0, 0.5, -1.0] * 4` — FR centers should be `[0.0, 1.9, -1.85]` to match new trajectory |
| D4 | L541 | `last_run = {}` — declared, never used; delete |
| D5 | L566–585 | Controllers pass single `active_joint` int; should pass `leg_joints = [0,1,2]` (numpy handles list indexing fine) |

---

## Refactor Diffs

### 1. Replace `desired_trajectory_friction` + `desired_trajectory_full` with `desired_trajectory_leg`

```diff
-def desired_trajectory_friction(t, local_joint_idx):
-    centers = [0.1, 1.1, -2.5]
-    amplitudes = [0.8, 1.0, 0.2]
-    c = centers[local_joint_idx]
-    A = amplitudes[local_joint_idx]
-    qd_joint    = c + A * (0.5*np.sin(0.3*t) + 0.3*np.sin(3.0*t) + 0.1*np.sin(10.0*t))
-    qd_d_joint  = A * (0.5*0.3*np.cos(0.3*t) + 0.3*3.0*np.cos(3.0*t) + 0.1*10.0*np.cos(10.0*t))
-    qd_dd_joint = A * (-0.5*0.09*np.sin(0.3*t) - 0.3*9.0*np.sin(3.0*t) - 0.1*100.0*np.sin(10.0*t))
-    return qd_joint, qd_d_joint, qd_dd_joint
-
-def desired_trajectory_full(t, active_joint_idx, nq, q_nominal):
-    qd = np.copy(q_nominal)
-    qd_d = np.zeros(nq)
-    qd_dd = np.zeros(nq)
-    qd_active, qd_d_active, qd_dd_active = desired_trajectory_friction(t, active_joint_idx % 3)
-    qd[active_joint_idx] = qd_active
-    qd_d[active_joint_idx] = qd_d_active
-    qd_dd[active_joint_idx] = qd_dd_active
-    return qd, qd_d, qd_dd
+def desired_trajectory_leg(t, leg_offset, nq, q_nominal):
+    """Excite all 3 joints of one leg simultaneously. Matches sysid_trajectory_generator.py."""
+    centers    = [0.0,  1.9,    -1.85]
+    amplitudes = [0.3,  0.4,     0.5]
+    freqs      = [0.5,  1.0,     1.5]        # Hz — incommensurate to avoid rank deficiency
+    phases     = [0.0,  np.pi/3, 2*np.pi/3]
+
+    qd    = np.copy(q_nominal)
+    qd_d  = np.zeros(nq)
+    qd_dd = np.zeros(nq)
+    for k in range(3):
+        w = 2 * np.pi * freqs[k]
+        qd[leg_offset + k]    = centers[k] + amplitudes[k] * np.sin(w * t + phases[k])
+        qd_d[leg_offset + k]  = amplitudes[k] * w * np.cos(w * t + phases[k])
+        qd_dd[leg_offset + k] = -amplitudes[k] * w**2 * np.sin(w * t + phases[k])
+    return qd, qd_d, qd_dd
```

### 2. Fix `butterworth_lowpass_filter` (L289)

```diff
-    data_filtered = data
+    data_filtered = data.copy()
```

### 3. `main()` — leg config + q_nominal

Replace the `if model.nq == 12` block and `last_run` declaration:

```diff
-    if model.nq == 12:
-        q_nominal = np.array([0.0, 0.5, -1.0] * 4)
-    else:
-        q_nominal = np.zeros(model.nq)
-
-    ...
-
-    last_run = {}
-
+    LEG_NAME   = "FR"
+    LEG_OFFSET = 0   # FR=0, FL=3, RR=6, RL=9
+    leg_joints = list(range(LEG_OFFSET, LEG_OFFSET + 3))
+    # FR joints at trajectory centers; other legs at default stand pose
+    q_nominal = np.array([0.0, 1.9, -1.85,   # FR
+                          0.0, 0.5, -1.0,     # FL
+                          0.0, 0.5, -1.0,     # RR
+                          0.0, 0.5, -1.0])    # RL
```

### 4. `main()` — remove for loop, single simulation block

Replace the `for active_joint in [0,1,2]:` loop with a single block:

```diff
-    for active_joint in [0,1,2]:
-        leg_idx     = active_joint // 3
-        local_idx   = active_joint % 3
-        joint_label = f"{leg_names[leg_idx]} {joint_names[local_idx]}"
-        print(f"\n{'─'*60}")
-        print(f"  Simulating active_joint = {active_joint}  ({joint_label})")
-        print(f"{'─'*60}")
-        traj_fn = lambda t: desired_trajectory_full(t, active_joint, model.nq, q_nominal)
-        verify_trajectory_safety(traj_fn, ts_sim, model, margin=0.05)
-        data_true = model.createData()
-        ctrl_fn_true = lambda q, v, qd, qd_d, qd_dd: controller(
-            model.nv, q, v, qd, qd_d, qd_dd, active_joint,
-            model_ctrl=model, data_ctrl=data_true,
-            Fc_ctrl=Fc_true, Fv_ctrl=Fv_true)
-        model_esti.armature = Ia_estimated
-        data_esti = model_esti.createData()
-        ctrl_fn_esti = lambda q, v, qd, qd_d, qd_dd: controller(
-            model.nv, q, v, qd, qd_d, qd_dd, active_joint,
-            model_ctrl=model_esti, data_ctrl=data_esti,
-            Fc_ctrl=Fc_estimated, Fv_ctrl=Fv_estimated)
-        model_noise.armature = Ia_noise
-        data_noise = model_noise.createData()
-        ctrl_fn_noise = lambda q, v, qd, qd_d, qd_dd: controller(
-            model.nv, q, v, qd, qd_d, qd_dd, active_joint,
-            model_ctrl=model_noise, data_ctrl=data_noise,
-            Fc_ctrl=Fc_noise, Fv_ctrl=Fv_noise)
+    print(f"\n{'─'*60}")
+    print(f"  Simulating leg {LEG_NAME} — all 3 joints simultaneously")
+    print(f"{'─'*60}")
+    traj_fn = lambda t: desired_trajectory_leg(t, LEG_OFFSET, model.nq, q_nominal)
+
+    data_true = model.createData()
+    ctrl_fn_true = lambda q, v, qd, qd_d, qd_dd: controller(
+        model.nv, q, v, qd, qd_d, qd_dd, leg_joints,
+        model_ctrl=model, data_ctrl=data_true,
+        Fc_ctrl=Fc_true, Fv_ctrl=Fv_true)
+    model_esti.armature = Ia_estimated
+    data_esti = model_esti.createData()
+    ctrl_fn_esti = lambda q, v, qd, qd_d, qd_dd: controller(
+        model.nv, q, v, qd, qd_d, qd_dd, leg_joints,
+        model_ctrl=model_esti, data_ctrl=data_esti,
+        Fc_ctrl=Fc_estimated, Fv_ctrl=Fv_estimated)
+    model_noise.armature = Ia_noise
+    data_noise = model_noise.createData()
+    ctrl_fn_noise = lambda q, v, qd, qd_d, qd_dd: controller(
+        model.nv, q, v, qd, qd_d, qd_dd, leg_joints,
+        model_ctrl=model_noise, data_ctrl=data_noise,
+        Fc_ctrl=Fc_noise, Fv_ctrl=Fv_noise)
+
+    verify_trajectory_safety(traj_fn, ctrl_fn_true, ts_sim, model, margin=0.05)
```

### 5. `main()` — metrics loop over all 3 leg joints (replaces single-joint block)

After the `integrate` / `central_difference` / clip block, replace metrics + plotting:

```diff
-        metrics_true  = compute_tracking_metrics(qs_true_clipped,  qd_des_all, active_joint)
-        metrics_esti  = compute_tracking_metrics(qs_esti_clipped,  qd_des_all, active_joint)
-        metrics_noise = compute_tracking_metrics(qs_noise_clipped, qd_des_all, active_joint)
-        metrics_summary.append({
-            'joint_idx':  active_joint,
-            'joint_name': joint_label,
-            ...
-        })
-        plot_active_joint_position(
-            ts_clipped, qs_true_clipped, qs_esti_clipped, qs_noise_clipped,
-            qd_des_all, active_joint, joint_label,
-            metrics_true, metrics_esti, metrics_noise)
+    for k, jidx in enumerate(leg_joints):
+        joint_label = f"{LEG_NAME} {joint_names[k]}"
+        metrics_true  = compute_tracking_metrics(qs_true_clipped,  qd_des_all, jidx)
+        metrics_esti  = compute_tracking_metrics(qs_esti_clipped,  qd_des_all, jidx)
+        metrics_noise = compute_tracking_metrics(qs_noise_clipped, qd_des_all, jidx)
+        metrics_summary.append({
+            'joint_idx':  jidx,
+            'joint_name': joint_label,
+            'true_rmse':  metrics_true['rmse'],  'true_max':  metrics_true['max_error'],  'true_mean':  metrics_true['mean_error'],
+            'esti_rmse':  metrics_esti['rmse'],  'esti_max':  metrics_esti['max_error'],  'esti_mean':  metrics_esti['mean_error'],
+            'noise_rmse': metrics_noise['rmse'], 'noise_max': metrics_noise['max_error'], 'noise_mean': metrics_noise['mean_error'],
+        })
+        plot_active_joint_position(ts_clipped, qs_true_clipped, qs_esti_clipped, qs_noise_clipped,
+                                   qd_des_all, jidx, joint_label,
+                                   metrics_true, metrics_esti, metrics_noise)
```

### 6. `make_grid` — highlight full leg, not single joint

Inside `make_grid`, the `is_active` check:
```diff
-            is_active = (jidx == active_joint)
+            is_active = (jidx in leg_joints)
```
Pass `leg_joints` instead of `active_joint` when calling `make_grid`.

---

## To check before running

- **`go1_dynamics.py` — `integrate(..., active_joint, ...)`**: verify whether `active_joint` is used to apply friction only to that single joint, or if friction is applied from the full `Fc/Fv/Ia` arrays. If the former, the signature needs to change to accept a list. If the latter, no change needed.
