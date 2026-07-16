# FR-leg inertial ID comparison: URDF vs Go2-sim vs Go2-real vs Go1-sim

Caveat: the `Go1-sim sol` / `Go1-sim/Go1-GT` columns are from a different robot (Go1), with its
own URDF/groundtruth — its raw solution values are not directly comparable to the `URDF(Go2)`
column. Only the ratio (`Go1-sim/Go1-GT`) is an apples-to-apples measure alongside `sim/URDF`
and `real/URDF`.

## HIP

| param | URDF(Go2) | sim(Go2) | real(Go2) | sim/URDF | real/URDF | Go1-sim sol | Go1-sim/Go1-GT |
|---|---|---|---|---|---|---|---|
| m | 0.678 | 0.87327 | 3.2293 | 1.29 | 4.76 | 0.6923 | 1.17 |
| mcx | -0.0036612 | -0.0034082 | -0.0032259 | 0.931 | 0.881 | -0.00317 | 0.949 |
| mcy | -0.0013153 | -0.13246 | -0.1685 | 101 | 128 | -0.09406 | -18.2 |
| mcz | -7.119e-05 | -0.047322 | -0.66635 | 665 | 9360 | -0.01032 | 172 |
| Ixx | 0.00048256 | 0.022658 | 0.14629 | 47.0 | 303 | 0.01293 | 34.1 |
| Ixy | -4.0927e-06 | -0.00051698 | -0.00016832 | 126 | 41.1 | -0.000431 | -10.7 |
| Iyy | 0.00090378 | 0.0025776 | 0.1375 | 2.85 | 152 | 0.000168 | 0.263 |
| Ixz | 7.2557e-07 | -0.00018469 | -0.00066564 | -255 | -917 | -4.73e-05 | -49.8 |
| Iyz | 1.2819e-06 | -0.0071781 | -0.034768 | -5600 | -27100 | -0.001402 | 1270 |
| Izz | 0.00061832 | 0.020106 | 0.0087948 | 32.5 | 14.2 | 0.01279 | 27.5 |

## THIGH

| param | URDF(Go2) | sim(Go2) | real(Go2) | sim/URDF | real/URDF | Go1-sim sol | Go1-sim/Go1-GT |
|---|---|---|---|---|---|---|---|
| m | 1.152 | 1.2217 | 1.2716 | 1.06 | 1.10 | 0.916 | 0.996 |
| mcx | -0.0043085 | -0.0192 | 0.025759 | 4.46 | -5.98 | 0.005594 | -1.82 |
| mcy | 0.02569 | 0.12075 | 0.065505 | 4.70 | 2.55 | 0.06322 | 3.81 |
| mcz | -0.03767 | -0.028906 | -0.037004 | 0.767 | 0.982 | -0.002936 | 0.0954 |
| Ixx | 0.0076447 | 0.075814 | 0.014216 | 9.92 | 1.86 | 0.04531 | 7.86 |
| Ixy | 8.8791e-06 | 0.012866 | 0.021611 | 1450 | 2430 | 4.22e-05 | -21.1 |
| Iyy | 0.0070479 | 0.0040711 | 0.060692 | 0.578 | 8.61 | 0.03021 | 5.47 |
| Ixz | -0.00042989 | 0.0010366 | 0.0073488 | -2.41 | -17.1 | 0.008926 | -27.8 |
| Iyz | 3.205e-05 | -0.0055767 | -0.00068825 | -174 | -21.5 | 0.014294 | -893 |
| Izz | 0.001619 | 0.076224 | 0.071261 | 47.1 | 44.0 | 0.029514 | 28.1 |

## CALF (URDF = calf+foot lumped)

| param | URDF(Go2) | sim(Go2) | real(Go2) | sim/URDF | real/URDF | Go1-sim sol | Go1-sim/Go1-GT |
|---|---|---|---|---|---|---|---|
| m | 0.194 | 0.065439 | 0.10795 | 0.337 | 0.556 | 0.2109 | 1.08 |
| mcx | 0.00084392 | -0.0065703 | 0.003287 | -7.79 | 3.89 | 0.002573 | 3.06 |
| mcy | 0.00015015 | 0.032574 | -0.019686 | 217 | -131 | -0.0225 | -118 |
| mcz | -0.02623 | -0.013452 | -0.023599 | 0.513 | 0.900 | -0.0283 | 0.988 |
| Ixx | 0.0049412 | 0.019602 | 0.1312 | 3.97 | 26.6 | 0.04553 | 8.03 |
| Ixy | -1.1628e-06 | 0.0033884 | 0.044589 | -2910 | -38300 | 0.009336 | -6670 |
| Iyy | 0.0049656 | 0.0034593 | 0.023411 | 0.697 | 4.71 | 0.0181 | 3.18 |
| Ixz | 0.00011425 | -0.0013669 | -0.0050597 | -12.0 | -44.3 | 0.003573 | 34.0 |
| Iyz | 8.9872e-06 | 0.0067807 | 0.011508 | 754 | 1280 | -0.01416 | -590 |
| Izz | 4.7271e-05 | 0.017508 | 0.14014 | 370 | 2960 | 0.04907 | 1230 |

## Observation

Mass is roughly right across all three runs (0.93-1.3× ground truth), but every off-diagonal
inertia term is wrong by 1-2+ orders of magnitude in all three datasets (Go1-sim, Go2-sim,
Go2-real alike) — including cases where Go1-sim's ratio is worse than Go2's (e.g. Hip mcy:
-18.2× for Go1 vs 101×/128× for Go2). Since Go1-sim is an independent robot/dataset showing the
same qualitative failure (mass fine, off-diagonals blown up), this points to a systematic issue
in the identification pipeline itself rather than something specific to Go2, hardware noise, or
the FF+PD sim change.
