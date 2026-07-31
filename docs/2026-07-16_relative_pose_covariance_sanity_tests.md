# Relative Pose Covariance Sanity Tests

| check | status |
|---|---|
| Known-noise identity chain | PASS |
| Pose3 covariance ordering | PASS |
| Anchor-strength sweep | PASS |
| Reverse-edge inverse consistency | PASS |

## Known-Noise Chain

Identity-pose chain with independent edge covariance `Q`. For any pair `xi -> xj`, the expected relative covariance is `(j - i) Q`.

| pair | expected trace | actual trace | Frobenius error | max abs error |
|---|---:|---:|---:|---:|
| `x0 -> x8` | 1.131200e+00 | 1.131200e+00 | 1.039422e-15 | 9.992007e-16 |
| `x2 -> x6` | 5.656000e-01 | 5.656000e-01 | 1.825727e-16 | 1.665335e-16 |
| `x4 -> x5` | 1.414000e-01 | 1.414000e-01 | 2.797268e-17 | 2.775558e-17 |

## Ordering Check

This checks that the covariance remains in GTSAM `Pose3` tangent order `[rot_x, rot_y, rot_z, trans_x, trans_y, trans_z]`.

- Expected diagonal: `3.000000e-06 1.200000e-05 2.700000e-05 3.000000e-02 1.200000e-01 2.700000e-01`
- Actual diagonal: `3.000000e-06 1.200000e-05 2.700000e-05 3.000000e-02 1.200000e-01 2.700000e-01`
- Frobenius error: `1.110223e-16`

## Anchor-Strength Sweep

The relative covariance `x0^-1 xN` should stay near the accumulated edge covariance, while the absolute covariance of `xN` grows when the prior on `x0` is weakened.

| prior sigma | rel trace | expected rel trace | abs to trace | rel/abs trace |
|---:|---:|---:|---:|---:|
| 1.000000e-06 | 1.131200e+00 | 1.131200e+00 | 1.131200e+00 | 1.000000e+00 |
| 1.000000e-02 | 1.131200e+00 | 1.131200e+00 | 1.131800e+00 | 9.994699e-01 |
| 1.000000e-01 | 1.131200e+00 | 1.131200e+00 | 1.191200e+00 | 9.496306e-01 |
| 1.000000e+00 | 1.131200e+00 | 1.131200e+00 | 7.131200e+00 | 1.586269e-01 |

## Reverse-Edge Consistency

For the same pair, `xj -> xi` should be the inverse-transform propagation of `xi -> xj`.

- Forward trace: `6.841651e-02`
- Reverse trace: `6.856593e-02`
- Predicted reverse trace: `6.856593e-02`
- Frobenius error: `6.357155e-11`
- Relative error: `6.357155e-11`

`Predicted reverse covariance`:

```text
[6.003037e-04, -4.640759e-06, -4.170586e-06, -1.135547e-07, 4.803908e-05, -6.875436e-05]
[-4.640759e-06, 7.264550e-04, -8.647170e-06, -5.682420e-05, 4.086929e-06, 3.039065e-04]
[-4.170586e-06, -8.647170e-06, 8.632413e-04, 9.623129e-05, -3.605400e-04, -3.973375e-06]
[-1.135547e-07, -5.682420e-05, 9.623129e-05, 1.503498e-02, -2.925791e-04, -2.564372e-04]
[4.803908e-05, 4.086929e-06, -3.605400e-04, -2.925791e-04, 2.181726e-02, -4.926938e-04]
[-6.875436e-05, 3.039065e-04, -3.973375e-06, -2.564372e-04, -4.926938e-04, 2.952370e-02]
```

`Actual reverse covariance`:

```text
[6.003037e-04, -4.640759e-06, -4.170586e-06, -1.135540e-07, 4.803908e-05, -6.875436e-05]
[-4.640759e-06, 7.264550e-04, -8.647169e-06, -5.682420e-05, 4.086929e-06, 3.039065e-04]
[-4.170586e-06, -8.647169e-06, 8.632413e-04, 9.623129e-05, -3.605400e-04, -3.973375e-06]
[-1.135540e-07, -5.682420e-05, 9.623129e-05, 1.503498e-02, -2.925791e-04, -2.564372e-04]
[4.803908e-05, 4.086929e-06, -3.605400e-04, -2.925791e-04, 2.181726e-02, -4.926938e-04]
[-6.875436e-05, 3.039065e-04, -3.973375e-06, -2.564372e-04, -4.926938e-04, 2.952370e-02]
```

