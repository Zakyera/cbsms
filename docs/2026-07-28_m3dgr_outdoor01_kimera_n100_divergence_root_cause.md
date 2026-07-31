# M3DGR Outdoor01 Kimera N100 Divergence Investigation

Date: 2026-07-28

## Scope

This note investigates the visible Kimera active-window jump near state 219 in:

```text
runs/20260728-140515_m3dgr_outdoor01_cbs_on_dual_factor_graphs_n100_60s_rate025_20260728
```

It also uses the focused nonlinear objective audit:

```text
runs/20260728-143518_m3dgr_outdoor01_cbs_on_n100_divergence_detail_audit_valid_60s_rate025_20260728
```

The detailed audit reached state 184 because factor-level logging slowed the
backend. It therefore establishes the failure mechanism, but does not directly
measure the complete nonlinear objective at state 219.

## State 219 Event

The iSAM2 update moved the 100 existing active poses by:

```text
mean translation: 1.099628 m
max translation:  1.107162 m
mean rotation:     0.259442 deg
max rotation:      0.289964 deg
```

The fitted common rigid translation was:

```text
[0.068994, 1.045779, 0.015556] m
norm: 1.048168 m
```

This common translation explains 95.32 percent of the mean pose movement. The
remaining mean translation after removing the fitted common transform was only
0.051680 m.

The subsequent `marginalizeLeaves()` stage moved no values. The jump happened
inside the accepted iSAM2 update, before marginalization.

The newly created marginal prior had 22 separator keys:

```text
b119, v119, x119, x120, ..., x138
```

There was no collapse in visual factor count. The active local covariance graph
contained:

```text
100 IMU factors
100 bias random-walk factors
1 LinearContainerFactor marginal prior
5206 SmartStereo factors
```

The active optimization also contained 99 G2K factors.

## Why Healthy G2K Did Not Stop the Jump

At update 219, the newest available G2K factor was `x217 -> x218`. There was no
G2K factor involving the new state `x219`, so the external information was one
Kimera keyframe behind.

For `x217 -> x218`:

```text
covariance trace:              1.957415e-4
translation residual:          0.018619 m
rotation residual:             0.003953 rad
NIS:                           7.166316
G2K Hessian Frobenius norm:     2.686056e5
local diagnostic Hessian sum:   2.450267e9
local / G2K diagnostic ratio:   9122
```

The Hessian ratio is a scale diagnostic, not an exact directional stiffness
ratio. More importantly, every active G2K factor is a relative
`BetweenFactor<Pose3>`. A common left transform applied to all active poses
leaves every such measurement unchanged:

```text
T_i' = S T_i
T_j' = S T_j

(T_i')^-1 T_j' = (S T_i)^-1 (S T_j) = T_i^-1 T_j
```

Consequently, G2K provides essentially zero stiffness against a common
translation of the whole active window. IMU relative chains and visual
structure have the same global-translation gauge property. The fixed-lag
marginal prior is the term that anchors the active graph to its previous world
frame.

Therefore, the dominant 1.048 m common-mode component of the state-219 jump was
not observable to the short relative G2K factors. Making those relative factors
stronger would constrain window shape more strongly, but would not directly
anchor the window's global translation.

## Nonlinear Optimizer Evidence

The focused N100 audit evaluated the represented nonlinear factors before and
after accepted updates:

```text
updates audited:                         185
accepted updates increasing exact error: 106
```

Its largest window movement was state 149:

```text
mean movement: 0.102090 m
max movement:  0.138429 m
objective:     1106.99377 -> 1139.79520
change:        +32.80143 (+2.963 percent)
```

Factor-category changes at state 149 were:

```text
bias:          -5.02934
IMU:           +24.19220
marginal prior:-0.21695
G2K between:   -0.39161
SmartStereo:   +14.24712
```

The accepted update slightly improved the G2K objective while worsening IMU
and SmartStereo enough to increase the total objective. This is direct evidence
that the external factor was active but could not prevent an
objective-worsening one-step update.

Earlier parity and shadow audits established:

1. Native GTSAM 4.3 fixed-lag Gauss-Newton reproduces the same class of
   whole-window backward revision with CBS and BPSAM absent.
2. Production BPSAM with CBS disabled also reproduces larger events.
3. Forcing full relinearization and re-elimination produced essentially the
   same critical candidate.
4. Direct marginal-prior covariance preservation tests found no evidence of a
   broken Schur complement or covariance copy.
5. `forceFullSolve` is still one undamped Gauss-Newton step. It is not a
   multi-iteration solve, line search, or objective acceptance test.

The narrow diagnosis is an accepted, objective-worsening one-step nonlinear
Gauss-Newton update in a fixed-lag graph whose absolute gauge is carried by a
linear marginal prior. The evidence does not support calling this a corrupt CBS
marginalization implementation.

## Active Window Versus Published Odometry

The Rerun event is a large reoptimization of old active poses. It is not a
1.1 m instantaneous reversal of the newest published odometry pose.

Around states 214 through 222, the published Kimera `y` coordinate continued
forward:

```text
27.593, 27.729, 27.872, 28.026, 28.189,
28.349, 28.508, 28.664, 28.827 m
```

The aligned ground-truth translation error grew gradually around this period
rather than jumping by 1.1 m in one sample.

## Conclusion

Two related problems are present:

1. Kimera can accept a poor undamped one-step Gauss-Newton candidate that moves
   much of the fixed-lag graph and increases the represented nonlinear
   objective.
2. The current G2K design sends short relative pose factors. Those factors are
   delayed by one Kimera keyframe and are mathematically blind to the dominant
   common translation mode of the active window.

Healthy GLIM therefore could not prevent this failure mode merely by providing
more short consecutive relative factors. Correcting global active-window drift
requires either an objective acceptance/damping policy in Kimera and/or CBS
information that observes a common-frame absolute or longer-baseline mode.

