# Stage 2C-D1: GEODE Short/Medium evaluator sanity and Tunnel4 reference screening

Date: 2026-08-12

Scope: passive offline audit only

Final evaluator decision: **A — EVALUATOR VERIFIED**

## 1. Executive result

The existing Short and Medium edge evaluator is consistent with the official
GEODE pose convention, Gamma-to-GT conversion, edge direction, and documented
zero-offset timing workflow. The unexpectedly large Medium 0.20-second
translation residual is genuine for the recorded local-only GLIM result; it is
not explained by an endpoint inversion, a missed Gamma/Beta lever arm, or a
supported constant clock correction.

The strongest evidence is:

- The local absolute-pose conversion and the official
  `gamma2GT_gnss.py` formula agree below `6.54e-13` in SE(3) before the
  official script's six-decimal text quantization.
- The official script's text output and the local converted trajectory differ
  by at most `8.59e-7 m` and `1.84e-6 rad`, which is consistent with the
  official script printing six decimal places.
- The documented `evo_ape`/`evo_rpe` workflow and an independently coded SE(3)
  implementation agree to `9.83e-12` or better.
- At Medium's 0.20-second horizon, GLIM and GT have almost equal translation
  magnitudes (`0.4395 m` and `0.4400 m` medians) but generally opposing local
  translation directions (median cosine `-0.9294`).
- The official full-trajectory consecutive-frame RPE translation median is
  `0.4119 m`; the audited two-frame/approximately-0.20-second edge residual is
  `0.8573 m`. The roughly doubled horizon produces a consistent scale.
- The released lever-arm transforms can contribute only millimetres at the
  observed angular motions (Medium p95 bound `0.0070 m`), not `0.86 m`.
- The diagnostic timing scan found no strong displaced correlation peak. Its
  best Medium translational-speed offset was `+0.12 s`, but correlation changed
  only from `0.55347` to `0.55465`; this is not evidence for a verified offset
  and no offset was applied.

Consequently the previously reported associations remain numerically valid
for their frozen scientific units:

- Short translation conditioning/error Spearman `rho=0.376`, temporal-block
  CI `[0.160, 0.562]`: remains a preliminary positive within-sequence
  association. Short later contains catastrophic GLIM divergence, so it is not
  a general calibration result.
- Medium translation `rho=0.140`, CI crossing zero: remains a weak/inconclusive
  within-sequence result, not evidence of a reliable monotonic relation.
- Medium rotation `rho=-0.177`: remains descriptive and does not support the
  intended monotonic relationship.

Tunnel4 was independently ingested and processed. Stage 1 naturally became
reference-ready after 30 bootstrap acceptances, but those samples contain a
`2.1002 s` gap against the frozen `0.5 s` qualification maximum. Therefore
Tunnel4 is **not** qualified as an offline Gamma reference and no profile was
exported. Stairs is the one recommended final GEODE screening candidate.

## 2. Preservation and source state

Before any D1 work, `git status --short`, `git diff --stat`, and
`git diff --check` were recorded for all nested repositories. Every
`git diff --check` passed. Existing dirty Stage 1/2A/2B/2C work was retained.
No reset, stash, commit, push, or estimator edit was performed.

Safety material is at:

```text
/home/yeranis/stage2c_d1_safety_20260811/
```

It contains tracked binary patches and separate untracked-file archives. The
relevant source states were:

| Repository | Branch | HEAD |
|---|---|---|
| `glim` | `cbs-gtsam43-noetic` | `6c4189e117c61015a79001388debb27413c78fa6` |
| `glim_ros1` | `cbs-gtsam43-noetic` | `2dea515e9e38743b6fc73e7dfdf7e45d3ae5bf6f` |
| `cbsms` | `cbsms/gtsam-4.3-develop` | `873f60fa748df97bf9f5f7f9dcdb825b4f5ed30c` |
| `liorf` | `cbsms/gtsam-4.3-develop` | `4286f982694dda550d1343ff21679f8ec2e708ba` |
| `cbs` | `cbsms/gtsam-4.3-develop` | `994b1d6a5c05fb38dd1b0731c6430ec11d2f1ff0` |
| `Kimera-VIO` | `cbsms/gtsam-4.3-develop` | `d0b2a31adf17ced8995994373f6b286679e26799` |
| `Kimera-VIO-ROS` | `cbsms/gtsam-4.3-develop` | `09d7e17b5d27f97f622f285f0f23a61e6748278d` |

The independent official GEODE checkout was fixed at:

```text
c6e930623d4fed450d7fc50e16e3ffe0288b692b
```

## 3. Files added in D1

Only passive configuration, analysis, test, and report files were added:

```text
src/cbsms/config/geode_stage2c_d1_audit_plan.json
src/cbsms/tools/geode_stage2c_d1_sanity_audit.py
src/cbsms/tools/geode_stage2c_d1_evo_crosscheck.py
src/cbsms/tools/test_geode_stage2c_d1_sanity_audit.py
src/cbsms/docs/2026-08-12_geode_stage2c_d1_evaluator_sanity_and_tunnel4_report.md
```

No GLIM, VGICP, CBS, Kimera, Stage 1, Stage 2A, or Stage 2B source was changed.
The analysis tools have no health-reference output interface and cannot invoke
or configure the estimator.

## 4. Frozen audit plan

The preregistered plan was written before reading D1 results:

```text
src/cbsms/config/geode_stage2c_d1_audit_plan.json
SHA-256 b3fbc8b06d9d42803f41992bd8913fe4a819fb386b9af4fc50e2f60d29a93e8e
```

It fixed:

- primary residual `Logmap(inverse(Z_GT) * Z_G)`;
- 0.05-second inclusive GT bracket;
- last publication per stable edge;
- alternatives A-D as diagnostics only;
- a tenfold reduction before flagging an alternative as a potential convention
  problem;
- timing offsets `[-1,+1] s` at `0.01 s` resolution, diagnostic only;
- deterministic five-row selection for first, median, p95, lowest-condition,
  and highest-condition groups;
- official `evo_ape` alignment: SE(3) Umeyama, no scale, `t_max_diff=0.1`,
  `t_offset=0`;
- the existing Tunnel reference contract: 30 samples, 2.9-second span,
  maximum gap 0.5 seconds, minimum interval acceptance fraction 0.5, and
  natural Stage 1 readiness.

No audit alternative can overwrite the primary output.

## 5. Official GEODE convention and clock audit

Official inputs were read from the GEODE repository and paper, not inferred
from the estimator:

- Repository README and evaluation instructions:
  <https://github.com/PengYu-Team/GEODE_dataset>
- Official Gamma conversion:
  <https://github.com/PengYu-Team/GEODE_dataset/blob/main/script/gamma2GT_gnss.py>
- Official trajectory evaluator wrapper:
  <https://github.com/PengYu-Team/GEODE_dataset/blob/main/script/rmse.py>
- Dataset paper:
  <https://arxiv.org/html/2409.04961v2>

The README states that inland-waterways algorithm trajectories for Gamma must
be spatially converted before evaluation and that effective synchronization
means only spatial offsets are applied. The official conversion constructs a
TUM pose as `T_world_device` and evaluates:

```text
T_world_GT_body = T_world_Gamma_LiDAR * inverse(S)
```

The official `rmse.py` calls:

```text
evo_ape tum <converted method trajectory> <GT trajectory>
        -va --t_max_diff 0.1 --t_offset 0
```

Thus the released rows and method rows are treated as `T_world_body`, not
`T_body_world`. A synthetic constant positive-X trajectory confirms that
`between(T_i,T_j)` has the expected positive relative translation, while
inverting either endpoint order is detected.

The paper documents a GNSS time source with TOD/PPS distributed by FPGA and
states that LiDARs coordinate internally. It also documents the caveat that
host-assigned sensor timestamps can have small transmission/processing delays,
which the dataset authors disregarded. This justifies a diagnostic offset scan,
but not fitting or applying one without independent evidence.

## 6. Absolute-pose conversion comparison

The official conversion script was run unchanged on both full GLIM
Gamma-LiDAR trajectories. The local conversion and official formula were then
compared independently.

| Check | Short | Medium |
|---|---:|---:|
| Formula samples | 101 | 101 |
| Maximum SE(3) Log norm, local vs official formula | `6.53e-13` | `1.14e-13` |
| Official text rows | 4,617 | 7,634 |
| Maximum timestamp difference | `4.77e-7 s` | `4.77e-7 s` |
| Maximum translation difference | `8.40e-7 m` | `8.59e-7 m` |
| Maximum rotation difference | `1.75e-6 rad` | `1.83e-6 rad` |

The sub-micrometre/text-level differences are from the official script's
six-decimal output formatting. They are not a frame discrepancy.

## 7. Short and Medium edge summaries

### 7.1 Translation and duration

| Quantity | Short p10 / median / p90 / p95 / max | Medium p10 / median / p90 / p95 / max |
|---|---|---|
| Duration (s) | 0.199670 / 0.199695 / 0.200486 / 0.200492 / 0.200511 | 0.199670 / 0.199695 / 0.200485 / 0.200490 / 0.200520 |
| GT displacement (m) | 0.148111 / 0.251281 / 0.258902 / 0.261067 / 0.273198 | 0.281250 / 0.439975 / 0.463076 / 0.465856 / 0.491028 |
| GLIM displacement (m) | 0.162851 / 0.246568 / 0.273580 / 0.327719 / 1051.159912 | 0.285347 / 0.439473 / 0.464963 / 0.473232 / 0.648537 |
| Translation residual (m) | 0.304784 / 0.454910 / 0.500135 / 0.530225 / 1263.908770 | 0.542949 / 0.857337 / 0.902777 / 0.913438 / 1.093853 |
| Residual / GT displacement | 1.65772 / 1.83399 / 2.01053 / 2.18145 / 8251.43 | 1.71955 / 1.95441 / 2.04973 / 2.06611 / 2.36364 |
| Translation-vector cosine | -0.96607 / -0.68570 / -0.41803 / -0.15960 / 0.99394 | -0.99149 / -0.92945 / -0.58956 / -0.15329 / 0.99808 |

Short's maximum values expose its known later catastrophic divergence; medians
and p95 remain finite and consistent before those extreme failures. Medium is
narrow: method and GT displacement magnitudes are nearly equal while the
relative vectors are predominantly opposed.

### 7.2 Rotation and speed

| Quantity | Short p10 / median / p90 / p95 / max | Medium p10 / median / p90 / p95 / max |
|---|---|---|
| GT edge rotation (rad) | 0.000474 / 0.001321 / 0.003730 / 0.006442 / 0.037854 | 0.000489 / 0.001456 / 0.005881 / 0.010577 / 0.034313 |
| GLIM edge rotation (rad) | 0.000999 / 0.002289 / 0.005607 / 0.017912 / 3.055679 | 0.002610 / 0.004913 / 0.009675 / 0.012439 / 0.021479 |
| Rotation residual (rad) | 0.001241 / 0.002902 / 0.008174 / 0.034918 / 3.053800 | 0.002906 / 0.005445 / 0.012971 / 0.021109 / 0.053319 |
| GT speed (m/s) | 0.74080 / 1.25641 / 1.29428 / 1.30457 / 1.36818 | 1.40672 / 2.19952 / 2.31502 / 2.32988 / 2.45097 |
| Lever-arm bound (m) | 0.000313 / 0.000873 / 0.002466 / 0.004258 / 0.025019 | 0.000323 / 0.000962 / 0.003887 / 0.006991 / 0.022679 |

The fixed-frame lever-arm effect is at least two orders of magnitude too small
at typical edges to explain the translation residual.

### 7.3 Rank diagnostics

| Spearman diagnostic | Short | Medium |
|---|---:|---:|
| Translation residual vs GT displacement | 0.338 | 0.645 |
| Translation residual vs GT speed | 0.336 | 0.643 |
| Translation residual vs duration | -0.0065 | 0.0302 |
| Translation residual vs GT rotation | -0.128 | -0.378 |

The near-zero duration relationship is expected because the edge horizon is
nearly constant. The speed relationship is descriptive, not a fitted timing
correction.

## 8. Edge-direction alternatives

The accepted evaluator was not changed.

| Dataset / diagnostic | Rotation median / p95 (rad) | Translation median / p95 (m) |
|---|---:|---:|
| Short A: primary | 0.002902 / 0.034918 | 0.454910 / 0.530225 |
| Short B: invert GLIM | 0.002479 / 0.011407 | 0.191682 / 0.334572 |
| Short C: invert GT | 0.002479 / 0.011407 | 0.191606 / 0.334036 |
| Short D: invert both | 0.002902 / 0.034918 | 0.454922 / 0.529487 |
| Medium A: primary | 0.005445 / 0.021109 | 0.857337 / 0.913438 |
| Medium B: invert GLIM | 0.004761 / 0.009466 | 0.156582 / 0.433047 |
| Medium C: invert GT | 0.004761 / 0.009466 | 0.156918 / 0.432928 |
| Medium D: invert both | 0.005445 / 0.021109 | 0.857508 / 0.913535 |

An inversion improves the Median translation diagnostic by factors of about
2.4 (Short) and 5.5 (Medium), but not the preregistered tenfold/order-of-
magnitude trigger and not to near-zero. Both inversions recover the primary
scale. An unapproved diagnostic that transposes GT rotations while retaining
world positions still has `0.251 m` Short and `0.624 m` Medium medians.

These alternatives demonstrate that the opposing local vectors are real in
the recorded estimate; they do not prove the released convention is reversed.
The official script and full-trajectory RPE independently use the primary
convention.

## 9. Official full-trajectory evaluation

Official configuration:

```text
alignment: SE(3) Umeyama, no scale
t_max_diff: 0.1 s
t_offset: 0 s
sensor extrinsic fitted: false
time offset fitted: false
```

| Metric | Short | Medium |
|---|---:|---:|
| Matched poses | 4,467 | 7,605 |
| APE translation median / RMSE / max (m) | 112.949 / 587.549 / 4432.241 | 16.284 / 26.133 / 65.783 |
| Consecutive-frame RPE translation median / RMSE / max (m) | 0.235873 / 90.385 / 2853.878 | 0.411940 / 0.402515 / 2.909238 |
| Consecutive-frame RPE rotation median / RMSE / max (rad) | 0.004755 / 0.469134 / 3.112637 | 0.004904 / 0.009110 / 0.087036 |
| Independent APE max difference | 0 | 0 |
| Independent translation-RPE max difference | `8.81e-13` | `2.29e-13` |
| Independent rotation-RPE max difference | `9.82e-12` | `4.14e-12` |

Numerically and in the generated aligned XY plots, Short is not a plausible
accurate trajectory: its converted GLIM path eventually diverges catastrophically.
Medium preserves a broadly trajectory-scale path but has large accumulated
shape/pose error. The official full-trajectory results are consistent with,
rather than contradictory to, the audited short-edge residuals.

The full-trajectory RPE uses consecutive emitted poses (approximately 0.1 s),
whereas the primary audit uses exact smoother G-to-K beliefs over approximately
0.2 s. It is a scale/convention cross-check, not the identical scientific
object.

## 10. Timing diagnostic

The offset convention is `compare GLIM at t with GT at t+offset`, scanned from
`-1` to `+1 s`. It was frozen before the results and was never applied to the
primary evaluator.

| Dataset | Zero-offset translation-speed Pearson | Best value / offset | Zero-offset angular-speed Pearson | Best value / offset |
|---|---:|---:|---:|---:|
| Short | -0.1132 | -0.1057 at -0.99 s | 0.0344 | 0.0682 at -0.49 s |
| Medium | 0.5535 | 0.5547 at +0.12 s | 0.0839 | 0.1103 at -0.73 s |

Short's correlations are unusably weak because the estimate diverges. Medium's
best translation peak improves Pearson by only `0.00119`; this is not a
meaningful displaced peak. At Medium's median speed, `0.857 m` corresponds to
about `0.39 s`, whereas the weak diagnostic maximum occurs at `0.12 s`.
There is no independent official evidence for either offset, so zero remains
the only defensible primary value.

Decision C (time-synchronization blocker) is therefore not supported.

## 11. Manual edge audit

Exactly 25 rows per dataset were selected by the frozen deterministic rules:

```text
5 first valid
5 nearest median translation residual
5 nearest p95 translation residual
5 lowest translational conditioning
5 highest translational conditioning
```

Every row contains endpoint timestamps and indices, GT endpoint poses,
interpolation brackets, emitted GLIM endpoint poses when present, all
intermediate `T_W_I`, `T_W_L`, and `T_W_B` conversions, `Z_GT`, `Z_I`, `Z_L`,
`Z_B`, the full six-vector residual, translation cosine, and conditioning.

Representative deterministic findings:

- Short first-five errors: `0.0903–0.1008 m`, all cosines `-0.733` to
  `-0.856`.
- Short median group: `0.4548–0.4549 m`; p95 group: `0.5299–0.5306 m`.
- Short highest-conditioning group exposes later divergence (`11.48` to
  `160.29 m`, plus `34.67`, `49.95`, and `85.97 m`).
- Medium first-five errors: `0.2606–0.3055 m`.
- Medium median group: `0.8573–0.8574 m`.
- Medium p95 group: `0.9134–0.9135 m`, cosines `-0.991` to `-0.999`.
- Medium highest-conditioning group: `0.8728–0.9067 m`, cosines `-0.976` to
  `-0.997`.

Machine-readable tables:

```text
stage2c_d1/short_sanity/manual_edge_audit.csv
stage2c_d1/medium_sanity/manual_edge_audit.csv
```

## 12. Evaluator decision

### A — EVALUATOR VERIFIED

Official pose direction, edge direction, frame chain, and documented timing
are mutually consistent. The `~0.86 m` Medium 0.20-second translation residual
is genuine for this local-only GLIM run.

This finding does **not** mean DCReg caused the error or that condition ratio
predicts metres. It only validates the offline correctness target used by the
preliminary association study.

Short's `rho=0.376` and Medium's `rho=0.140` remain correctly computed. Only
Short provides a positive preliminary association with a nonzero temporal-
block CI; Medium remains weak/inconclusive.

## 13. Tunnel4 deterministic ingestion

Official bag:

```text
src/datasets/GEODE/Tunneling_Tunnel4_Gamma/Tunneling_tunnel4_gamma.bag
size: 2,023,936,729 bytes
SHA-256: de0523961cef5983a05c18a9bb8aec158620a8dd8bbd7fa45ce672734f148ffc
duration: 223.943271 s
Livox scans: 2,239
Xsens IMU records: 22,392
```

The frozen Gamma deskew-barrier adapter was run twice. Both output bags were
byte-identical and both serialized message manifests were byte-identical:

```text
converted bag SHA-256:
27dff7b12a537bdc5b3737e6b0402209f20d8111f5f08fe49e3f5a869491f7d7

event/message sequence SHA-256:
f1af6b62f27e6a558168d7c884a9973ca62888a8bafb4a371dc1df46591a5dca

point semantic sequence SHA-256:
308f69954357b6e795c9923d691e8fd23e2dade623f9a60cd9bea71ec7d1a5fb
```

Semantic audit:

- 53,736,000 ordered point records;
- fields `x,y,z,t,intensity,ring`, 18-byte point step;
- point-time range `0–99,866,645 ns`;
- exactly 8,956,000 records for each Livox line ID 0 through 5;
- 22,392 IMU messages copied;
- no mismatches and no terminal uncovered scans;
- deterministic event count 24,631;
- `tag` remains intentionally absent because the frozen GLIM adapter does not
  consume it.

## 14. Tunnel4 local-only run and reference result

The run reused the unchanged Stage 2C-C Gamma configuration manifest:

```text
20ee041e9420015a23c82954e320f0fb7900e6d88ff300011b0d9d1347cdccc8
```

It retained LiDAR matching, IMU, and fixed-lag marginal prior factors. G-to-K
was observe-only; K-to-G used `/geode/stage2c_b/no_k_to_g` and a 999999-second
start delay. The K-to-G CSV contains only its header: zero incoming factors.

Run result:

| Quantity | Result |
|---|---:|
| GLIM return code | 0 |
| Wall time | 81.30 s |
| Aggregate valid Hessian scans | 2,207 / 2,207 |
| Registration-converged scans | 1,955 / 2,207 |
| Linear-solve-success scans | 2,207 / 2,207 |
| Logger snapshots / rows / drops | 2,207 / 2,207 / 0 |
| Health worker snapshots / drops | 2,207 / 0 |

Conditioning:

| Maximum ratio | min | p10 | median | p90 | p95 | max |
|---|---:|---:|---:|---:|---:|---:|
| Rotation | 1.640 | 7.128 | 55.854 | 113.788 | 127.401 | 196.504 |
| Translation | 2.619 | 4.377 | 9.071 | 33.694 | 35.477 | 38.878 |

Absolute-degenerate occupancy:

```text
rotation:    1,876 / 2,207 = 85.00%
translation:   984 / 2,207 = 44.59%
```

Support remained high despite poor geometry:

| Resolution | Median source | Median inliers | Median inlier fraction | Median initial / final cost |
|---|---:|---:|---:|---:|
| 0.5 m | 8,033 | 7,987 | 0.9943 | 616.64 / 517.36 |
| 1.0 m | 8,033 | 8,024 | 0.9988 | 1277.54 / 1192.39 |

Reference activity and rejection reasons:

| Reason | Count |
|---|---:|
| Combined rotation+translation absolute degeneracy | 882 |
| Rotation absolute degeneracy | 994 |
| Translation absolute degeneracy | 102 |
| Unstable condition history | 77 |
| Registration not converged | 43 |
| Axis alignment confidence low | 2 |
| Rotation spectral cluster | 1 |
| Translation spectral cluster | 1 |
| Reference change too large | 2 |
| Bootstrap accepted | 29 |
| Session reference initialized | 1 |
| Later adaptation accepted | 73 |

Stage 1 initialized at frame 2086, timestamp `1706585455.1827319`. The 30
bootstrap acceptances span `5.59984 s`, and all have maximum rotation and
translation ratios below 10. However, they split into two groups with a
`2.100196 s` hole between frames 2060 and 2081. The frozen qualification
maximum is `0.5 s`.

Therefore:

```text
Stage 1 internal reference_ready: true
Frozen independent offline-source qualification: false
Profile exported: no
```

Tunnel4 is not presumed healthy and is not used as a Gamma reference. Per the
requested stop rule, **Stairs_Gamma is the one final GEODE screening
candidate**, using exactly the same adapter, configuration, and gates. Tunnel4
has only position truth in this workflow, and no 6-DoF accuracy claim was made.

## 15. Tests and build status

No compiled source changed, so no C++ rebuild was necessary. Existing frozen
binaries and all relevant Python tools were exercised.

Representative commands were:

```text
docker exec cbsms_ws ... /workspace/cbs_gtsam4.3/devel/scan_dcreg_diagnostics_test
docker exec cbsms_ws ... /workspace/cbs_gtsam4.3/devel/lib/glim_ros/dcreg_edge_metadata_test
docker exec cbsms_ws ... /workspace/cbs_gtsam4.3/devel/lib/kimera_vio_ros/dcreg_shadow_analysis_test
docker exec cbsms_ws ... python3 test_geode_gamma_offline_adapter.py
docker exec cbsms_ws ... python3 test_geode_stage2c_b_evaluator.py
docker exec cbsms_ws ... python3 test_geode_stage2c_c_analysis.py
docker exec cbsms_ws ... python3 test_geode_stage2c_d1_sanity_audit.py
python3 tools/test_glim_dcreg_calibration_protocol.py

evo_ape tum <officially-converted-GLIM> <released-GT> -va --t_max_diff 0.1 --t_offset 0
evo_rpe tum <officially-converted-GLIM> <released-GT> -va --t_max_diff 0.1 --t_offset 0 --delta 1 --delta_unit f
```

| Test | Result |
|---|---:|
| Stage 1 `scan_dcreg_diagnostics_test` | 39 / 39 pass |
| Stage 2A `dcreg_edge_metadata_test` | 31 / 31 pass |
| Stage 2B `dcreg_shadow_analysis_test` | 35 / 35 pass |
| GEODE adapter | 3 / 3 pass |
| GEODE frame/GT evaluator | 21 / 21 pass |
| Stage 2C-C analysis | 20 / 20 pass |
| Stage 2C-D1 sanity audit | 9 / 9 pass |
| Stage 2B offline shadow analysis | 7 / 7 pass |
| Experiment runner | 4 / 4 pass |
| Calibration protocol | 3 / 3 pass under host Python 3.12 |
| Directional audit | 4 / 4 pass |

The calibration-protocol test uses Python 3.9+ generic-annotation syntax and
therefore was run under host Python 3.12, not ROS Noetic Python 3.8. Its source
was unchanged.

New D1 tests cover pose inversion, endpoint-order inversion, translation
cosine, official/local formula equality, official six-decimal TUM output,
the constant-speed offset example, no silent diagnostic replacement, frozen
Tunnel4 qualification, and absence of any GT-to-reference interface.

## 16. Artifacts

Primary files:

```text
stage2c_d1/short_sanity/summary.json
stage2c_d1/short_sanity/edge_sanity_audit.csv
stage2c_d1/short_sanity/manual_edge_audit.csv
stage2c_d1/short_sanity/timing_cross_correlation.csv
stage2c_d1/short_sanity/*.svg

stage2c_d1/medium_sanity/summary.json
stage2c_d1/medium_sanity/edge_sanity_audit.csv
stage2c_d1/medium_sanity/manual_edge_audit.csv
stage2c_d1/medium_sanity/timing_cross_correlation.csv
stage2c_d1/medium_sanity/*.svg

stage2c_d1/full_trajectory/short/
stage2c_d1/full_trajectory/medium/
stage2c_d1/official_conversion/short/
stage2c_d1/official_conversion/medium/

stage2c_d1/tunnel4/converted/
stage2c_d1/tunnel4/runs/tunneling4_gamma_log_only_v1/
stage2c_d1/tunnel4/reference_screening/tunneling4_gamma.json
```

Selected hashes:

| Artifact | SHA-256 |
|---|---|
| Short summary | `1bec1fc59a35769e64e3fbc9d36af34b48fa0710cbf785147691184f931b8ad6` |
| Short edge audit | `e01c59cbe554743d46e411aaba7f333df25600197a84743d827623f6a916f1cc` |
| Short manual audit | `75c8c98f60cbc6e47742462f1913dc32becd0c9e0e22200f8ebeb09ee651f10f` |
| Medium summary | `dc9224b230e8343f8411d2333953857a0ccd3bf9bfafb89cc0c1a715640fcafe` |
| Medium edge audit | `e827d73955ea55c273cc68c3ccf01b01139b5348cbcfcc93f316e14341766324` |
| Medium manual audit | `7fd14b2725ed31729adb48e18859be160739ecf929cdf42a95206359f608b535` |
| Short evo cross-check | `c74f1dcebe2f94dbabbbdc6484a4a02e51668b29791f8c9fe97354d02811b03a` |
| Medium evo cross-check | `080959521d610a3318508dc072bb6f6c4cf3db8885c15772cb605e463ef4c202` |
| Tunnel4 semantic audit | `e607e0329e9fc3c127f6c74c9c21cd0ab884199a7ef237fce94fb69ed81fb84f` |
| Tunnel4 reference screening | `39b18b77ba0012ff09669118619419a07c7106cb7af282343e1b5f6cda4b09bb` |

## 17. Exact next blocker

The evaluator is no longer the blocker. The remaining scientific limitation
is independent replication and reference coverage:

1. Screen Stairs once, with unchanged frozen gates, as the last planned GEODE
   Gamma reference candidate.
2. Do not use Tunnel4 as an offline reference unless a future reviewed policy
   explicitly changes the independent continuity contract; no such change is
   authorized here.
3. Retain Short and Medium as separate degraded-sequence results. Neither
   authorizes condition-to-error calibration, covariance inflation, gating, or
   active weighting.
4. A final healthy-versus-degraded claim still needs a defensible same-Gamma
   healthy reference/evaluation source.

No active weighting was begun.
