# Stage 2C-C: Gamma reference qualification and Inland absolute-conditioning analysis

Date: 2026-08-11  
Status: **NO REFERENCE FOUND + ABSOLUTE ANALYSIS COMPLETE (Outcome B)**

## 1. Outcome and scientific boundary

Both approved passive tracks are complete:

- Track A screened exactly two additional same-Gamma sequences without
  weakening any frozen Stage 1 gate. Neither qualified as a healthy reference.
- Track B joined frozen Stage 1/2A observability diagnostics to the 2,164
  previously verified, deduplicated Inland full-6-DoF ground-truth edges and
  ran the preregistered within-sequence analysis.

The clearest preliminary result is a moderate positive monotonic association
between maximum translational condition and true translational edge error on
the 2,094 edges having valid conditioning:

```text
Spearman rho = 0.3763
10 s temporal-block bootstrap 95% CI = [0.1603, 0.5617]
```

The rotational association was weak and its interval included zero:

```text
Spearman rho = 0.1034
10 s temporal-block bootstrap 95% CI = [-0.0444, 0.2479]
```

This is a preliminary within-Inland association, not a ratio-to-error
calibration. No mapping to metres or centimetres, covariance multiplier,
probability, trust weight, or active policy was fitted or implemented.
Relative health remained unavailable because no valid Gamma reference exists.

No Stage 1, Stage 2A, Stage 2B, GLIM, VGICP, CBS, Kimera, covariance, factor,
or optimizer behavior was changed.

## 2. Preserved source state

Before editing, repository status/diff state was recorded. Safety patches and
untracked-file archives are under:

```text
/home/yeranis/stage2c_c_safety_20260811/
```

No reset, stash, discard, commit, or push occurred. Branches and HEADs were
unchanged:

| Repository | Branch | HEAD |
|---|---|---|
| `glim` | `cbs-gtsam43-noetic` | `6c4189e117c61015a79001388debb27413c78fa6` |
| `glim_ros1` | `cbs-gtsam43-noetic` | `2dea515e9e38743b6fc73e7dfdf7e45d3ae5bf6f` |
| `cbsms` | `cbsms/gtsam-4.3-develop` | `873f60fa748df97bf9f5f7f9dcdb825b4f5ed30c` |
| `liorf` | `cbsms/gtsam-4.3-develop` | `4286f982694dda550d1343ff21679f8ec2e708ba` |
| `cbs` | `cbsms/gtsam-4.3-develop` | `994b1d6a5c05fb38dd1b0731c6430ec11d2f1ff0` |
| `Kimera-VIO` | `cbsms/gtsam-4.3-develop` | `d0b2a31adf17ced8995994373f6b286679e26799` |
| `Kimera-VIO-ROS` | `cbsms/gtsam-4.3-develop` | `09d7e17b5d27f97f622f285f0f23a61e6748278d` |

All final `git diff --check` invocations were clean. Existing dirty Stage
1/2A/2B work was preserved.

Stage 2C-C added these `cbsms` files:

```text
config/geode_stage2c_c_analysis_plan.json
config/geode_stage2c_c_provenance.json
tools/geode_stage2c_c_analysis.py
tools/geode_stage2c_c_rerun.py
tools/test_geode_stage2c_c_analysis.py
docs/2026-08-11_geode_stage2c_c_reference_and_absolute_analysis_report.md
```

It also updated `src/datasets/GEODE/DOWNLOAD_MANIFEST.md`. Large experiment
artifacts remain untracked under `stage2c_c/`.

## 3. Frozen analysis plan and provenance

The plan was written before final Inland associations:

```text
config/geode_stage2c_c_analysis_plan.json
SHA-256 92ad7e7f430a226515c0869b23a784820133b4e774794861b7ddfccfef5b6fca
```

Frozen choices included:

- primary GT interpolation bracket `<= 0.05 s`;
- last publication per stable edge as scientific unit;
- fixed bins `[1,10)`, `[10,30)`, `[30,100)`, `[100,300)`, `[300,inf)`;
- at least 30 edges per interpretable bin and at least three adequate bins;
- useful range requires `p90/p10 >= 3`;
- 10-second temporal blocks, 10,000 bootstrap replicates, 95% coverage,
  random seed `20260811`;
- reference qualification requires at least 30 accepted samples, at least
  2.9 seconds span, maximum accepted gap `<= 0.5 s`, acceptance fraction
  `>= 0.5`, a naturally ready reference, and exact fingerprint match;
- GT cannot enter reference construction;
- no regression to metric error and no threshold retuning.

Machine-readable provenance is in
`config/geode_stage2c_c_provenance.json`. The preferred second candidate,
`Stairs_Gamma`, could not be downloaded because its public Google Drive file
quota was exhausted. The single allowed fallback was the irregular-walled
`Tunneling_Tunnel1_Gamma`; searching stopped after it. Its translation-only GT
with zero quaternion fields was not used.

## 4. Deterministic conversion and runtime configuration

Both candidate conversions were executed twice. Independent A/B conversions
were byte-identical at bag and serialized-message-manifest levels.

| Sequence | Scans | IMUs | Events | Event SHA-256 | Output bag SHA-256 |
|---|---:|---:|---:|---|---|
| Offroad7 | 1,841 | 18,410 | 20,251 | `e30bbaa2adf835be3bbddc043c2bccec978a8b9557a162883d87da666e4f3f3e` | `be6384a089ef6b2fd9c99070c1af9b186800e689e8efdb9670ae6b388c50e895` |
| Tunnel1 | 2,088 | 20,881 | 22,969 | `de9b6323ed4925134792487f98ac497201702681376547da5532506995149aa4` | `846b0e5a29c5ddf730b39f9c72e072340fd5b216440dd00da20ab7cc18c39fa8` |

There were zero uncovered scans. Ordered `x,y,z,t,intensity,ring`, nanosecond
point offsets, all six line IDs, and IMU payloads/timestamps were preserved.
Livox `tag` remains omitted because frozen GLIM does not consume it.

The tunnel source bag is 1,820,964,443 bytes with SHA-256
`79f89b1c8653c65841fd00dcc0f1ed24799f9d0a839b34f107ec336cac5ebb50`.

Every Gamma sequence used the same GLIM/VGICP configuration fingerprint:

```text
VGICP|1|9.9999999999999998e-13|1.0000000000000001e-09|1e-08|0.5|1
```

Generated-config manifest SHA-256:

```text
20ee041e9420015a23c82954e320f0fb7900e6d88ff300011b0d9d1347cdccc8
```

Runs retained LiDAR matching, IMU, and fixed-lag marginal prior factors. G-to-K
publication was observe-only, K-to-G reception used an unused topic and
999999-second delay, and Stage 1 remained `log_only`.

| Sequence | GLIM return | Health rows | Wall time | Logger/worker drops | K-to-G factors |
|---|---:|---:|---:|---:|---:|
| Offroad7 | 0 | 1,809 | 63.60 s | 0 | 0 |
| Tunnel1 | 0 | 2,056 | 73.57 s | 0 | 0 |

## 5. Track A: reference screening

### Offroad7

Offroad7 is not healthy under frozen rules.

| Quantity | Result |
|---|---:|
| Valid Hessians | 1,809 / 1,809 |
| Accepted candidates | 0 |
| Reference-ready samples | 0 |
| Absolute rotation degeneracy | 1,680 / 1,809 (92.87%) |
| Absolute translation degeneracy | 1,809 / 1,809 (100%) |
| Median / p95 max rotation condition | 24.660 / 308.831 |
| Median / p95 max translation condition | 190.270 / 426.378 |

Rejections were 1,680 combined rotation/translation degeneracy and 129
translation degeneracy. Median source count was 4,886; median inlier fraction
was 0.949 at 0.5 m and 0.985 at 1.0 m. Strong support did not remove the
directional shape degeneration.

### Tunnel1

Tunnel1 was richer but did not satisfy the contract.

| Quantity | Result |
|---|---:|
| Valid Hessians | 2,056 / 2,056 |
| Accepted candidates | 26 (minimum 30) |
| Accepted span | 5.2005 s |
| Maximum accepted gap | 0.6007 s (limit 0.5) |
| Acceptance fraction | 0.4906 (minimum 0.5) |
| Reference-ready samples | 0 |
| Absolute rotation degeneracy | 1,948 / 2,056 (94.75%) |
| Absolute translation degeneracy | 1,294 / 2,056 (62.94%) |
| Median / p95 max rotation condition | 36.877 / 129.385 |
| Median / p95 max translation condition | 12.144 / 23.072 |

Records comprised 1,228 combined-degeneracy, 720 rotation-degeneracy, 66
translation-degeneracy, 26 accepted bootstrap candidates, 10 unstable-history,
and 6 non-converged cases. Median source count was 8,068; median inlier
fractions were 0.995 and 0.999 at the two resolutions.

No Gamma profile was exported. There is no Inland hybrid rerun, no fabricated
`health=1`, and no relative-health analysis. The frozen gates correctly
refused to learn frequently degenerate geometry as normal.

## 6. Track B: verified Inland edge set

The accepted released `E` and `S` frame chain, stable GT sorting, SLERP, no
extrapolation, no alignment, no fitted time offset, exact `<= 0.05 s` bracket,
and stable-edge deduplication were reused unchanged.

The standalone GTSAM fixture remained below `1e-12`:

```text
endpoint-versus-edge path          8.186e-16
G-to-K payload round trip          1.712e-16
left-world-alignment cancellation  1.755e-15
quaternion-sign invariance         1.694e-19
```

| Quantity | Count |
|---|---:|
| Belief publication rows | 27,636 |
| Unique stable edges | 4,604 |
| GT-valid unique edges | 2,164 |
| Rejected for bracket > 0.05 s | 2,361 |
| Rejected outside coverage | 79 |
| Valid conditioning available | 2,094 |
| Relative health available | 0 |

Rolling republications were not counted independently.

## 7. Edge errors and invalid-health category

Across all 2,164 GT-valid edges:

| Error | Median | p95 | Maximum |
|---|---:|---:|---:|
| Rotation | 0.002902 rad | 0.034918 rad | 3.053800 rad |
| Translation | 0.454910 m | 0.530225 m | 1263.908770 m |

The extreme maxima occur during later estimator failure/divergence. Spearman
ranks and quantiles are more robust, but the 70 edges without valid
conditioning are not hidden or encoded as healthy. Their errors were severe:

```text
rotation median 1.266 rad, p95 2.486 rad
translation median 40.37 m, p95 471.84 m
```

Unavailable/numerically invalid health is therefore a distinct failure
category, but it cannot be assigned a finite condition ratio.

## 8. Conditioning range and occupancy

The frozen dynamic-range test passed:

| Statistic | Rotation | Translation |
|---|---:|---:|
| Valid edges | 2,094 | 2,094 |
| Minimum | 1.049 | 3.236 |
| p10 | 11.204 | 18.722 |
| Median | 75.495 | 139.452 |
| p90 | 201.274 | 324.855 |
| p95 | 229.013 | 378.363 |
| p90/p10 | 17.965 | 17.352 |
| Adequate bins | 4 | 5 |

Near-`1e9` maxima represent thresholded near-singularity, not metric error.

Binary absolute degeneration was saturated:

```text
rotation    1,925 / 2,164 = 88.96%
translation 2,062 / 2,164 = 95.29%
```

Occupancy/error associations were correspondingly weak:

| Predictor/response | rho | 95% temporal-block CI |
|---|---:|---:|
| Rotation occupancy / rotation error | -0.0610 | [-0.1776, 0.0659] |
| Translation occupancy / translation error | 0.0875 | [-0.0348, 0.2097] |

The masks remain valid absolute labels, but Inland is not a balanced binary
comparison.

## 9. Continuous conditioning versus true error

Primary preregistered results:

| Predictor/response | rho | 95% temporal-block CI | Interpretation |
|---|---:|---:|---|
| `ln(max rotation condition)` / rotation error | 0.1034 | [-0.0444, 0.2479] | weak; includes zero |
| `ln(max translation condition)` / translation error | 0.3763 | [0.1603, 0.5617] | moderate positive preliminary association |

Translation bins:

| Condition bin | Edges | Median error [m] | p95 [m] |
|---|---:|---:|---:|
| `[1,10)` | 32 | 0.4383 | 0.4784 |
| `[10,30)` | 396 | 0.4366 | 0.4703 |
| `[30,100)` | 450 | 0.4509 | 0.4798 |
| `[100,300)` | 944 | 0.4642 | 0.5093 |
| `[300,inf)` | 272 | 0.4594 | 0.5516 |

This is not a deterministic lookup: the median is not perfectly monotonic,
and the highest bin has large outliers while retaining a typical median.
Rotation-bin medians were not monotonic; its highest bin had only 17 edges and
failed the minimum-count criterion.

Longest absolute-run correlations were negative rather than the expected
positive relation (`rotation rho=-0.1928`, `translation rho=-0.1436`). This is
not interpreted as protective: the mask is saturated, edges are short,
invalid records break runs, and catastrophic failure is represented by
unavailable rather than long finite runs.

## 10. Support and matching remain separate

Exploratory Spearman associations:

| Predictor | Rotation error rho | Translation error rho |
|---|---:|---:|
| Source points | -0.1610 | -0.2272 |
| Inlier fraction | -0.1656 | -0.2228 |
| Initial cost | -0.0041 | -0.0785 |
| Final cost | -0.0385 | -0.0608 |

More support/inliers generally accompanied lower errors, while raw matching
cost was weak. No combined score was created.

## 11. Calibration sensitivity and passivity

Released calibration remains primary. No transform or time offset was fitted.
Because Stage 2C-B established no independent accuracy bounds for `E` and `S`
perturbations, sensitivity remains pending rather than choosing magnitudes
after inspecting correlations.

Stage 2C-C added offline tools only. The accepted Stage 2C-B deterministic
Gamma proof already established byte-identical health-off versus `log_only`
GLIM poses, registrations, G-to-K means/covariances, and local factors. Runtime
code is unchanged here. Both new manifests report observe-only CBS, no active
health/covariance policy, and zero incoming K-to-G factors. GT is only read by
the offline evaluator.

No profile qualified, so the conditional profile-mode Inland no-op rerun was
not applicable.

## 12. Rerun recordings

Every Stage 2C-C experiment has a finalized passive Rerun file. All pass
`rerun rrd verify`:

| Recording | Size | SHA-256 |
|---|---:|---|
| `stage2c_c/rerun/offroad7_gamma_stage2c_c.rrd` | 7,020,022 B | `3436df0b84200814304f9152ee4da8588e413c22aba1aabd350ff699d256d891` |
| `stage2c_c/rerun/tunneling1_gamma_stage2c_c.rrd` | 14,017,032 B | `1f68c284cc1d0075b93277097a57006d24a2b2244df200a241ceaeeebf817aef` |
| `stage2c_c/rerun/inland_waterways_short_gamma_stage2c_c.rrd` | 1,666,255 B | `21ad8b6c8d724e82c06cf0cf0aba1fd02787d6cf1908df6ed0453884f51cc913` |

Candidate recordings contain bounded LiDAR clouds, GLIM trajectories,
condition ratios, masks, reference-candidate/ready state, and per-resolution
support/costs. Inland contains GT trajectory, true edge errors, conditioning,
occupancy, support/costs, and interpolation brackets. Rerun is offline and
cannot affect estimator timing.

## 13. Build and tests

Build:

```text
catkin build liorf glim glim_ros cbs cbsms kimera_vio kimera_vio_ros \
  --no-deps --no-status --summarize -j8 -p2
```

All 11 scheduled packages/dependencies succeeded with no warnings.

Tests:

- frozen Stage 1: 39/39;
- frozen Stage 2A: 31/31;
- frozen Stage 2B C++: 35/35;
- original Livox conversion: 2/2;
- GEODE adapter: 3/3;
- GEODE frame/GT evaluator: 21/21;
- Stage 2C-C: 20/20;
- Stage 2B offline shadow: 7/7;
- experiment runner: 4/4;
- calibration/directional reporting: 3/3 and 4/4;
- standalone GTSAM frame fixture: pass;
- three Rerun recordings: verified.

The two legacy reporting scripts require Python 3.9+ annotation syntax, so
they ran under host Python 3.12 rather than ROS Noetic Python 3.8. Their code
was unchanged.

Primary artifacts:

```text
stage2c_c/reference_screening/offroad7_gamma.json
stage2c_c/reference_screening/tunneling1_gamma.json
stage2c_c/inland_absolute_analysis_v1/summary.json
stage2c_c/inland_absolute_analysis_v1/inland_unique_edge_conditioning_analysis.csv
stage2c_c/inland_absolute_analysis_v1/rotation_condition_vs_gt_error.svg
stage2c_c/inland_absolute_analysis_v1/translation_condition_vs_gt_error.svg
stage2c_c/runs/offroad7_gamma_log_only_v1/
stage2c_c/runs/tunneling1_gamma_log_only_v1/
stage2c_c/rerun/
```

Converted test bags occupy about 1.71 GB; candidate source bags about 3.22 GB;
Rerun output about 22.7 MB. Health queues reported zero drops.

## 14. Limitations and decision

Limitations:

1. No frozen-gate-compatible Gamma reference exists, so relative health was
   not evaluated.
2. Inland is one degraded sequence, not an independent healthy/degraded suite.
3. Temporal block bootstrap cannot create sequence-level replication.
4. Binary masks are saturated.
5. Seventy severe-failure edges have unavailable conditioning and cannot enter
   finite-ratio correlations.
6. Stage 1 measures scan-to-map LiDAR geometry, whereas the evaluated edge is
   a LiDAR/IMU fixed-lag smoother output.
7. Released-calibration sensitivity lacks independent perturbation bounds.

### Outcome B — NO REFERENCE FOUND + ABSOLUTE ANALYSIS COMPLETE

Continuous translational conditioning has a moderate preliminary association
with true Inland short-edge translation error. Rotation does not show an
equally clear association, and threshold occupancy is too saturated for a
strong binary result.

The blocker to a final claim is an independent full-6DoF same-Gamma evaluation
sequence that uses the same calibration/configuration, naturally passes every
frozen healthy gate, has verified GT, and is held out from reference
construction and analysis development.

Nothing here authorizes active weighting, covariance inflation, factor gating,
coordinator design, or health-to-error calibration.
