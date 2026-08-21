# Stage 3A: reference-free GLIM LiDAR observability characterization

Date: 2026-08-14

Status: passive analysis complete; review required

Scope: RA-L method characterization only

## Executive decision

Stage 3A supports using the frozen DCReg Schur condition ratios as
**reference-free LiDAR registration observability characterization**. It does
not support treating them as confidence, probability, covariance, trust,
accuracy, or expected metric error.

The strongest positive results are:

1. The exact frozen GLIM/VGICP/DCReg path recovers the expected weak
   rotational and translational subspaces in controlled plane, corridor,
   tunnel, corner, and rich-3D geometries.
2. Schur characterization exposes weakness hidden by apparently
   well-conditioned rotational/translational diagonal blocks. In the clearest
   controlled example, both diagonal blocks have condition `5`, while both
   Schur complements have condition `250.125` after cross-variable
   compensation is allowed.
3. Continuous condition ratios retain severity information in GEODE even when
   the binary `tau=10` mask is active for most edges.
4. Support, local conditioning, matching convergence, scan/IMU consistency,
   and ground-truth correctness are demonstrably different signals.
5. Short shows a preliminary association between continuous translational
   conditioning and true edge error; Medium does not independently replicate
   it. Medium instead supplies a valuable wrong-correspondence-basin failure
   case that local conditioning alone cannot detect.

The paper should therefore present DCReg as one interpretable piece of passive
CBS diagnostic context, not as a complete trust mechanism. Active weighting
is not scientifically authorized by these results.

## 1. Git and source state

The task started from the frozen local commits below.

| Repository | Branch | Frozen HEAD | Stage 3A source changes |
|---|---|---|---|
| `src/glim` | `cbs-gtsam43-noetic` | `6c4189e117c61015a79001388debb27413c78fa6` | none |
| `src/glim_ros1` | `cbs-gtsam43-noetic` | `3bc6927d18ec154214065699f2c2325561afa314` | none |
| `src/liorf` | `cbsms/gtsam-4.3-develop` | `4609767c6f740a7713e8813e64c44a834f716de5` | none |
| `src/cbs` | `cbsms/gtsam-4.3-develop` | `994b1d6a5c05fb38dd1b0731c6430ec11d2f1ff0` | none |
| `src/Kimera-VIO` | `cbsms/gtsam-4.3-develop` | `bcd158f15c45fd27f2054037472e3caeb87e45fe` | none |
| `src/Kimera-VIO-ROS` | `cbsms/gtsam-4.3-develop` | `13b0cc09038b16bcbd8576170dada04532c62068` | none; existing `scripts/__pycache__/` preserved |
| `src/cbsms` | `cbsms/gtsam-4.3-develop` | `e9a0104f0005f69a385baa856f9daa8e68f32b7e` | offline tools/tests/config/report only |

The two pre-existing documentation-only blank-line differences were restored
to the committed bytes. Existing `__pycache__` content was not deleted or
tracked. No reset, restore, stash, amend, rebase, commit, or push occurred.

`git diff --check` is clean. The frozen estimator and ROS repositories remain
unchanged.

## 2. Files added

Source-side additions are confined to `src/cbsms`:

- `config/glim_dcreg_stage3a_analysis_plan.json`: thresholds, bins, taxonomy
  boundaries, wrong-basin selection, and reporting constraints frozen before
  the final analysis.
- `tools/dcreg_stage3a_vgicp_harness.cpp`: deterministic offline harness using
  the actual `IntegratedVGICPFactor` and frozen GLIM DCReg analyzer.
- `tools/dcreg_stage3a_characterization.py`: reference-free synthetic and
  GEODE analysis plus SVG/table generation.
- `tools/dcreg_stage3a_extract_case_clouds.py`: deterministic, read-only
  extraction of point-cloud samples for the preregistered Medium wrong-basin
  cases from the existing converted bag.
- `tools/test_dcreg_stage3a_characterization.py`: focused detector/baseline
  tests.
- this report.

Generated results are outside the nested source repositories under:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/stage3a_characterization/
```

The generated directory is `5.7 MiB`. It contains no bag, RRD, estimator
output, or source change.

## 3. Frozen detector semantics

The production quantity is derived only from the final local LiDAR matching
factors. For the final optimized scan pose, GLIM linearizes each unary VGICP
factor, extracts the `[rx, ry, rz, tx, ty, tz]` local-right-tangent Hessian,
and sums the two frozen Gamma voxel levels (`0.5 m`, `1.0 m`) for one scan.
IMU factors, marginal priors, smoother factors, and CBS factors are not in this
Hessian.

Partition

```text
H = [ H_RR  H_Rt ]
    [ H_tR  H_tt ]
```

and form controlled-pseudoinverse Schur complements

```text
S_R = H_RR - H_Rt H_tt^+ H_tR
S_t = H_tt - H_tR H_RR^+ H_Rt.
```

The authoritative reference-free quantities are

```text
kappa_R = max eigenvalue(S_R) / floored min eigenvalue(S_R)
kappa_t = max eigenvalue(S_t) / floored min eigenvalue(S_t)
D_R = [kappa_R > tau_R]
D_t = [kappa_t > tau_t]
```

with production `tau_R=tau_t=10`. Continuous ratios remain authoritative.
The plotting-only signed margin is

```text
m = log(tau / kappa).
```

No Stage 3A result changes `tau=10` or writes to GLIM/CBS.

## 4. Controlled synthetic methodology

The offline C++ harness uses:

- the frozen `gtsam_points::IntegratedVGICPFactor` implementation;
- the Gamma CPU profile's `0.5 m` and `1.0 m` voxel maps;
- one VGICP thread;
- production unary-factor Hessian extraction;
- the frozen exported `glim::analyzeScanDcregHessian` symbol;
- deterministic point generation and noise seeds;
- true identity source/target alignment for geometry-isolation cases;
- separate deterministic noise, half-support, and initial-perturbation cases.

The planar surfaces are finite large patches, rather than mathematical
infinite planes. Surface covariance is deterministic and anisotropic. Thus the
experiment tests expected modes and comparative detector behavior; it does not
claim universal numerical kappa values for all LiDARs or covariance models.

The generated JSON is byte deterministic across repeated runs:

```text
SHA-256 15cfd25b46b7a33a5f9333056210509cd1c23cf332e6e3abddbfd75eed6e967c
wall time 0.090 s for 12 scenes
```

An independent NumPy implementation of the frozen pseudoinverse and Schur
equations agrees with the authoritative C++ output to maximum relative error
`1.232e-13`. The maximum absolute eigenvalue difference is `4.366e-6` on
eigenvalues of order `1e8`; it is a library eigensolver rounding difference,
not a detector discrepancy.

## 5. Expected and detected weak modes

| Scene | Points | Expected weak R | Expected weak t | kappa_R | kappa_t | Weak-projector overlap R / t |
|---|---:|---|---|---:|---:|---:|
| Single plane | 625 | normal-axis rotation (`rz`) | two in-plane translations (`tx,ty`) | 88.89 | 179.34 | 1.000 / 1.000 |
| Parallel-plane corridor | 960 | corridor-axis rotation (`rx`) | corridor/vertical translations (`ty,tz`) | 151.69 | 177.78 | 1.000 / 1.000 |
| Cylindrical tunnel | 1,008 | tunnel-axis rotation (`rx`) | tunnel-axis translation (`tx`) | 111.12 | 62.57 | 1.000 / 1.000 |
| Perpendicular corner | 792 | none expected | remaining vertical translation (`tz`) | 1.39 | 74.95 | n/a / 1.000 |
| Three orthogonal surfaces | 972 | none expected | none expected | 1.04 | 8.00 | n/a / n/a |
| Rich irregular 3D | 1,014 | none expected | none expected | 2.54 | 7.68 | n/a / n/a |
| Sparse rich 3D | 150 | none expected | none expected | 2.31 | 6.82 | n/a / n/a |
| Dense degenerate plane | 2,304 | same as plane | same as plane | 88.89 | 179.40 | 1.000 / 1.000 |
| Tilted long-range plane | 625 | plane-normal rotation | two plane-tangent translations | 88.89 | 112.95 | 1.000 / 0.999 |

The projector comparison is sign invariant and handles multidimensional weak
subspaces. This is essential for the plane's two translational weak modes and
for clustered spectra. A pure physical-axis label is not assigned when the
eigenspace is clustered or alignment is ambiguous.

Noise, overlap, support, and initialization variations preserve the rich-3D
shape conclusion:

| Variant | Inlier fraction | kappa_R | kappa_t |
|---|---:|---:|---:|
| Rich baseline | 1.000 | 2.540 | 7.679 |
| 2 cm deterministic source noise | 0.972 | 2.541 | 7.715 |
| Half source support | 1.000 | 2.522 | 7.679 |
| Larger initial perturbation, optimized | 0.996 | 2.521 | 7.652 |

This does not prove robustness outside the tested basin; it shows that the
local characterization behaves consistently for these controlled cases.

## 6. Detector baselines

| Baseline | Matrix/signal | Handles R/t coupling? | Scale sensitive? | Directional? | Reference free? |
|---|---|---|---|---|---|
| DCReg Schur condition | `S_R`, `S_t` | yes, through conditional elimination | no under uniform scaling | separate R/t modes/subspaces | yes |
| Full-Hessian condition | full `6x6 H` | implicit | no | one mixed 6D mode | yes |
| Full-Hessian minimum eigenvalue | full `6x6 H` | implicit | yes | one mixed 6D mode | yes |
| Fixed numerical rank/nullity | full `6x6 H` | implicit | threshold/scale dependent | no stable physical interpretation alone | yes |
| Diagonal-block condition | `H_RR`, `H_tt` | no | no | separate but conditional compensation ignored | yes |
| Diagonal-block minimum eigenvalue | `H_RR`, `H_tt` | no | yes | separate but conditional compensation ignored | yes |
| Inlier fraction | correspondence support | no | normalized count only | no | yes |
| Source/inlier count | correspondence support | no | yes | no | yes |

Concrete baseline failures:

- All 12 synthetic Hessians have numerical rank 6 because finite VGICP
  covariance regularization supplies small positive information. Rank therefore
  does not distinguish the plane from rich 3D under the frozen threshold.
- The dense plane has a larger full-Hessian minimum eigenvalue (`356,749`) than
  sparse rich 3D (`179,922`) because it has far more measurements, even though
  its translational shape ratio is degenerate (`179.40` versus `6.82`).
- The dense plane has 2,304 points and inlier fraction 1.0, but remains strongly
  degenerate. Support-only indicators cannot describe directional geometry.
- Full-Hessian condition does separate these particular scenes (`1006.6` for
  dense plane versus `95.3` for sparse rich 3D), but mixes radians/metres and
  does not say whether rotational or translational observability is weak.

## 7. Why Schur decoupling matters

The controlled coupled positive-semidefinite quadratic uses

```text
H_RR = H_tt = diag(100, 50, 20)
H_Rt = H_tR = diag(99.9, 0, 0).
```

Both diagonal blocks have eigenvalues `[20,50,100]` and condition `5`, which
appears benign. Allowing translation to compensate rotation, and vice versa,
gives Schur eigenvalues `[0.1999,20,50]` and condition `250.125` in both groups.

Intuitively, `H_RR` answers how much the objective changes if translation is
held fixed. `S_R` answers how much rotational information remains after the
best compensating translation is allowed. The latter is the relevant local
observability question when both variables are optimized together.

The example is explanatory, not a claim that every real VGICP frame produces
that exact block structure.

## 8. Threshold sensitivity and binary saturation

Thresholds `{5,10,20}` were frozen before analysis. Occupancies below use only
edges with valid continuous kappa. Missing kappa remains unavailable, never
nondegenerate.

| Sequence | tau | D_R occupancy | D_t occupancy | Longest R run | Longest t run |
|---|---:|---:|---:|---:|---:|
| Short | 5 | 97.47% | 99.71% | 109.5 s | 241.0 s |
| Short | 10 | 91.93% | 98.47% | 105.6 s | 100.4 s |
| Short | 20 | 77.08% | 88.59% | 91.9 s | 95.5 s |
| Medium | 5 | 94.77% | 99.14% | 160.6 s | 351.5 s |
| Medium | 10 | 77.81% | 95.97% | 102.3 s | 189.6 s |
| Medium | 20 | 62.88% | 84.97% | 97.1 s | 133.7 s |

Short has 2,094 kappa-available edges and 70 unavailable edges. For continuity
with the Stage 2C report, counting the degenerate numerator over all 2,164
GT-valid edges gives 88.96% rotational and 95.29% translational occupancy; the
correct available-data denominators give 91.93% and 98.47%. Both denominator
definitions are now explicit.

Despite binary saturation, continuous severity remains broad:

| Sequence | kappa_R median / p10 / p95 | kappa_t median / p10 / p95 |
|---|---|---|
| Short | 75.49 / 11.20 / 229.01 | 139.45 / 18.72 / 378.36 |
| Medium | 35.24 / 6.11 / 240.64 | 70.66 / 16.09 / 406.45 |

At `tau=10`, a binary value says almost every Short translation edge is beyond
the boundary, while continuous kappa still distinguishes approximately 19,
139, and 378 at p10/median/p95. This supports continuous reporting alongside
the binary mask.

## 9. GEODE conditioning versus true error

The verified Stage 2C evaluator and unique-stable-edge policy were reused
without modification. Sequences are reported separately.

| Sequence | GT-valid unique edges | log kappa_R vs rotation error | log kappa_t vs translation error |
|---|---:|---|---|
| Inland Short | 2,164 | rho `0.103`, CI `[-0.044,0.248]` | rho `0.376`, temporal-block CI `[0.160,0.562]` |
| Inland Medium | 5,242 | rho `-0.177`, CI `[-0.343,-0.0004]` | rho `0.140`, CI `[-0.077,0.339]` |

Interpretation:

- Short provides preliminary evidence that more severe translational
  conditioning can coincide with greater true translational edge error.
- Medium does not independently replicate that monotonic translation result.
- The negative Medium rotational association is further evidence that kappa
  is not a universal correctness predictor.
- The sequences are not pooled and no condition-to-metre mapping is fitted.

Fixed conditioning bins tell the same nuanced story. Short median translation
error rises from `0.438 m` in `[10,20)` to `0.463 m` in `>=100`, but the change
is modest relative to saturation. Medium's medians are non-monotonic:
`0.863 m` below 5, `0.581 m` in `[10,20)`, and `0.862 m` at `>=100`.

## 10. Support, matching, and scan-IMU consistency

Support is complementary rather than interchangeable with conditioning.

| Sequence | Inlier fraction vs translation error | Source points vs translation error | Scan-IMU translation disagreement vs translation error |
|---|---:|---:|---:|
| Short | rho `-0.223` | rho `-0.227` | rho `0.112` |
| Medium | rho `-0.380` | rho `-0.219` | rho `0.330` |

Short separates the signals particularly clearly:

- 82 low-support edges (`inlier fraction <0.25`) have median translation error
  `40.37 m`, reflecting the known extreme failure tail.
- 2,081 high-support edges have median `0.454 m` error, but 137 still exceed the
  fixed descriptive `0.5 m` boundary.
- Within high support, continuous translational conditioning retains rho
  `0.367` with translation error.

All 5,242 Medium edges fall in the preregistered high-support stratum, yet 4,796
exceed `0.5 m` translation error. Inlier fraction still has rho `-0.380` within
its high range, while conditioning has rho `0.140`. High support does not prove
correct correspondence.

Initial/final cost associations with Medium translation error are weak
(`-0.035`, `-0.044`). Cost describes the selected local solution and does not
prove it is the correct basin.

The existing passive scan/IMU diagnostic is mathematically defined as norms
of

```text
inverse(pred_T_last_current) * scan_T_last_current.
```

It was available for every eligible Short and Medium edge. Median translation
disagreement is `0.353 m` Short and `0.122 m` Medium. Its descriptive
translation-error association is weak in Short (`0.112`) and larger in Medium
(`0.330`). This is a promising complementary signal, but it remains an
uncalibrated passive diagnostic and is not used for control here.

## 11. Failure taxonomy

The taxonomy rules were frozen before execution:

- poor conditioning: `kappa>10`;
- high translation error: `>=0.5 m`;
- high rotation error: `>=0.02 rad`;
- low support: inlier fraction `<0.25`;
- high support: inlier fraction `>=0.50`.

These are descriptive bins, not detector-accuracy labels. Categories overlap
where they describe different dimensions.

| Translation category | Short | Medium |
|---|---:|---:|
| Poor conditioning + high error | 150 | 4,594 |
| Poor conditioning + low error | 1,912 | 437 |
| Moderate/good conditioning + high error | 0 | 202 |
| Moderate/good conditioning + low error | 32 | 9 |
| Conditioning unavailable | 70 | 0 |
| High support + high error | 137 | 4,796 |
| Low support + high error | 82 | 0 |
| At least one associated scan not convergence-flagged | 1,846 | 3,930 |
| Fully converged/solved high-support wrong-basin candidate | 9 | 892 |

The large “not convergence-flagged” count reflects GLIM's very strict local
increment convergence criterion. It is kept separate from solve failure and
true error; it is not treated as rejected registration.

## 12. Medium wrong-basin case study

Wrong-basin candidates obey a fixed rule: translation error at least `0.5 m`,
vector cosine at most `-0.8`, GLIM/GT displacement-magnitude ratio in
`[0.8,1.25]`, inlier fraction at least `0.5`, valid continuous kappa, and all
associated scans converged and solved. The six displayed cases are the
earliest candidate in each of six equal-duration bins, not cherry-picked by
kappa or error.

| Edge | GT translation [m] | GLIM translation [m] | cosine | error [m] | kappa_t | inlier fraction |
|---|---|---|---:|---:|---:|---:|
| 983->985 | `[-0.3767,-0.2825,0.0256]` | `[0.4523,0.0234,0.0521]` | -0.816 | 0.884 | 121.96 | 0.797 |
| 2069->2071 | `[-0.4282,-0.1617,0.0271]` | `[0.3855,0.0322,0.0329]` | -0.951 | 0.836 | 310.45 | 0.786 |
| 3163->3165 | `[-0.3028,-0.1654,0.0245]` | `[0.2865,0.0260,0.1044]` | -0.835 | 0.625 | 7.28 | 0.847 |
| 4229->4231 | `[-0.4244,-0.0220,0.0105]` | `[0.4209,0.0346,-0.0968]` | -0.979 | 0.854 | 100.73 | 0.936 |
| 5302->5304 | `[-0.4504,-0.0440,0.0233]` | `[0.4520,0.0476,-0.0565]` | -0.997 | 0.911 | 25.71 | 0.842 |
| 6389->6391 | `[-0.3969,-0.1925,0.0264]` | `[0.4447,0.0370,-0.0413]` | -0.933 | 0.875 | 17.55 | 0.905 |

Edge `3163->3165` is the decisive limitation: it is high support, converged,
solved, has nearly opposite motion, and has `kappa_t=7.28`, inside the default
observability boundary. DCReg correctly characterizes the local Hessian around
the selected solution; it cannot establish that the correspondences belong to
the globally correct basin.

For geometric context, the deterministic extractor recovered the two endpoint
clouds for every displayed edge from the existing converted Medium bag. All
12 cloud matches are within `116 ns` of the requested sensor timestamp and are
stored at a bounded `2,500` points per cloud. The source artifacts are:

```text
wrong_basin_case_clouds.npz
SHA-256 cfe923b576e3486c3116aacb8c16ce0d579d33919e7311861626e7840b882b35

wrong_basin_case_clouds.manifest.json
SHA-256 7b952a51bcb2171cd5c3448aa291a58b0c1bb9cb262345d164ca4000896bc6cb
```

Extraction is passive and does not rerun or alter GLIM.

This limitation is a central paper result, not an anomaly to hide.

## 13. Paper-facing CBS representation

For each G-to-K belief, the frozen Stage 2A side channel should be described as
separate evidence groups:

```text
Observability:
  worst kappa_R, worst kappa_t
  rotational/translational absolute-degenerate fraction
  longest degenerate run

Support:
  source-point, inlier-count, and inlier-fraction summaries

Matching:
  initial/final cost, convergence, solve success

Validity:
  association completeness, missing/invalid coverage, ambiguity

Optional receiver-side observation:
  GLIM-Kimera rotational and translational disagreement
```

This is descriptive modality-specific context alongside the Gaussian belief.
It is not trust, confidence, uncertainty, or a covariance multiplier. The
legacy mean and covariance are unchanged.

## 14. Relative health position

The implemented relative-health layer remains available as optional
normalization against an independently qualified, sensor- and
configuration-compatible normal-observability profile.

Its fail-safe properties remain important:

- degraded startup cannot initialize itself as healthy;
- incompatible offline profiles are rejected;
- no qualifying evidence means `reference_ready=false` and health unavailable;
- no healthy value of 1 is fabricated;
- public GEODE Gamma Flat, Inland, Offroad7, Tunnel1, Tunnel4, and Stairs did
  not yield an authorized compatible offline reference.

Therefore relative Gamma health is absent from the principal Stage 3A
evaluation. The system's refusal to manufacture a reference is a positive
scientific-safety result.

## 15. Runtime and passivity evidence

Frozen accepted measurements are retained:

| Component | Measurement |
|---|---:|
| Stage 1 monitor | median `0.01395 ms/scan` |
| Stage 1 nonblocking queue publish | mean `0.002094 ms/scan` |
| Stage 2A belief-thread descriptor copy | mean `1.008-1.039 us/array` |
| Stage 2A worker association/hash/message | mean `68.8-82.0 us/array` |
| Stage 2B active belief enqueue | maximum `2.744 us` in isolated run |
| Stage 2B receiver observation enqueue | maximum `2.894 us` in isolated run |
| Producer waits | zero |
| Accepted deterministic estimator/CBS no-op | exact equality at `1e-12` |

Stage 3A is entirely offline and has no ROS publisher or estimator callback.
Its final analysis over 7,406 unique edges and 12 synthetic scenes takes about
`3.5 s` in the accepted container.

## 16. Candidate paper figures

1. [Figure A: synthetic geometry and weak modes](../../../stage3a_characterization/analysis/figures/figure_a_synthetic_geometry.svg)
2. [Figure B: Schur versus diagonal-block coupling](../../../stage3a_characterization/analysis/figures/figure_b_schur_coupling.svg)
3. [Figure C: GEODE continuous/binary/error/support timeline](../../../stage3a_characterization/analysis/figures/figure_c_geode_timeline.svg)
4. [Figure D: conditioning versus true error, sequences separate](../../../stage3a_characterization/analysis/figures/figure_d_conditioning_vs_error.svg)
5. [Figure E: wrong-basin vectors and failure taxonomy](../../../stage3a_characterization/analysis/figures/figure_e_failure_taxonomy.svg)
6. [Figure F: covariance-neutral CBS diagnostic side channel](../../../stage3a_characterization/analysis/figures/figure_f_cbs_architecture.svg)

All six SVGs parse as valid XML. SHA-256 values and byte sizes are recorded in
`stage3a_analysis_summary.json`.

## 17. Candidate paper tables and artifacts

Generated tables:

- `tables/detector_comparison.csv`
- `tables/synthetic_scene_characterization.csv`
- `tables/threshold_sensitivity.csv`
- `tables/geode_short_medium_findings.csv`
- `tables/failure_taxonomy.csv`
- `tables/wrong_basin_cases.csv`
- `tables/runtime_passivity.csv`
- enriched unique-edge tables for Short and Medium.

Authoritative summary:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/stage3a_characterization/analysis/stage3a_analysis_summary.json
SHA-256 964ba3b63c3d2078b06bdcff88f76454b9afe3f62922411b4165bee67f8c82bb
```

## 18. Tests and builds

The offline harness was compiled against the already-built frozen workspace:

```text
g++ -std=c++17 -O2 \
  -I src/gtsam/gtsam/3rdparty/Eigen \
  -I src/glim/include \
  -I src/glim/thirdparty/json/include \
  -I src/gtsam_points/include -I devel/include \
  tools/dcreg_stage3a_vgicp_harness.cpp \
  -L devel -L devel/lib -lglim -lgtsam_points -lgtsam -ltbb
```

Results:

- Stage 3A focused Python tests: `16/16` passed.
- Frozen Stage 1 C++ tests: `39/39` passed.
- Frozen Stage 2A C++ tests: `31/31` passed.
- Frozen Stage 2B C++ tests: `35/35` passed.
- Complete `src/cbsms/tools` discovery in the accepted Python 3.8 container:
  90 test cases passed; one pre-existing module failed import because its
  annotation uses Python 3.9+ `list[tuple[...]]` syntax.
- That unchanged calibration-protocol module passed `3/3` under the host
  Python 3.12 interpreter. No historical source was modified to hide the
  environment-version incompatibility.
- Repeated synthetic output had an identical SHA-256.
- All figure SVGs passed XML parsing.
- `git diff --check` passed.

No production package required rebuilding because no frozen implementation or
CMake file changed. The C++ harness itself compiled and ran successfully
against the frozen libraries.

## 19. Claim-support matrix

| Claim | Decision | Exact evidence |
|---|---|---|
| 1. Pure LiDAR registration observability can be extracted without IMU/prior/CBS contamination. | **SUPPORTED** | Frozen capture linearizes only final unary VGICP matching factors; deterministic no-op is exact at `1e-12`; synthetic harness calls the same factors/analyzer. |
| 2. Schur characterization provides R/t observability after cross-coupling. | **SUPPORTED** | Exact equations and coupled PSD example: block condition 5 versus Schur 250.125; expected synthetic weak projectors overlap 0.999-1.000. |
| 3. Continuous conditioning retains severity when binary degeneration saturates. | **SUPPORTED** | Translational tau-10 occupancy is 98.47% Short and 95.97% Medium, while continuous kappa retains broad p10/median/p95 ranges. Thresholds 5/10/20 change occupancy but not continuous ordering. |
| 4. Observability and support/matching correctness are distinct. | **SUPPORTED** | Sparse rich geometry remains well shaped; dense plane remains degenerate. Medium has high support on all edges and 4,796 high-error edges; 892 fully converged/solved wrong-basin candidates remain. |
| 5. Observability degradation can coincide with increased true error but is not universal. | **PARTIALLY SUPPORTED** | Short translation rho 0.376 with positive block CI; Medium translation rho 0.140 with CI through zero; Medium rotation is negative; a kappa_t=7.28 wrong-basin case exists. |
| 6. CBS can propagate interpretable modality-specific context without altering the belief. | **SUPPORTED** | Frozen Stage 2A exact association/identity, unchanged legacy bytes, exact deterministic estimator/CBS equality, separate optional topic, zero control consumer. |

## 20. Remaining limitations

1. Controlled geometry uses finite patches and chosen anisotropic point
   covariances. It validates expected local modes, not universal threshold
   calibration.
2. GEODE supplies two degraded waterways recordings but no independently
   qualified same-Gamma healthy reference/evaluation pair.
3. Short's translational result is preliminary within-sequence evidence;
   Medium did not replicate it.
4. Local Hessian observability cannot detect a locally consistent but globally
   wrong correspondence basin.
5. Stage 1 describes scan-to-rolling-map geometry, whereas the evaluated CBS
   edge also reflects IMU, fixed-lag smoothing, and marginal information.
6. Neighboring unique edges remain temporally correlated; accepted confidence
   intervals use temporal blocks and are not independent dataset replication.
7. The optional scan/IMU diagnostic is useful complementary evidence but has
   not been independently calibrated or validated as a control signal.

## 21. Recommended RA-L method statement

Recommended concise method language:

> We passively extract the pure LiDAR VGICP Hessian at the final GLIM scan pose
> and characterize rotational and translational observability using Schur
> complements in the local right tangent. The resulting condition ratios and
> absolute masks describe local registration geometry after accounting for
> rotation-translation compensation. They are transmitted as optional CBS
> metadata alongside, but never used to modify, the original Gaussian belief.
> Controlled geometry verifies the expected weak subspaces. Real-data results
> show that continuous conditioning can expose severity beyond a saturated
> binary mask and can coincide with increased error, while wrong-basin cases
> demonstrate that local observability is not registration correctness.

The paper should pair observability with separate support, matching,
convergence, and optional scan/IMU consistency evidence. It should not propose
a single opaque confidence score in this stage.

## 22. Authorization boundary

Stage 3A does **not** authorize:

- covariance inflation or replacement;
- condition-to-error/probability calibration;
- active factor weighting or gating;
- belief rejection;
- Kimera/coordinator policy;
- directional interval aggregation;
- a GT-trained threshold;
- a new healthy-reference search.

**Active weighting is not yet scientifically authorized.**
