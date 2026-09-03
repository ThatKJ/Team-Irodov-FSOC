# 20 — AI Dataset & Training (V2)

**Status:** phase in progress on `feat/ai-perception`. Dataset generation and the
training toolchain are complete; training results are filled in after a run.
See `docs/19_AI_PERCEPTION_ARCHITECTURE.md` for the model and `docs/21_AI_VALIDATION.md`
for evaluation.

## 1. Data source — our own virtual camera, not a downloaded set

The task is highly specialised (a narrow-FOV virtual FSOC tracking camera looking
at a point beacon), so training data is **synthesised from the same virtual
camera model** used by the closed loop. `fsoc_ai_datagen`
(`include/fsoc/ai_frame_synth.hpp`, `src/ai_frame_synth.cpp`) starts from the
identical analytic Gaussian beacon that `SyntheticCameraRenderer` (Step 4, frozen)
produces, then layers difficult optical conditions on top. It is **dataset
tooling only** — not in the control path, and it does not modify the frozen
renderer.

Generator: `apps/generate_ai_dataset.cpp` → executable `generate_ai_dataset`.

```
generated/ai_dataset/                       (git-ignored — regenerate from seed)
├── train/  train_000000.png ...            CV_8UC1 640×480, lossless PNG
├── val/    val_000000.png ...
├── test/   test_000000.png ...
└── metadata/
    ├── train.jsonl  val.jsonl  test.jsonl  one label record per line
    └── dataset.json                        generator version, seed, ranges,
                                            split index ranges, pos/neg counts
```

## 2. Frozen operation order (one sample)

Every sample is built in exactly these stages (negatives skip stage 3 only):

1. float working buffer filled with `background.dark_offset`
2. `+` linear background-gradient plane, `×` radial vignette
3. **positive only:** `+` analytic Gaussian beacon at sub-pixel `(x, y)`, drawn
   peak, drawn σ, optional mild anisotropy (√ratio major / minor) + rotation; the
   centre may sit up to `max_edge_overshoot_px` (6) outside the frame → partial
   edge clipping
4. `+` `K` star-like clutter spots; with probability `bright_distractor_probability`
   one is forced **brighter than the beacon** (`beacon_peak + excess`); optional
   tight 2–4 spot cluster
5. exactly one optical-degradation operator, chosen by weight:
   `none | Gaussian blur | defocus disk | linear motion blur`
6. shot-like noise (Gaussian approximation to Poisson, variance ∝ signal) then
   additive Gaussian read noise
7. hot pixels (→ near-max), dead pixels (→ 0), salt impulses (→ max)
8. clamp `[0, 255]`, round to `CV_8UC1`

All randomness is drawn from `std::mt19937_64` seeded per sample — **never**
`cv::randn`/`cv::randu` (not portably reproducible). `synthesize(seed)` is a pure
function: equal seed ⇒ byte-identical frame and labels on any platform (unit test
`fsoc_ai_datagen_tests`).

## 3. Domain randomization ranges (generator v1.0.0 defaults)

| group | parameter | range |
|---|---|---|
| beacon | peak intensity (counts) | 70 – 255 |
| | PSF σ (px) | 1.2 – 3.2 |
| | anisotropy (major/minor) | 1.0 – 1.6, applied w.p. 0.25 |
| | max edge overshoot (px) | 6 |
| background | dark offset (counts) | 2 – 14 |
| | gradient amplitude (counts) | 0 – 26 |
| | vignette strength | 0 – 0.35 |
| noise | read σ (counts) | 1 – 6 |
| | shot scale | 0 – 1 (× Poisson variance) |
| | hot / dead / salt pixels | 0–12 / 0–6 / 0–20 |
| optical | mode weights | none .45 / gauss .25 / defocus .15 / motion .15 |
| | Gaussian blur σ (px) | 0.6 – 2.2 |
| | defocus radius (px) | 1.0 – 3.5 |
| | motion blur length (px) | 3 – 11, any angle |
| clutter | star count | 0 – 9 |
| | star peak (counts) | 40 – 200 |
| | star σ (px) | 0.7 – 1.8 |
| | bright distractor | w.p. 0.30, excess 6 – 45 counts over beacon |
| | tight cluster | w.p. 0.20 |

The full realised parameter set for every frame is written to the `.jsonl`
manifest, so any sample can be explained, regenerated, or stratified. A single
scalar `difficulty ∈ [0,1]` proxy (weighted mix of inverse-SNR, read noise,
optical strength, clutter count, bright-distractor flag) is recorded **for
stratified reporting only** — it is never a label and the model never sees it.

## 4. Negatives (mandatory)

`negative_fraction` default **0.25** (≈ 25–30 % as the brief suggests). A negative
frame runs every stage except the beacon: background + gradient + vignette +
clutter + optical + noise + hot/dead/salt, `target_present = false`, `x_px` /
`y_px` = `null` (no sentinel). The learned detector must return **no target** →
`std::nullopt` through the perception contract for these.

## 5. Determinism & split integrity

- **Dataset seed:** `26169`.
- One global sample-index space `[0, N)`. Splits are **contiguous, disjoint
  blocks**: `train = [0, n_train)`, `val = [n_train, n_train+n_val)`,
  `test = [..., N)`. Recorded in `metadata/dataset.json → splits.*.global_index_range`.
- Per-sample seed `sample_seed_for(26169, global_index) = splitmix64(26169 ⊕
  splitmix64(global_index + k))`. A sample is therefore fully addressable and
  **no frame can appear in two splits** — different index ⇒ different seed ⇒
  different frame. `fsoc_ai_datagen_tests` checks 20 000 seeds are collision-free.
- Negatives are evenly spaced inside each split (Bresenham pick), giving an exact
  count `round(split_size × negative_fraction)` with no positional clustering.
- `train_beacon_net.py` and `eval_beacon_net.py` assert train/val and val/test id
  sets are disjoint at startup and abort on any overlap.
- Test degradation combinations are drawn from the same distribution but from
  disjoint seeds, so they are not duplicates of training frames.

## 6. Labels

Positive: `target_present = true`, `x_px`, `y_px` (sub-pixel, original 640×480).
Training builds the target heatmap on the fly — a `[60, 80]` Gaussian, σ = 1.75
cells, peak 1.0, at `orig_to_heatmap(x, y)` (truncated by the border for clipped
beacons). Negative: `target_present = false`, all-zero heatmap. **Truth labels
are used only in dataset generation, training, and evaluation — never at
inference.**

## 7. Loss

```
L = λ_p · BCEWithLogits(presence_logit, target_present)
  + λ_h · MSE( sigmoid(heatmap_logit), gaussian_heatmap )
```

`λ_p = λ_h = 1.0` (CLI-tunable, logged separately every epoch). MSE on the
sigmoid heatmap vs the Gaussian target is chosen over focal/BCE-heatmap losses:
at 60×80 there is no severe dense class imbalance, and MSE directly shapes the
surface the soft-argmax decoder integrates. The heatmap loss is applied to
negatives too (all-zero target) so the surface goes flat when no beacon is
present.

## 8. Training toolchain (`tools/ai/`)

Offline only (Python permitted for this per AI brief §5; deployed detector is
C++/OpenCV-DNN). Setup and exact reproduce commands: `tools/ai/README.md`.

| script | role |
|---|---|
| `common.py` | frozen preprocessing / heatmap / decode / coord maps — mirrored in C++ |
| `model.py` | `TinyBeaconNet` |
| `dataset.py` | reads `generated/ai_dataset/` → tensors |
| `train_beacon_net.py` | AdamW + cosine LR, per-epoch val metrics, `best.pt` / `last.pt`, early stopping (`--patience`), `training_history.json` |
| `eval_beacon_net.py` | threshold sweep on **val**, freeze threshold, stratified **test** metrics, latency |
| `export_onnx.py` | `best.pt` → `models/tiny_beacon_net.onnx` (opset 12, dynamic batch) + `.meta.json`; torch↔onnxruntime parity check |
| `make_parity_fixture.py` | writes `tests/fixtures/ai_parity/` for the C++ parity tests |

Device auto-selects Apple MPS → CUDA → CPU. Seeds: `random`, `numpy`, `torch`,
DataLoader generator all set from `--seed` (default 26169).

## 9. Training configuration (recorded after the run)

```
dataset            : generated/ai_dataset  (train 6000 / val 1200 / test 1200, neg 0.25)
seed               : 26169
epochs             : 40 (early-stop patience 8)
batch size         : 32
optimizer          : AdamW  lr 2e-3  weight_decay 1e-4  cosine anneal
lambda presence/hm : 1.0 / 1.0
device             : <filled from run>
trainable params   : <filled from run>   (budget < 1,000,000)
```

## 10. Training results — *TO BE FILLED FROM `models/training_summary.txt`*

| metric (best-val checkpoint) | value |
|---|---|
| epoch of best val | _pending_ |
| val presence precision / recall / F1 | _pending_ |
| val false-positive rate | _pending_ |
| val centroid MAE / RMSE (px) | _pending_ |
| calibrated presence threshold (val, fp ≤ 0.02) | _pending_ |
| test detection rate / FP rate / centroid MAE | _pending_ |
| python (torch CPU) latency mean / p95 (ms) | _pending_ |

Deployed C++ OpenCV-DNN latency (mean / median / p95 / max / throughput) is
measured by `fsoc_ai_validation` and reported in `docs/21_AI_VALIDATION.md`.
