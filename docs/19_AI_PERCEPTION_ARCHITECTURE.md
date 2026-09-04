# 19 — AI Perception Architecture (V2)

**Status:** phase in progress on `feat/ai-perception`. `v1_baseline` is frozen and
untouched. This document is the design contract for the learned perception stack;
see `DECISIONS.md` ADR-015 / ADR-016 and `docs/15_INTERFACE_CONTRACTS.md`.

## 1. Why AI, and where it lives

The validated v1 detector is:

```
pixels → threshold → 8-connected components → brightest component
       → intensity-weighted centroid → BeaconDetection
```

Transparent, deterministic, ~0.02 px sub-pixel error on clean interior frames.
Its weakness is **selection under difficulty**: it has no model of *which* bright
region is the beacon. When the frame degrades — low SNR, star-like clutter,
multiple bright points, a distractor brighter than the target, hot pixels, blur,
defocus, motion blur, background gradient, partial edge clipping, or the beacon
briefly absent — "brightest integrated signal" is not the right answer, and the
classical detector either locks onto the wrong blob or returns a centroid for a
target that is not there.

`TinyBeaconNet` is a small CNN trained specifically on those conditions. It
**only changes perception**. The learned detector emits the *same*
`std::optional<BeaconDetection>` as the classical one; everything downstream —
`compute_tracking_error`, the PID law and its frozen gains, `PanTiltCamera`, the
actuator limits, the `SimulationRunner` step order, the Step-10 gates — is
unchanged.

```
                          CAMERA FRAME  (CV_8UC1 640×480)
                                 │
                  ┌──────────────┴───────────────┐
                  │                              │
          CLASSICAL DETECTOR             AI BEACON DETECTOR
          (fsoc_perception, v1)          (fsoc_ai_perception, TinyBeaconNet)
          brightest CC → weighted        preprocess → ONNX → (presence, heatmap)
          sub-pixel centroid             → soft-argmax sub-pixel centroid
                  │                              │
                  └──────────────┬───────────────┘
                                 │
                        HYBRID PERCEPTION  (config-driven policy)
                                 │
                        std::optional<BeaconDetection>     ← unchanged contract
                                 │
                          compute_tracking_error           ← unchanged
                                 │
                          PIDController (kp=12, ki=0, kd=0) ← unchanged
                                 │
                          PanTiltCamera::step              ← unchanged
```

**AI EXISTS ONLY IN PERCEPTION.** The gimbal control law is the validated v1 PID.

## 2. TinyBeaconNet

Fully-convolutional heatmap-localization network. **Not** YOLO / bounding-box
detection — the target is a sub-pixel point source, so the network predicts a
location *heatmap* plus a *presence* probability, and a sub-pixel decoder reads
the centroid off the heatmap.

```
Input  [N, 1, 240, 320]  float32 ∈ [0,1]
  stem      Conv 1→16  3×3 s2  · BN · ReLU        → 16 ×120×160
  block1    DWConv 16 s2 · PWConv 16→32 · BN·ReLU → 32 × 60× 80
  block2    DWConv 32 s1 · PWConv 32→48 · BN·ReLU → 48 × 60× 80
  block3    DWConv 48 s1 · PWConv 48→64 · BN·ReLU → 64 × 60× 80   (= HM_H×HM_W)
  ├─ heatmap head   Conv 64→32 3×3 · BN·ReLU · Conv 32→1 1×1 → [N,1,60,80] logits
  └─ presence head  GlobalMaxPool(64) · FC 64→32 · ReLU · FC 32→1 → [N,1] logit
```

- **Presence pooling is global *max*, not average** (ADR-017): the beacon occupies
  << 1 cell of the 60×80 trunk map, so averaging over ~4800 cells destroys the
  "is there a peak anywhere" signal. `MaxPool` stays inside the frozen opset-12
  ONNX interface and is OpenCV-DNN-safe.
- **Parameter budget:** < 1,000,000 trainable params. Actual **27,282** (printed by
  `train_beacon_net.py`; recorded in `models/tiny_beacon_net.meta.json` and the
  model card).
- **Ops only:** Conv, depthwise-separable Conv, BatchNorm, ReLU, global max
  pool, Linear. No pretrained weights, no ResNet/ViT/CLIP.
- **Raw logits out.** `sigmoid()` is applied *outside* the ONNX graph, identically
  in Python eval and in C++, so the graph stays tiny and `BCEWithLogits` is exact.

## 3. Frozen numeric contract (`tools/ai/common.py` ⇔ `src/ai_beacon_detector.cpp`)

| constant | value | meaning |
|---|---|---|
| `ORIG_W × ORIG_H` | 640 × 480 | native virtual-camera frame |
| `IN_W × IN_H` | 320 × 240 | network input (native ÷ 2) |
| `HM_W × HM_H` | 80 × 60 | heatmap head (native ÷ 8, input ÷ 4) |
| `INPUT_STRIDE` | 8 | original px per heatmap cell |
| `HEATMAP_SIGMA_CELLS` | 1.75 | Gaussian label σ, in heatmap cells |
| `DECODE_WINDOW_RADIUS` | 2 | soft-argmax uses a 5×5 window around the peak |

**Preprocessing (frozen):** `cv2.resize(frame, (320,240), INTER_AREA)` →
`float32` → `× 1/255` → NCHW `[1,1,240,320]`. No mean/std standardization. The C++
path calls the identical OpenCV `INTER_AREA` resize, so Python↔C++ preprocessing
is numerically equivalent (parity test, tolerance ±1 count).

**Coordinate map (pixel-centre aligned, matches renderer `cx = W/2.0`):**
```
x_hm   = (x_orig + 0.5) / 8 − 0.5
x_orig = (x_hm   + 0.5) × 8 − 0.5           (same for y)
```
Convention unchanged: origin top-left, `+x` right, `+y` down.

**Heatmap decode (frozen):**
1. `sigmoid` the heatmap logits.
2. integer `argmax` → peak cell.
3. soft-argmax = intensity-weighted centroid over the 5×5 window (clamped to the
   grid) using the sigmoid values as weights → sub-pixel `(x_hm, y_hm)`.
4. map back to original pixels.
`peak_confidence` = sigmoid value at the argmax cell.

## 4. C++ AI detector (`fsoc_ai_perception`, lands in a later stage)

```cpp
struct AiBeaconDetectorConfig {
    std::string model_path;        // models/tiny_beacon_net.onnx
    double presence_threshold;     // from models/threshold.json (val-calibrated)
    int input_width  = 320;
    int input_height = 240;
};

struct AiBeaconDetection {
    BeaconDetection detection;     // the controller-facing contract
    double confidence;             // sigmoid(presence_logit) ∈ [0,1] — DIAGNOSTIC
    double peak_confidence;        // heatmap peak sigmoid — DIAGNOSTIC
    double inference_ms;           // DIAGNOSTIC
};

class AiBeaconDetector {
public:
    explicit AiBeaconDetector(AiBeaconDetectorConfig);   // validates the model file
    [[nodiscard]] std::optional<AiBeaconDetection> detect(const cv::Mat& frame) const;
};
```

- Constructor fails cleanly (`std::invalid_argument` / `std::runtime_error`) on:
  missing / unreadable model, output tensor shapes ≠ the frozen contract, invalid
  config, invalid dimensions.
- `detect()` rejects an empty frame or any type other than `CV_8UC1`, matching the
  classical detector's validation style.
- The controller path consumes only `std::optional<BeaconDetection>`. `confidence`
  is exposed for telemetry/hybrid logic and never reaches the PID.

## 5. Perception modes & hybrid policy

```cpp
enum class PerceptionMode   { Classical, AI, Hybrid };            // control-relevant selection
enum class PerceptionSource { None, Classical, AI, HybridAgreement };  // DIAGNOSTIC only
```

`SimulationRunnerConfig` gains a `PerceptionMode` field, **default `Classical`**
(bit-identical to v1 — regression-tested). Hybrid policy (all thresholds
config-driven, defaults below):

| situation | action | `PerceptionSource` |
|---|---|---|
| classical **and** AI detect, centroids within `agreement_radius_px` (default 2.0) | use the **classical** centroid (best clean sub-pixel precision) | `HybridAgreement` |
| only AI detects, `confidence ≥ presence_threshold` | use AI | `AI` |
| only classical detects | use classical | `Classical` |
| both detect but centroids disagree > `agreement_radius_px` | if `confidence ≥ high_confidence_threshold` (default 0.90) use AI, else return `std::nullopt` (perception ambiguity → safe hold) | `AI` / `None` |
| neither detects | `std::nullopt` | `None` |

Two unrelated detections are **never averaged**. On `std::nullopt` the existing
target-loss policy runs unchanged (PID reset, zero command, camera holds).

## 6. Diagnostics (telemetry only — never control)

`ai_confidence`, `perception_source`, `classical_ai_distance_px`,
`ai_inference_ms`. These are additive telemetry fields; they do not touch
`TrackingState` / `DemoRunState` and the PID never reads them.

## 7. Synthetic-to-real limitation

`TinyBeaconNet` v1 is trained **exclusively** on domain-randomized synthetic
virtual-camera imagery from `fsoc_ai_datagen`. No real optical sensor data, no
measured atmospheric turbulence, no flight heritage, no hardware-in-the-loop. The
domain randomization (see `docs/20`) is broad, but a real sensor will have noise
statistics, PSF shape, and fixed-pattern artefacts this model has not seen. This
is stated in `models/MODEL_CARD.md` and must not be overstated in the demo.

## 8. Documented future work (NOT this phase)

Phase 2 spatio-temporal (3-frame) detector · Phase 3 UKF state estimation ·
Phase 4 motion prediction / feed-forward · Phase 5 MPC · Phase 6 satellite
ephemeris + SGP4 coarse pointing · Phase 7 real camera + physical gimbal.
