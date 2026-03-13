# Deployment Scenarios

This document covers the two deployment modes Tri-Link supports, their difficulty profiles, concrete industry case studies, and the recommended implementation order.

For the business context behind these scenarios — the industries, customers, and problem being solved — see [background.md](background.md).

---

## End-to-End Scenarios

### Outdoor lot with cloud inference

**Context:** A vehicle inspection site with stable network connectivity (outdoor lot, covered but ventilated structure).

**Flow:**
1. The patrol robot begins its pre-programmed route through the car park.
2. At each vehicle, the robot's camera captures images. Each image is timestamped to the microsecond at the moment of capture. The robot's SLAM system records its exact pose at the same moment.
3. Tri-Link receives each frame and forwards the JPEG to the cloud inference endpoint, attaching the capture timestamp as a request header.
4. The inference service (running the full-precision damage detection model) responds with bounding boxes: class, confidence, and pixel coordinates for each detected defect.
5. Tri-Link looks up the robot's pose at the original capture time — not the current time — correcting for the ~100 ms inference delay.
6. Each bounding box center is unprojected from pixel space into 3D world coordinates using the camera calibration and the historical pose.
7. A record is written to `damage_map.jsonl` and `damage.db`: damage class, confidence, world position, and a reference to the saved JPEG.
8. The customer opens the 3D model of the site and sees damage markers overlaid at the correct physical positions.

**Inference deployment:** Cloud. No quantization required.

---

### Underground car park with no network (on-robot inference)

**Context:** An underground car park or tunnel structure with no reliable network connectivity.

**Flow:** Same as the outdoor scenario, except:
- The inference service runs on the robot's on-board processor.
- The damage detection model must be quantized to fit within the robot's SoC memory and compute budget. Accuracy is validated before deployment.
- `[inference].base_url` in `config.toml` points to `localhost`.
- Tri-Link code is unchanged — it calls an HTTP endpoint either way.
- Damage records accumulate locally during the patrol.
- After the robot returns to a network-connected docking station, the damage map is synced to the cloud and made available to customers.

**Inference deployment:** On-robot. Quantization required.

---

### Customer reviews the results after a patrol

**Context:** After a patrol completes, a fleet manager reviews the results.

**Flow:**
1. The customer opens the 3D model of the site in the robot platform's cloud viewer.
2. Damage markers (sourced from `damage.db`) are overlaid at their world coordinates.
3. The customer clicks a marker to see: damage class, confidence score, captured JPEG, and timestamp.
4. The customer exports a damage report for insurance or repair dispatch.

Tri-Link's role ends at step 2 — it produces the `damage.db` records that populate the markers. Integration of those records into the 3D viewer is the robot platform's responsibility.

---

## Deployment Difficulty Comparison

The Tri-Link **code** is the same in both scenarios — only `[inference].base_url` in `config.toml` changes. The difficulty difference is almost entirely outside Tri-Link, in the model and hardware layer.

| Aspect | Cloud inference | On-robot inference |
|---|---|---|
| **Tri-Link code change** | None | None |
| **Model quantization** | Not required | Required — high difficulty |
| **Quantization toolchain** | Not required | Hardware-specific (TensorRT, CoreML, etc.) |
| **Accuracy regression testing** | Low risk | Required on every model update |
| **Hardware constraints** | None | SoC memory and compute limits are tight |
| **Clock drift during patrol** | Mitigated by NTP | Risk on long patrols with no NTP signal |
| **Deployment operations** | Standard cloud deploy | Firmware-style deploy across a fleet of robots; rollback strategy needed |
| **Post-patrol data sync** | Not needed | Needed — sync to cloud when docked; edge case if sync is interrupted |
| **Overall difficulty** | Medium | High |

**What drives the high difficulty on-robot:**

1. **Quantization is not a one-time task.** Every time the vision AI platform updates the damage detection model, the model must be re-quantized, re-validated for accuracy loss, and re-deployed to the robot fleet. This creates an ongoing operational burden.

2. **Accuracy validation.** Quantization (INT8 or INT4) can degrade detection precision for small or low-contrast defects such as fine scratches. The quantized model must be benchmarked against the full-precision baseline on a representative test set before deployment.

3. **Hardware-specific toolchain.** Each robot SoC (NVIDIA Jetson, Qualcomm, etc.) has a different quantization and inference runtime. The vision AI platform team must support this toolchain in addition to their cloud deployment.

4. **Fleet rollout.** Pushing a new quantized model to a fleet of robots requires version management, staged rollout, and a rollback path if the model causes regressions.

**What stays easy on-robot:**

Tri-Link itself does not change. The pose buffer, unprojection math, and output format are all the same. The only operational difference from Tri-Link's perspective is that damage records accumulate locally and are synced after the robot docks.

---

## Deployment Case Studies

### Case Study A — Rental fleet depot (cloud inference)

**Operator:** A national car rental company with 800+ vehicles across three regional depots.

**Environment:** Outdoor, canopied depot structures with stable enterprise Wi-Fi throughout. Vehicles enter and leave continuously throughout the day.

**Problem:** Each vehicle must be inspected at check-out and check-in. With 200+ transactions per day per depot, manual inspection takes 8–12 minutes per vehicle and produces inconsistent records. Damage disputes with customers are frequent and time-consuming to resolve.

**Deployment:** Cloud inference. The robot patrols the depot every 2 hours. Tri-Link forwards each JPEG to the cloud inference endpoint in real time.

**Business outcome:** Damage dispute resolution time drops from 30 minutes (staff reviewing photos manually) to under 2 minutes (staff clicks a damage marker in the 3D model, sees the timestamped JPEG and exact panel location). Liability is attributed before the next customer collects the vehicle.

**Why cloud works here:** Stable connectivity throughout the site. No quantization or hardware constraints. The full-precision model delivers the highest detection accuracy for small defects such as fine key scratches.

---

### Case Study B — Seaport vehicle holding area (cloud inference)

**Operator:** A seaport authority receiving 3,000–5,000 vehicles per week from international car carriers.

**Environment:** Outdoor holding lots of 20–80 hectares. Vehicles arrive by roll-on/roll-off ship and sit in the lot for 1–4 days before handover to road carriers. Stable 4G/LTE and Wi-Fi cover the entire port area.

**Problem:** Damage occurring during unloading and lot handling must be attributed to the correct party: the ship carrier, the port handler, or the outbound road carrier. Currently, teams of inspectors walk the lot at shift changes. At peak throughput a lot holds 12,000 vehicles — a full walk takes 6 hours and still misses damage.

**Deployment:** Cloud inference. A fleet of patrol robots covers the lot daily. Each robot reports to the same cloud inference endpoint.

**Business outcome:** Every vehicle in the lot is scanned every 24 hours. New damage detected between two consecutive scans is automatically flagged with the timestamp window, narrowing liability to a specific shift and handling team. The 3D world map of the port shows exactly which vehicle and panel is affected.

**Why cloud works here:** The port has reliable outdoor connectivity. The full-precision model is important because paint transfer from loading equipment is small and low-contrast. Fleet-scale patrol (multiple robots) is a natural fit for a cloud endpoint with no per-robot inference hardware.

---

### Case Study C — Underground city car park (on-robot inference)

**Operator:** A facility management company operating multi-story urban car parks for corporate tenants. Tenants lease parking bays under long-term contracts and are charged for damage at contract end.

**Environment:** Floors B1–B4 are underground with no cellular or Wi-Fi coverage. The robot docks at street level where it connects to the building network.

**Problem:** The operator needs a damage record for each vehicle at the start and end of the lease contract, and a periodic record showing damage that accumulates during the contract. Manual inspection of 400 underground bays is impractical at the frequency needed.

**Deployment:** On-robot inference. The robot patrols all underground floors during off-peak hours (02:00–06:00). Damage records accumulate on-robot. When the robot docks at street level at the end of the patrol, the damage map syncs to the cloud.

**Differences from the cloud scenario:**

| Item | Requirement |
|---|---|
| Tri-Link code | Unchanged — `base_url` points to `localhost` |
| Model | Quantized to fit the robot's SoC; accuracy validated before deployment |
| Post-patrol sync | Automated when robot docks and network becomes available |
| Fleet management | Model updates must be pushed to each robot; staged rollout with rollback |
| Clock drift | Robot clock runs unsynchronized during patrol; validated to drift < 5 ms over a 4-hour patrol |

**Why on-robot is necessary here:** No alternative — the signal environment is a physical constraint. All the on-robot complexity (quantization, fleet management) is a cost that must be paid to serve this use case.

---

## Recommended Implementation Order

**Implement cloud inference (Case Studies A and B) first.**

### Rationale

**1. The largest addressable market is connected.**
Car rental depots, port vehicle lots, and auction yards — the three highest-volume inspection environments — are all outdoor sites with reliable connectivity. On-robot inference is needed only for sites where network is physically unavailable (underground structures). The connected market is larger and more immediately accessible.

**2. Cloud inference validates the entire Tri-Link pipeline without external dependencies.**
The full pipeline — ingress, pose buffer, unprojection, inference client, semantic map output — can be built, tested, and deployed using only standard HTTP to a cloud endpoint. No quantization, no SoC toolchain, no hardware constraints. This is the fastest path to a working, customer-testable system.

**3. A reference deployment is a prerequisite for on-robot negotiations.**
Convincing the vision AI platform to invest in quantization toolchain support and the robot platform to qualify on-robot inference requires a proof of value. A live cloud deployment with real customer results is that proof. Without it, the on-robot effort is speculative for all three parties.

**4. On-robot adds a dependency that is outside our control.**
Quantization requires the vision AI platform to commit engineering time to a hardware-specific toolchain. That commitment is easier to secure after a working cloud deployment demonstrates revenue potential. Trying to solve quantization before the core pipeline is proven inverts the risk.

**5. Risk and feedback velocity.**
The cloud deployment can iterate quickly: update the model endpoint, redeploy, measure accuracy. On-robot iteration requires rebuilding the quantized artifact, pushing firmware to robots, and re-validating — a cycle that is 5–10× slower. Proving the concept in the cloud setting first means defects in the pipeline are found and fixed before they become embedded in a slower iteration loop.

### Suggested phasing

| Phase | Deployment | Target industry | Prerequisite |
|---|---|---|---|
| **Phase 1** | Cloud inference | Car rental, port operations, auctions | Core pipeline complete (M1–M6) |
| **Phase 2** | On-robot inference | Underground car parks, tunnels, signal-dark facilities | Phase 1 reference deployment; vision AI platform quantization support confirmed |

Phase 2 requires no changes to Tri-Link code. The investment in Phase 2 is entirely in the model quantization pipeline and robot fleet management tooling — work that is justified by Phase 1 revenue and customer demand.

---

## How Accurate Is the Damage Location?

The target is 1 cm position accuracy. The key factors are depth sensor quality, camera calibration, clock synchronization, and inference latency. See [assumptions.md](assumptions.md#damage-localization-accuracy) for the full accuracy parameter table.
