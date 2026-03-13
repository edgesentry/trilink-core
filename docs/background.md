# Project Background

## The Industries That Need This

Vehicle inspection is a routine but labor-intensive task across several industries. The common thread: large numbers of vehicles, high turnover, and a business need to document the condition of each vehicle precisely and quickly.

**Car rental operators**
Rental companies must document the condition of every vehicle at check-out and check-in to determine whether damage occurred during the rental period. A typical depot holds hundreds of vehicles. Manual inspection by a staff member before and after every rental is slow, inconsistent, and a frequent source of customer disputes. Automated inspection reduces staff time and creates an objective photographic record for damage claims.

**Vehicle logistics and port operations**
When vehicles are shipped by sea or rail, they pass through staging yards and port terminals that may hold tens of thousands of units at a time. Damage occurring during transit must be detected and attributed to the correct handling party (carrier, port handler, rail operator). A patrol robot that scans the lot daily and maps new damage to specific vehicles solves a problem that currently requires teams of inspectors walking large outdoor areas in all weather conditions.

**Used vehicle auctions and wholesale dealers**
Auction houses grade vehicle condition to establish pricing. A consistent, automated condition report that includes 3D-located damage improves buyer confidence and reduces post-sale disputes. High-volume auction sites process hundreds of vehicles per day — manual inspection is a bottleneck.

**Fleet and leasing companies**
Corporate fleet operators and vehicle leasing companies need damage records when a vehicle is returned at the end of a lease contract. The inspection determines whether the lessee owes a damage charge. Automated inspection removes the subjectivity of a manual check and provides a timestamped, spatially-located record that is harder to dispute.

**Insurance assessment**
After an incident, an insurer or assessor needs to document and locate all damage on a vehicle. Spatial damage records anchored in a 3D model provide a structured evidence base that supplements or replaces traditional photo-based reports.

---

## The Customer

The customer is the operator of the vehicle lot — a rental company, port authority, auction house, or fleet manager. They use the 3D damage map to:

- Identify damage and assign liability before a vehicle leaves or after it returns
- Track damage over a vehicle's service life
- Reduce time spent on manual walk-around inspections
- Provide objective, timestamped evidence in dispute resolution

---

## The Problem Tri-Link Solves

Vehicle inspection at scale is a manual, time-consuming process. A person walks through a car park lot, visually checks each vehicle surface for scratches, dents, and cracks, and records findings on paper or a mobile device. At a large site — a rental fleet depot, an auction yard, a port vehicle holding area — this takes hours and is prone to missed damage and inconsistent reporting.

An autonomous patrol robot can walk the same route faster and without fatigue. An AI model can flag damage in images more consistently than a tired human eye. But combining the two creates a new problem: a list of damage detections is only useful if you know *where* each one is — not just "frame 4,712 of the rear-left camera", but "the rear bumper of the gray sedan in bay C-14".

Tri-Link is the layer that converts image-space detections into 3D spatial records anchored in the robot's world model. The output — a damage map — lets customers open the site's 3D model and immediately see which vehicle has damage, where on the vehicle it is, and what type it is, without walking the lot themselves.

---

## How the Three Parties Fit Together

Three parties contribute to the solution. Tri-Link connects them:

| Party | What they own |
|---|---|
| **Robot platform** | Robot hardware, autonomous navigation, SLAM-based 3D site model, cloud hosting of the 3D map |
| **Vision AI platform** | Damage detection model (trained on vehicle defect images), model serving infrastructure |
| **Tri-Link** | Binding layer: receives robot data + AI detections, resolves locations in 3D, writes the damage map |

For end-to-end deployment scenarios, case studies, and implementation priority, see [scenarios.md](scenarios.md).

---

## What Else This Approach Could Solve

The core pattern Tri-Link implements is general:

> A patrol robot captures images and pose. An AI model detects something in 2D. Tri-Link maps those detections to 3D world coordinates, so a human can find the problem in physical space.

Vehicle damage detection is the first application, but the same pipeline applies to any problem where you want to answer: *"Where exactly in a real-world space is this thing I detected in an image?"*

### Candidate future applications

**Infrastructure inspection**
A robot patrols a bridge, tunnel, or building facade. The AI model detects cracks, corrosion, or surface deterioration in concrete. Each defect is mapped to the structure's 3D model, giving maintenance teams precise repair locations without a manual inspection survey.

**Construction site progress monitoring**
A robot walks a construction site daily. The AI model detects deviations from the building plan — missing rebar, improperly poured concrete, misaligned structural elements. Defects are located in the 3D BIM (Building Information Model), giving site managers a daily updated view of what needs attention.

**Warehouse shelf and floor inspection**
A robot patrols warehouse aisles. The AI model detects damaged goods, misplaced items, floor cracks, or spills. Each finding is pinned to the warehouse floor plan, allowing operators to dispatch staff directly to the problem location rather than relying on reports or random checks.

**Industrial facility maintenance**
A robot patrols a factory floor or power plant. The AI model detects equipment anomalies — leaking pipes, worn conveyor belts, damaged insulation. The 3D facility model shows maintenance engineers exactly which piece of equipment requires attention.

### What would need to change

For each new application, only two things change:

1. **The AI model** — retrained for the new detection target (cracks, inventory issues, equipment faults). Tri-Link's `InferenceClient` is unchanged; it still sends JPEG and receives bounding boxes.
2. **The damage class labels** — the `Detection.class` field currently holds `"scratch"`, `"dent"`, `"crack"`. For a new domain these would be replaced with domain-specific labels. No structural code change is needed.

Everything else — temporal sync, unprojection math, pose buffer, JSONL/SQLite output — is domain-agnostic and reusable as-is.

---

## Open Source vs. Enterprise

The codebase splits naturally into two layers with different business value. The pattern follows the same model as the broader edgesentry organization: publish the generic infrastructure as commercially friendly open source to build trust and community, and capture enterprise value in the integration and operational layer.

### What should be open source (MIT / Apache-2.0)

These components solve general robotics and computer vision problems. They have no proprietary knowledge embedded and are more valuable as open source because community adoption drives quality.

| Component | Why open source |
|---|---|
| `buffer/` — `PoseBuffer` | Temporal synchronization of any time-series sensor data. Useful far beyond vehicle inspection. |
| `bridge/unproject.rs` — unprojection math | Standard pinhole camera geometry. No proprietary logic. Publishing it builds credibility. |
| Core types — `InspectionPacket`, `Detection`, `BBox2D`, `Transform4x4`, etc. | Generic data model for robot-vision fusion. Reusable as a shared schema. |
| `ingress/` — `RobotSource` trait + `MockSource` | Generic streaming interface. Any robotics project can implement it. |
| `error.rs` — `TriError` | Just error types; no value in keeping private. |

Publishing these as a crate gives other teams a foundation to build on and establishes a standard interface for robot-vision integration.

### What should be closed source (enterprise license)

These components encode the operational and integration knowledge that is hard to replicate without the specific partnerships and deployments. This is where the business moat lives.

| Component | Why enterprise |
|---|---|
| `infer/` — `InferenceClient` | Implements the specific API contract with the vision AI platform. Reflects negotiated integration knowledge. |
| `egress/semantic_map.rs` — damage map output | Output format and schema are designed to integrate with the robot platform's 3D viewer. This contract has value. |
| `main.rs` + config pipeline | Operational knowledge of how to wire the full system for real deployments. |
| Multi-site management, reporting, analytics | Future layers built on top of the core — dashboards, fleet-level damage trends, SLA reporting. |
| Deployment tooling, SLA, support | Enterprise service value delivered around the open-source core. |

### The principle

The open-source layer earns trust. The enterprise layer earns revenue. Neither undercuts the other: publishing the math and the buffer implementation does not give competitors the integration — they still need the partnerships, the deployment experience, and the operational tooling.

In the AI era, code itself has a shorter secrecy half-life — models and algorithms become widely known quickly. The durable moat is the integration depth with specific hardware and AI platforms, and the operational track record built on real deployments.
