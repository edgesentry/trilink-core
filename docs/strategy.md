# Open Source vs. Enterprise

The codebase splits naturally into two layers with different business value. The pattern follows the same model as the broader edgesentry organization: publish the generic infrastructure as commercially friendly open source to build trust and community, and capture enterprise value in the integration and operational layer.

## What should be open source (MIT / Apache-2.0)

These components solve general robotics and computer vision problems. They have no proprietary knowledge embedded and are more valuable as open source because community adoption drives quality.

| Component | Why open source |
|---|---|
| `buffer/` — `PoseBuffer` | Temporal synchronization of any time-series sensor data. Useful far beyond vehicle inspection. |
| `bridge/unproject.rs` — unprojection math | Standard pinhole camera geometry. No proprietary logic. Publishing it builds credibility. |
| Core types — `InspectionPacket`, `Detection`, `BBox2D`, `Transform4x4`, etc. | Generic data model for robot-vision fusion. Reusable as a shared schema. |
| `ingress/` — `RobotSource` trait + `MockSource` | Generic streaming interface. Any robotics project can implement it. |
| `error.rs` — `TriError` | Just error types; no value in keeping private. |

Publishing these as a crate gives other teams a foundation to build on and establishes a standard interface for robot-vision integration.

## What should be closed source (enterprise license)

These components encode the operational and integration knowledge that is hard to replicate without the specific partnerships and deployments. This is where the business moat lives.

| Component | Why enterprise |
|---|---|
| `infer/` — `InferenceClient` | Implements the specific API contract with the vision AI platform. Reflects negotiated integration knowledge. |
| `egress/semantic_map.rs` — damage map output | Output format and schema are designed to integrate with the robot platform's 3D viewer. This contract has value. |
| `main.rs` + config pipeline | Operational knowledge of how to wire the full system for real deployments. |
| Multi-site management, reporting, analytics | Future layers built on top of the core — dashboards, fleet-level damage trends, SLA reporting. |
| Deployment tooling, SLA, support | Enterprise service value delivered around the open-source core. |

## The principle

The open-source layer earns trust. The enterprise layer earns revenue. Neither undercuts the other: publishing the math and the buffer implementation does not give competitors the integration — they still need the partnerships, the deployment experience, and the operational tooling.

In the AI era, code itself has a shorter secrecy half-life — models and algorithms become widely known quickly. The durable moat is the integration depth with specific hardware and AI platforms, and the operational track record built on real deployments.
