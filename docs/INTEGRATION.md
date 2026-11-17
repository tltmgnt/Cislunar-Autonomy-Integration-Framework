# Integration boundaries

## Ownership

The Government spacecraft team supplies navigation, clocks, health telemetry, command execution, and mission-approved target states and constraints. The autonomy integrator supplies algorithms, adapters, resource measurements, validation artifacts, and configuration management. A prospective launch or hosted-payload provider supplies its own agreed interfaces. No external provider is integrated in this baseline.

## SpaceX collaboration contract

Potential SpaceX participation requires an actual agreement and documented interfaces; the repository makes no claim that reusable SpaceX autonomy software or cislunar network coverage is available. Keep launch and hosting contracts separate from the software API.

An eventual adapter must define:

| Boundary | Required information |
|---|---|
| Navigation | Reference frame, units, timestamps, covariance, validity, source identity |
| Command execution | Thrust frame, impulse limits, authorization, sequence IDs, expiry, acknowledgments |
| Compute | CPU/OS/ABI, scheduler, storage, RAM, power and thermal limits |
| Communications | Availability, latency, authenticated uplink, disconnected behavior |
| Demonstration | Advisory or closed-loop authority, telemetry access, abort authority, qualification evidence |
| Model update | Signed manifest, hash, compatible baseline, validation results, rollback slot |

Normalize all values before entering this API. Perform reference-frame conversion and timestamp validation in the spacecraft adapter. Do not connect demonstration outputs directly to flight actuators.

## Resource reporting

No hosting resource or power claim has been measured on a flight processor. The optional model has 18 double-precision weights (144 bytes), plus version and validation metadata. This is not a total runtime RAM measurement. The simulator logs to a host filesystem; flight adapters need bounded telemetry queues. Processor throughput, worst-case execution time, memory allocation, average/peak power, and radiation behavior require platform measurement.

## RFI coverage

| Area | Baseline support | Remaining evidence/work |
|---|---|---|
| Mission management | Per-tick health/communications mode handling | Mission sequencing, subsystem recovery and certified CONOPS |
| Fault isolation | Explicit invalid/stale and health gates | Diagnostic inference, isolation and ambiguity handling |
| Trajectory | CR3BP simulation, bounded target tracking | Optimization, orbit targeting and flight validation |
| Computer vision | None | Qualified perception product and sensors |
| SI/ML | Optional linear correction interface | Existing trained model and applicable flight heritage |
| V&V | Unit, invariant, C ABI and CLI integration tests | HIL, Monte Carlo, independent review and qualification |
| Updates | In-memory staging and rollback | Secure persistent update pipeline and onboard validation |
| Schedule/cost | No unsupported commitments | Product owner estimates and Government hardware baseline |

The 90-day and 18-month RFI objectives cannot be established from this new implementation. They require an existing qualified product baseline and a funded, platform-specific integration plan.
