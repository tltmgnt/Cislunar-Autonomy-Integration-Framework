# Cislunar Autonomy Integration Framework

A dependency-free C++17 software-in-the-loop reference framework, with a C11-compatible API, for evaluating bounded spacecraft autonomy in an idealized Earth–Moon environment.

**Development baseline only.** This repository has no claimed flight heritage, flight qualification, SpaceX affiliation, or Government endorsement. It does not satisfy the RFI's existing flight-proven software requirement by itself. No SpaceX proprietary interfaces, pretrained flight models, or classified information are included.

## Build and run

Requirements: C++17 compiler, C compiler, Make; Python 3 for integration tests. No runtime network access or package downloads.

```sh
make test
./build/cislunar_sim --scenario outage --steps 2000 --output telemetry.csv
```

Or use CMake 3.16+:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
python3 tests/integration.py ./build/cislunar_sim
```

Scenarios: `nominal`, `outage`, `dropout`, `thermal`. Faults apply during the middle third of the run. `--help` lists CLI options. CSV records pre-propagation state, commanded acceleration, executive mode, cumulative commanded delta-v, and decision reason for every tick. Output paths are overwritten when explicitly supplied.

## Implemented flow

1. Obtain simulated navigation and health observations.
2. Check navigation validity, sensor age, battery, temperature, and primary/secondary keep-out distances.
3. Calculate a proportional/derivative correction to a supplied target state; optionally add an 18-weight linear model correction through the C++ API.
4. Reject non-finite control, clamp acceleration, enforce cumulative maneuver budget, and check the predicted endpoint against keep-out distances.
5. Issue a bounded command or zero thrust; record the reason and propagate using fourth-order Runge–Kutta.

Communication loss changes the mode to `disconnected` but does not require ground approval to continue bounded control. Safe mode means **zero commanded thrust only**; passive dynamics continue. It does not guarantee collision avoidance, thermal recovery, attitude control, or power survival. Recovery is automatic on the next healthy observation; no latched fault reset is implemented.

## Coordinate and time conventions

The propagator uses the circular restricted three-body problem (CR3BP), with Earth–Moon mass ratio 0.012150585609624. All state, acceleration, timestep, and budget values are normalized, in the barycentric rotating frame:

- Distance unit: Earth–Moon separation (approximately 384,400 km).
- Time unit: inverse Earth–Moon mean angular rate (approximately 375,200 seconds); a full circular orbit is 2π time units.
- Velocity unit: distance/time; acceleration unit: distance/time².
- State order: x, y, z, vx, vy, vz. Earth is at x = -mu; Moon at x = 1 - mu.

These conversions are illustrative; no ephemeris ingestion exists. The timestep is restricted to (0, 0.01]. CLI timestep is 0.0001. The fixed initial target is a tracking example, not a validated periodic orbit or trajectory optimizer. Solar gravity, radiation pressure, eccentricity, navigation uncertainty, propulsion mass depletion, and attitude dynamics are omitted.

## Interfaces and models

- `include/cislunar/core.hpp`: dynamics, executive, configuration, observations, decisions, and in-memory model lifecycle.
- `include/cislunar/c_api.h`: opaque controller creation/destruction and exception-contained step API for C callers. Mode values are documented in the header. Compile C clients with a C compiler and link the library with a C++ linker.
- `ModelStore`: stage, activate against a caller-supplied maximum error, and one-level rollback. Invalid staging clears pending activation. Active state is preserved after rejected updates.

No model is loaded by default. The optional linear component is a schema demonstration, not a pretrained SI capability. Validation scores are caller-provided metadata, not independently evaluated evidence. Model persistence, signed update transport, anti-replay protection, independent validation datasets, and atomic reboot-safe activation remain integration work.

Instances are single-owner and not thread-safe. The executive counts commanded acceleration magnitude × timestep, assuming commands are applied once. Production adapters must reconcile acknowledgments, missed execution, measured impulse, and retries. The C++ implementation uses strings, exceptions, and dynamic allocation; it is not hard-real-time or flight-certified.

## Validation

`make test` runs dynamics invariant checks, safety/budget gates, model lifecycle checks, a client compiled as C11, and actual CLI scenarios with CSV assertions. CI runs GCC and Clang builds plus AddressSanitizer/UndefinedBehaviorSanitizer. See [validation plan](docs/VALIDATION.md) and [integration boundaries](docs/INTEGRATION.md).
