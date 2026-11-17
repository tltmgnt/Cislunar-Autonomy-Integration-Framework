# Validation and qualification plan

## Executed software tests

- CR3BP Jacobi integral conservation over 10,000 unforced RK4 steps.
- Acceleration clamping and cumulative command budget exhaustion.
- Invalid timestep, non-finite navigation/model values, stale sensors, keep-out, battery and temperature gates.
- Ground outage continuation and zero-thrust fallback.
- Model activation rejection and one-level rollback.
- C11 compilation, C ABI mode/error behavior and pointer checks.
- CLI fault injection, recovery, finite CSV output, acceleration bounds and argument/output failure handling.

These are software checks in an idealized model, not proof of mission safety. Endpoint keep-out checks do not protect the swept trajectory between endpoints, enforce uncertainty margins, or guarantee viability after thrust inhibition.

## Required before flight integration

1. Specify mission orbits, operating envelope, uncertainty, hazard definitions, acceptable risk and command authority.
2. Replace illustrative targets with validated trajectories and verify dynamics against an independent ephemeris-based reference.
3. Use independent datasets for model evaluation; test out-of-distribution inputs, corruption, timing errors and adversarial cases.
4. Extend constraints to continuous/swept paths, navigation covariance, attitude, propulsion, power, thermal and communications resources.
5. Implement authenticated updates, anti-rollback policy where required, independent acceptance tests, persistent dual-image slots and reboot recovery.
6. Run processor-in-the-loop/hardware-in-the-loop timing, fault injection, radiation mitigation and resource/power measurements.
7. Establish spacecraft safe-state behavior, latched fault policy, command acknowledgments and independent fault containment.
8. Complete mission-specific qualification and traceability with authorized reviewers.

Keep evidence tied to source commit, executable hash, toolchain, processor, scenario, dataset and acceptance threshold. No flight-readiness claim should be derived solely from CI success.
