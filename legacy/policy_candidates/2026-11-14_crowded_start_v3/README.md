# Crowded start v3: simulation navigation transport

This retains v2 incremental transit and scopes `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` to the simulated Nav2 processes. The first v2 trial failed before mission start: the controller configure response timed out and navigation stayed unavailable. That infrastructure failure is preserved separately from policy results. The transport change does not prove that shared memory was the sole cause.

The candidate build passed all 382 runtime tests. The native supervisor additionally checks seven actual navigation processes for the UDP setting and absence of Fast DDS shared-memory threads. Image processes and real-hardware launch retain their previous transport settings.

The first 55 cm native trial started and moved about 1.1 m with no contact, but then its map clearance fell to 0.20 m and no further safe goals were produced. **Successful departure is not successful corridor escape. Full native acceptance is pending/unsatisfied; no production or hardware promotion.** Further work must address initial turning, continuing short transit, and actuator-path readiness. Existing and queued runtimes remain frozen.

Build and compare in new paths:

```bash
python3 policy_candidates/2026-11-14_crowded_start_v3/build.py --workspace /absolute/new/workspace --test
python3 training_robust/run_crowded_start_regression_v3.py --output /absolute/new/results --candidate /absolute/candidate/workspace --baseline /absolute/baseline/workspace
```
