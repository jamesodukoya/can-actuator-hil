# Requirements

Stage 1 verifies these requirements in software-in-the-loop (SIL) against a plant model with estimated parameters. No SIL result describes the real axle; hardware verification is added per requirement in later stages. `scripts/write_traceability.py` reads the table: keep one row per ID and three columns.

| ID | Requirement | Method |
|---|---|---|
| REQ-CAN-001 | The ECU discards an `SBW_Command` whose length is not 8 or whose `Crc8` is not CRC-8/SAE-J1850 of bytes 0–6. Three consecutive such frames latch `FaultCrc` and turn torque off. | unit, SIL |
| REQ-CAN-002 | The ECU discards an `SBW_Command` whose `AliveCounter` equals the last accepted counter or jumps by 4–15 (mod 16); a jump also resynchronizes the reference. Three consecutive discards latch `FaultCounter` and turn torque off. | unit, SIL |
| REQ-CAN-003 | The byte layouts, scalings and ranges of `SBW_Command`, `SBW_Status`, `SBW_Heartbeat` and `SBW_Diag` in the C++ code match `dbc/steer_by_wire.dbc`. | unit, SIL |
| REQ-CAN-004 | The ECU sends `SBW_Status` every 5 ms and `SBW_Heartbeat` every 10 ms of ECU time, plus one extra `SBW_Status` in the tick a fault latches; its runtime sends `SBW_Diag` every 100 ms. | unit, SIL |
| REQ-MON-001 | While torque is enabled, if no command has been accepted for more than 50 ms, the ECU latches `FaultTimeout` and turns torque off. | unit, SIL |
| REQ-MON-002 | While torque is enabled, if the motor-encoder angle (at the kingpin) and the kingpin-encoder angle differ by more than 50 mrad for 10 consecutive 1 ms ticks, the ECU latches `FaultDisagree` and turns torque off. | unit, SIL |
| REQ-MON-003 | While torque is enabled, if the kingpin-encoder reading is stale (the read failed) for 3 consecutive 1 ms ticks, the ECU latches `FaultInternal` and turns torque off; until then it holds the last fresh angle. | unit, SIL |
| REQ-JAM-001 | In `DUAL_LOOP` with `CurrentLimit` of at least 1.5 A, a jam that stops the kingpin while it moves at 0.5 rad/s or more turns torque off and sends `SBW_Status` with `FaultJam` within 20 ms (FTTI). | unit, SIL |
| REQ-JAM-002 | No jam is declared during ±300 mrad steps or a 0.45 rad, 0.5 Hz sine sweep, on the stands or floor plant. | unit, SIL |
| REQ-CTL-001 | In `DUAL_LOOP`, after a centre command from ±300 mrad, the true kingpin angle stays within ±6 mrad of centre and varies by at most 3.07 mrad peak to peak over 2.0–2.5 s after the command, on the stands and floor plants. | unit, SIL |
| REQ-CUR-001 | With the motor locked, the 10 kHz current loop settles within 2 % of a 1 A step in at most 2 ms with at most 5 % overshoot at the nominal 1.5 mH winding inductance, and settles within 10 ms with at most 25 % overshoot at 0.5 mH and at 4 mH. | unit |
| REQ-CUR-002 | The measured motor current has the correct sign whether the current flows with or against the H-bridge direction (driving and braking), decoded from the drive-phase and brake-phase samples; below 6 % duty the last known direction is kept. In closed loop the plant current stays within 1.6 A while the current limit is 1.5 A. | unit |
| REQ-TIM-001 | The steering step's output takes effect exactly 2 current-loop ticks (200 µs) after its inputs are copied, and torque is never turned on at any other moment. One missed hand-off keeps the previous output; two consecutive misses turn torque off at the second and latch `FaultInternal` in the next tick. | unit, SIL |
| REQ-SAF-001 | The current command never exceeds `min(CurrentLimit, 2.5 A)` in magnitude and is 0 whenever torque is off. | unit |
| REQ-SAF-002 | A latched fault keeps torque off until a `CLEAR_FAULT` command is accepted while no disagreement condition is present and the latest kingpin reading is fresh; after clearing, the mode is `OFF` until a new mode command. | unit, SIL |
| REQ-SAF-003 | Every fault latch sets the active mode to `OFF` and disables the H-bridge in the same tick. | unit, SIL |
| REQ-SAF-004 | The controller clamps the target kingpin angle to ±0.5236 rad (±30°). | unit |
| CHAR-PLAY-001 | Model characterization: in `MOTOR_ONLY`, the centring error depends on the approach direction by at least the play half-width (13.33 mrad for the 1.2 mm slot), on both plants. | unit, SIL |
| CHAR-HUNT-001 | Model characterization: on the stands plant, `KINGPIN_ONLY` holds a limit cycle of at least 20 mrad peak to peak after centring, while `DUAL_LOOP` stays within 3.07 mrad. | unit, SIL |
