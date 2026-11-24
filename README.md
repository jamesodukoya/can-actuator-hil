# can-actuator-hil

Steer-by-wire actuator software for a 1:10-scale front axle: a C++20 controller core written as step functions (dual-loop steering law, 10 kHz current loop, jam observer and safety monitors) on a fixed tick schedule; a CAN protocol with alive counter and CRC-8; and a pytest suite that steps a simulated axle in lockstep over Linux virtual CAN.

Status: Stage 1 (simulation tests).
Steer the axle model in your browser: https://daraodukoya.com/work/steer-by-wire/

## Plan

| Stage | What it adds |
|---|---|
| 1 | Simulation tests: the controller core, the CAN protocol and a lockstep SIL suite tagged by requirement |
| 2 | ECU firmware: the STM32G474 runtime, hand-written DMA drivers and the 10 kHz current loop |
| 3 | Axle build: the printed axle with its hub laser and wall scale, and the dual loop on hardware |
| 4 | Safety on hardware: the jam observer and safety monitors on the real axle |
| 5 | ROS 2: a `ros2_control` hardware interface over SocketCAN |
| 6 | HIL bench: an RP2040 pin-level bench and a self-hosted runner, so the same suite runs SIL and HIL |
| 7 | Measured model: system identification and a MuJoCo axle model scored on held-out runs |

## Build and test

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

## Layout

- `core/` controller core and CAN protocol: step functions with no heap or I/O
- `sim/` axle plant and sensor models for simulation
- `tools/` host tools (CAN test-vector printer)
- `tests/` C++ unit tests
- `dbc/` CAN message definitions
- `hil/` Python SIL tests (later pointed at the HIL bench)
- `docs/` requirements, analysis plan, lab notebook, failure log
- `scripts/`, `cmake/` CI, virtual CAN setup and build helpers

## Licence

MIT, see `LICENSE`.
