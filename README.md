# can-actuator-hil

Steer-by-wire actuator software for a 1:10-scale front axle: a C++20 controller core written as step functions (dual-loop steering law, 10 kHz current loop, jam observer and safety monitors) on a fixed tick schedule; a CAN protocol with alive counter and CRC-8; and a pytest suite that steps a simulated axle in lockstep over Linux virtual CAN. A pin-level hardware-in-the-loop bench comes later.

Status: Stage 1, software only. No result in this repository comes from hardware yet.

## Build and test

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

## Layout

- `core/` controller core, CAN protocol, plant and sensor model
- `sim/` host runtime and simulation-only frames
- `virtual_ecu/` virtual ECU on Linux virtual CAN
- `tests/` C++ unit tests
- `dbc/` CAN message definitions
- `hil/` Python SIL tests (later pointed at the hardware bench)
- `docs/` requirements, analysis plan, lab notebook, failure log
- `hardware/` axle CAD, carrier board and assembly guide
