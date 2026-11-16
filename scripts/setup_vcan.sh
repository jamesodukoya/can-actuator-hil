#!/usr/bin/env bash
# Creates the virtual CAN interface vcan0 used by the SIL suite. Run with sudo; safe to rerun.
set -euo pipefail
modprobe vcan
if ! ip link show vcan0 > /dev/null 2>&1; then
  ip link add dev vcan0 type vcan
fi
# A lockstep step of 1,000 ticks sends about 1,300 frames at once; a longer queue avoids ENOBUFS.
ip link set vcan0 txqueuelen 1000
ip link set up vcan0
