#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-up}"
IFACE="${2:-vcan0}"

if [[ "${MODE}" != "up" && "${MODE}" != "down" ]]; then
  echo "Usage: $0 [up|down] [iface]"
  exit 1
fi

if [[ "${MODE}" == "up" ]]; then
  sudo modprobe vcan
  if ! ip link show "${IFACE}" >/dev/null 2>&1; then
    sudo ip link add dev "${IFACE}" type vcan
  fi
  sudo ip link set up "${IFACE}"
  echo "[OK] ${IFACE} is up"
  exit 0
fi

if ip link show "${IFACE}" >/dev/null 2>&1; then
  sudo ip link set down "${IFACE}" || true
  sudo ip link delete "${IFACE}" type vcan || true
fi

echo "[OK] ${IFACE} is down"
