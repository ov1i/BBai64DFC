#!/bin/sh

IFACE="${1:-eth0}"
HOST="${2:-}"
TIMEOUT="${3:-0}"
echo "[dfc-wait-net] waiting for IPv4 on ${IFACE} ..."
start_ts=$(date +%s)
while :; do
  ip -4 addr show dev "${IFACE}" | grep -q "inet "
  if [ $? -eq 0 ]; then
    if [ -n "${HOST}" ]; then
      ping -c1 -W1 "${HOST}" >/dev/null 2>&1 && break
    else
      break
    fi
  fi
  if [ "${TIMEOUT}" -gt 0 ]; then
    now=$(date +%s)
    [ $((now - start_ts)) -ge "${TIMEOUT}" ] && exit 1
  fi
  sleep 1
done
echo "[dfc-wait-net] network ready on ${IFACE}"
exit 0
