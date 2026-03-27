#!/usr/bin/env bash
set -euo pipefail

DEV="${1:-lo0}"
MODE="${2:-apply}"

if [[ "$MODE" == "clear" ]]; then
  sudo pfctl -F all -f /etc/pf.conf >/dev/null
  echo "Cleared netem profile on ${DEV}"
  exit 0
fi

cat <<EOF >/tmp/swarm_dummynet.conf
dummynet in quick on ${DEV} proto udp from any to any pipe 1
dummynet out quick on ${DEV} proto udp from any to any pipe 1
EOF

sudo dnctl -q pipe 1 config delay 100ms plr 0.05
sudo pfctl -f /tmp/swarm_dummynet.conf >/dev/null
sudo pfctl -E >/dev/null

echo "Applied swarm profile on ${DEV}: 100ms latency, 5% packet loss"
