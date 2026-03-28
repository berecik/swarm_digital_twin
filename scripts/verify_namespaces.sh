#!/usr/bin/env bash
# Verify Zenoh namespace isolation for the 6-drone swarm.
#
# Checks:
#   1. Each drone publishes on its own /swarm/drone_N/ namespace
#   2. No cross-namespace pollution (drone_1 not publishing on drone_2's topics)
#   3. Consensus topics follow /swarm/drone_N/consensus/raft_tx pattern
#   4. Bridge configs match their assigned DDS domain IDs
#
# Usage:
#   ./verify_namespaces.sh [config_dir]
#
# Author: beret <beret@hipisi.org.pl>
# Company: Marysia Software Limited <ceo@marysia.app>
set -euo pipefail

CONFIG_DIR="${1:-$(dirname "$0")/../docker/zenoh}"
DRONES=(1 2 3 4 5 6)
ERRORS=0

echo "=== Zenoh Namespace Verification ==="
echo "Config directory: ${CONFIG_DIR}"
echo ""

# ── Check 1: Drone configs have correct namespace ────────────────────────

echo "--- Check 1: Drone config namespaces ---"
for d in "${DRONES[@]}"; do
    config="${CONFIG_DIR}/drone_${d}.json5"
    if [[ ! -f "$config" ]]; then
        echo "  FAIL: ${config} not found"
        ERRORS=$((ERRORS + 1))
        continue
    fi

    ns=$(grep '"namespace"' "$config" | head -1 | sed 's/.*"\(\/swarm\/drone_[0-9]*\)".*/\1/')
    expected="/swarm/drone_${d}"
    if [[ "$ns" == "$expected" ]]; then
        echo "  OK: drone_${d} namespace = ${ns}"
    else
        echo "  FAIL: drone_${d} namespace = '${ns}', expected '${expected}'"
        ERRORS=$((ERRORS + 1))
    fi
done
echo ""

# ── Check 2: Publications are scoped to own drone ────────────────────────

echo "--- Check 2: Publication scope ---"
for d in "${DRONES[@]}"; do
    config="${CONFIG_DIR}/drone_${d}.json5"
    [[ -f "$config" ]] || continue

    # All allowed publications should contain drone_N (own ID)
    pubs=$(grep -A 10 '"publications"' "$config" | grep '/swarm/drone_' | grep -v "drone_${d}" || true)
    if [[ -z "$pubs" ]]; then
        echo "  OK: drone_${d} only publishes on /swarm/drone_${d}/"
    else
        echo "  FAIL: drone_${d} publishes on other namespaces:"
        echo "$pubs"
        ERRORS=$((ERRORS + 1))
    fi
done
echo ""

# ── Check 3: Consensus topics present ─────────────────────────────────────

echo "--- Check 3: Consensus topic presence ---"
for d in "${DRONES[@]}"; do
    config="${CONFIG_DIR}/drone_${d}.json5"
    [[ -f "$config" ]] || continue

    if grep -q "consensus/raft_tx" "$config"; then
        echo "  OK: drone_${d} has consensus/raft_tx"
    else
        echo "  FAIL: drone_${d} missing consensus/raft_tx"
        ERRORS=$((ERRORS + 1))
    fi
done
echo ""

# ── Check 4: Multicast disabled (Discovery Storm prevention) ─────────────

echo "--- Check 4: Multicast scouting disabled ---"
for d in "${DRONES[@]}"; do
    config="${CONFIG_DIR}/drone_${d}.json5"
    [[ -f "$config" ]] || continue

    if grep -q '"enabled": false' "$config"; then
        echo "  OK: drone_${d} multicast disabled"
    else
        echo "  WARN: drone_${d} may have multicast enabled"
        ERRORS=$((ERRORS + 1))
    fi
done
echo ""

# ── Check 5: Bridge configs match DDS domains ────────────────────────────

echo "--- Check 5: Bridge DDS domain isolation ---"
for d in "${DRONES[@]}"; do
    bridge="${CONFIG_DIR}/bridge_drone_${d}.json5"
    if [[ ! -f "$bridge" ]]; then
        echo "  FAIL: ${bridge} not found"
        ERRORS=$((ERRORS + 1))
        continue
    fi

    domain=$(grep '"domain"' "$bridge" | head -1 | sed 's/[^0-9]*//g')
    if [[ "$domain" == "$d" ]]; then
        echo "  OK: bridge_drone_${d} DDS domain = ${domain}"
    else
        echo "  FAIL: bridge_drone_${d} DDS domain = '${domain}', expected '${d}'"
        ERRORS=$((ERRORS + 1))
    fi
done
echo ""

# ── Summary ───────────────────────────────────────────────────────────────

if [[ $ERRORS -eq 0 ]]; then
    echo "=== ALL CHECKS PASSED ==="
    exit 0
else
    echo "=== ${ERRORS} CHECK(S) FAILED ==="
    exit 1
fi
