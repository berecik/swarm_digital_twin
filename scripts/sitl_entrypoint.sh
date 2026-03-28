#!/usr/bin/env bash
# sitl_entrypoint.sh — Start ArduPilot SITL with correct vehicle type and ports.
#
# Usage:
#   ./sitl_entrypoint.sh copter [--param-file /path/to/file.parm]
#   ./sitl_entrypoint.sh plane  [--param-file /path/to/file.parm]

set -euo pipefail

VEHICLE="${1:-copter}"
shift || true

PARAM_FILE=""
SIM_LAT="${SIM_LAT:--0.508333}"
SIM_LNG="${SIM_LNG:--78.141667}"
SIM_ALT="${SIM_ALT:-4500}"
INSTANCE="${INSTANCE:-0}"

# Parse optional args
while [[ $# -gt 0 ]]; do
    case $1 in
        --param-file) PARAM_FILE="$2"; shift 2 ;;
        *) shift ;;
    esac
done

echo "=== ArduPilot SITL ==="
echo "Vehicle:  $VEHICLE"
echo "Instance: $INSTANCE"
echo "Origin:   $SIM_LAT, $SIM_LNG, $SIM_ALT"

# Select binary and defaults
case "$VEHICLE" in
    copter|quad)
        BIN="arducopter"
        DEFAULTS="/ardupilot/Tools/autotest/default_params/copter.parm"
        FRAME="quad"
        ;;
    plane|fw)
        BIN="arduplane"
        DEFAULTS="/ardupilot/Tools/autotest/default_params/plane.parm"
        FRAME="plane"
        ;;
    *)
        echo "Unknown vehicle type: $VEHICLE (use copter or plane)"
        exit 1
        ;;
esac

# Combine defaults with custom param file if provided
if [ -n "$PARAM_FILE" ] && [ -f "$PARAM_FILE" ]; then
    DEFAULTS="$DEFAULTS,$PARAM_FILE"
    echo "Params:   $PARAM_FILE"
fi

# Build SITL arguments
SITL_ARGS=(
    --model "$FRAME"
    --home "$SIM_LAT,$SIM_LNG,$SIM_ALT,0"
    --instance "$INSTANCE"
    --defaults "$DEFAULTS"
    --base-port 5760
)

echo "Binary:   $BIN"
echo "Frame:    $FRAME"
echo ""

# Start SITL
exec "$BIN" "${SITL_ARGS[@]}"
