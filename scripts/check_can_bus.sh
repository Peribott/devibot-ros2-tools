#!/usr/bin/env bash
# =============================================================================
# check_can_bus.sh
# Diagnose the CANopen bus used by devibot motor controllers and BMS.
# Requires: can-utils (apt install can-utils)
#
# Usage:
#   ./scripts/check_can_bus.sh [interface]
#   ./scripts/check_can_bus.sh can0
#   ./scripts/check_can_bus.sh can0 --monitor    # live candump for 5s
#
# Peribott Dynamic LLP — Hyderabad, India
# =============================================================================

set -euo pipefail

CAN_IFACE="${1:-can0}"
MONITOR=false
DURATION=5

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

for arg in "$@"; do
    case $arg in
        --monitor) MONITOR=true ;;
        --duration=*) DURATION="${arg#*=}" ;;
    esac
done

echo -e "\n${BOLD}${CYAN}devibot CAN Bus Diagnostics${NC}"
echo -e "Interface: ${CAN_IFACE}\n"

# ── Check can-utils available ─────────────────────────────────────────────────
if ! command -v candump &>/dev/null; then
    echo -e "${RED}ERROR: can-utils not installed.${NC}"
    echo "  sudo apt install can-utils"
    exit 1
fi

# ── Check interface exists ────────────────────────────────────────────────────
if ! ip link show "$CAN_IFACE" &>/dev/null; then
    echo -e "${RED}ERROR: Interface '$CAN_IFACE' not found.${NC}"
    echo ""
    echo "Available network interfaces:"
    ip link show | grep -E "^[0-9]+" | awk -F': ' '{print "  " $2}'
    echo ""
    echo "To bring up a CAN interface:"
    echo "  sudo ip link set $CAN_IFACE type can bitrate 500000"
    echo "  sudo ip link set $CAN_IFACE up"
    exit 1
fi

# ── Interface state ───────────────────────────────────────────────────────────
echo -e "${BOLD}Interface State${NC}"
CAN_STATE=$(ip -details link show "$CAN_IFACE" 2>/dev/null)
echo "$CAN_STATE" | head -4 | sed 's/^/  /'

# Check if interface is UP
if echo "$CAN_STATE" | grep -q "state UP"; then
    echo -e "\n  ${GREEN}✓${NC} Interface is UP"
else
    echo -e "\n  ${RED}✗${NC} Interface is DOWN"
    echo "    Run: sudo ip link set $CAN_IFACE up"
    exit 1
fi

# ── CAN error counters ────────────────────────────────────────────────────────
echo -e "\n${BOLD}Error Counters${NC}"
if ip -s link show "$CAN_IFACE" &>/dev/null; then
    ip -s link show "$CAN_IFACE" | grep -E "(RX|TX|errors|dropped)" | sed 's/^/  /'
fi

# ── Capture frames for analysis ───────────────────────────────────────────────
echo -e "\n${BOLD}Capturing frames (3 seconds)...${NC}"
TMPFILE=$(mktemp /tmp/candump.XXXXXX)
timeout 3 candump -t A "$CAN_IFACE" > "$TMPFILE" 2>/dev/null || true
FRAME_COUNT=$(wc -l < "$TMPFILE")
echo -e "  Captured ${FRAME_COUNT} frames\n"

if [[ $FRAME_COUNT -eq 0 ]]; then
    echo -e "  ${YELLOW}WARNING: No CAN frames received.${NC}"
    echo "  Check:"
    echo "  • CAN bus termination (120Ω at both ends)"
    echo "  • Bitrate matches devices (devibot: 500 kbit/s)"
    echo "  • Physical connections (CAN-H, CAN-L)"
    echo "  • Motor controller power"
    rm -f "$TMPFILE"
    exit 1
fi

# ── Identify active CAN IDs ───────────────────────────────────────────────────
echo -e "${BOLD}Active CAN IDs${NC}"

# Known devibot CAN IDs (CANopen standard)
declare -A KNOWN_IDS=(
    ["601"]="SDO Request  → Motor Controller 1 (Node 1)"
    ["581"]="SDO Response ← Motor Controller 1 (Node 1)"
    ["602"]="SDO Request  → Motor Controller 2 (Node 2)"
    ["582"]="SDO Response ← Motor Controller 2 (Node 2)"
    ["181"]="PDO1 TX ← Motor 1 (position/velocity)"
    ["182"]="PDO1 TX ← Motor 2 (position/velocity)"
    ["281"]="PDO2 TX ← Motor 1 (current/temp)"
    ["282"]="PDO2 TX ← Motor 2 (current/temp)"
    ["701"]="NMT Heartbeat ← Motor 1"
    ["702"]="NMT Heartbeat ← Motor 2"
    ["351"]="BMS → Pack voltage & current (Pylontech)"
    ["355"]="BMS → SoC & SoH"
    ["356"]="BMS → Cell temperatures"
    ["359"]="BMS → Status flags"
)

awk '{print $4}' "$TMPFILE" | sort | uniq -c | sort -rn | while read -r count can_id; do
    # Remove interface prefix if present (e.g. "can0#701" -> "701")
    id_clean="${can_id##*#}"
    id_clean="${id_clean%%#*}"
    id_upper="${id_clean^^}"

    if [[ -n "${KNOWN_IDS[$id_upper]+_}" ]]; then
        echo -e "  ${GREEN}${id_upper}${NC}  (${count}×) — ${KNOWN_IDS[$id_upper]}"
    else
        # Check if extended frame (29-bit)
        if [[ ${#id_clean} -gt 3 ]]; then
            echo -e "  ${CYAN}${id_upper}${NC}  (${count}×) — Extended frame (29-bit)"
        else
            echo -e "  ${YELLOW}${id_upper}${NC}  (${count}×) — Unknown device"
        fi
    fi
done

# ── Check expected devices ────────────────────────────────────────────────────
echo -e "\n${BOLD}Expected Device Check${NC}"

check_device() {
    local id="$1"
    local label="$2"
    if grep -qi "$id" "$TMPFILE"; then
        echo -e "  ${GREEN}✓${NC} $label"
    else
        echo -e "  ${RED}✗${NC} $label — no frames seen"
    fi
}

check_device "181\|701" "Motor Controller 1 (Node 1)"
check_device "182\|702" "Motor Controller 2 (Node 2)"
check_device "351\|355\|356" "Pylontech BMS"

# ── Bus load ──────────────────────────────────────────────────────────────────
echo -e "\n${BOLD}Bus Load${NC}"
FIRST_TS=$(head -1 "$TMPFILE" | awk '{print $1}' | tr -d '()')
LAST_TS=$(tail -1 "$TMPFILE" | awk '{print $1}' | tr -d '()')
if [[ -n "$FIRST_TS" && -n "$LAST_TS" ]]; then
    DURATION_ACTUAL=$(echo "$LAST_TS - $FIRST_TS" | bc 2>/dev/null || echo "3")
    if (( $(echo "$DURATION_ACTUAL > 0" | bc -l 2>/dev/null || echo 0) )); then
        RATE=$(echo "scale=1; $FRAME_COUNT / $DURATION_ACTUAL" | bc 2>/dev/null || echo "?")
        echo -e "  Frame rate: ${RATE} frames/second"
    fi
fi

# ── Live monitor ──────────────────────────────────────────────────────────────
if [[ "$MONITOR" == "true" ]]; then
    echo -e "\n${BOLD}Live Monitor (${DURATION}s — Ctrl+C to stop)${NC}"
    echo ""
    timeout "$DURATION" candump -t A "$CAN_IFACE" || true
fi

rm -f "$TMPFILE"
echo ""
echo -e "${GREEN}${BOLD}CAN bus diagnostics complete.${NC}"
