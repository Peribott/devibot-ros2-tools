#!/usr/bin/env bash
# =============================================================================
# log_viewer.sh
# Structured log viewer for devibot rotating log files.
#
# Usage:
#   ./scripts/log_viewer.sh [--filter LEVEL] [--file FILE] [--lines N] [--tail]
#
# Options:
#   --filter ERROR    Show only ERROR and above (DEBUG, INFO, WARN, ERROR)
#   --file bridge     View bridge.log (default: all logs)
#   --file api        View api.log
#   --lines 100       Show last N lines (default: 50)
#   --tail            Follow log in real time (like tail -f)
#   --stats           Show log statistics (error counts, sizes)
#
# Log locations:
#   /var/log/robot-dashboard/bridge.log  (ROS2 bridge)
#   /var/log/robot-dashboard/api.log     (FastAPI backend)
#
# Peribott Dynamic LLP — Hyderabad, India
# =============================================================================

set -euo pipefail

LOG_DIR="/var/log/robot-dashboard"
FILTER=""
FILE_ARG="all"
LINES=50
TAIL_MODE=false
STATS_MODE=false

RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
DIM='\033[2m'
BOLD='\033[1m'
NC='\033[0m'

# ── Parse arguments ───────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --filter)   FILTER="${2^^}"; shift 2 ;;
        --file)     FILE_ARG="$2"; shift 2 ;;
        --lines)    LINES="$2"; shift 2 ;;
        --tail)     TAIL_MODE=true; shift ;;
        --stats)    STATS_MODE=true; shift ;;
        --help)
            sed -n '3,25p' "$0" | sed 's/^# //'
            exit 0
            ;;
        *) shift ;;
    esac
done

# ── Check log directory ───────────────────────────────────────────────────────
if [[ ! -d "$LOG_DIR" ]]; then
    echo -e "${YELLOW}Log directory not found: $LOG_DIR${NC}"
    echo "The robot dashboard service may not be installed or has not run yet."
    echo "Check: systemctl status robot-dashboard"
    exit 1
fi

# ── Determine which log files to show ────────────────────────────────────────
declare -a LOG_FILES=()

case "$FILE_ARG" in
    bridge) LOG_FILES=("$LOG_DIR/bridge.log") ;;
    api)    LOG_FILES=("$LOG_DIR/api.log") ;;
    all)
        for f in "$LOG_DIR"/*.log; do
            [[ -f "$f" ]] && LOG_FILES+=("$f")
        done
        ;;
    *)
        # Treat as path
        if [[ -f "$FILE_ARG" ]]; then
            LOG_FILES=("$FILE_ARG")
        else
            echo -e "${RED}File not found: $FILE_ARG${NC}"
            exit 1
        fi
        ;;
esac

if [[ ${#LOG_FILES[@]} -eq 0 ]]; then
    echo -e "${YELLOW}No log files found in $LOG_DIR${NC}"
    exit 0
fi

# ── Stats mode ────────────────────────────────────────────────────────────────
if [[ "$STATS_MODE" == "true" ]]; then
    echo -e "\n${BOLD}${CYAN}devibot Log Statistics${NC}\n"
    for log_file in "${LOG_FILES[@]}"; do
        if [[ ! -f "$log_file" ]]; then continue; fi
        filename=$(basename "$log_file")
        filesize=$(du -sh "$log_file" | cut -f1)
        total_lines=$(wc -l < "$log_file")
        error_count=$(grep -c "ERROR\|CRITICAL\|FATAL" "$log_file" 2>/dev/null || echo 0)
        warn_count=$(grep -c "WARNING\|WARN" "$log_file" 2>/dev/null || echo 0)

        echo -e "${BOLD}${filename}${NC}"
        echo -e "  Size:     $filesize"
        echo -e "  Lines:    $total_lines"
        echo -e "  ${RED}Errors:   $error_count${NC}"
        echo -e "  ${YELLOW}Warnings: $warn_count${NC}"

        # Last error
        last_error=$(grep "ERROR\|CRITICAL" "$log_file" 2>/dev/null | tail -1)
        if [[ -n "$last_error" ]]; then
            echo -e "  Last error: ${DIM}${last_error:0:100}${NC}"
        fi
        echo ""
    done
    exit 0
fi

# ── Colour filter function ────────────────────────────────────────────────────
colourize() {
    while IFS= read -r line; do
        if [[ "$line" =~ ERROR|CRITICAL|FATAL ]]; then
            echo -e "${RED}${line}${NC}"
        elif [[ "$line" =~ WARNING|WARN ]]; then
            echo -e "${YELLOW}${line}${NC}"
        elif [[ "$line" =~ INFO ]]; then
            echo -e "${line}"
        elif [[ "$line" =~ DEBUG ]]; then
            echo -e "${DIM}${line}${NC}"
        else
            echo "$line"
        fi
    done
}

# ── Level filter ──────────────────────────────────────────────────────────────
level_pattern() {
    case "$FILTER" in
        ERROR)  echo "ERROR\|CRITICAL\|FATAL" ;;
        WARN)   echo "WARNING\|WARN\|ERROR\|CRITICAL\|FATAL" ;;
        INFO)   echo "INFO\|WARNING\|WARN\|ERROR\|CRITICAL\|FATAL" ;;
        DEBUG)  echo "" ;;  # Show all
        "")     echo "" ;;  # Show all
        *)      echo "$FILTER" ;;
    esac
}

PATTERN=$(level_pattern)

# ── Display logs ──────────────────────────────────────────────────────────────
echo -e "\n${BOLD}${CYAN}devibot Log Viewer${NC}"
[[ -n "$FILTER" ]] && echo -e "  Filter: ${YELLOW}$FILTER${NC} and above"
echo ""

for log_file in "${LOG_FILES[@]}"; do
    if [[ ! -f "$log_file" ]]; then continue; fi
    filename=$(basename "$log_file")
    echo -e "${BOLD}── $filename ──────────────────────────────────${NC}"

    if [[ "$TAIL_MODE" == "true" ]]; then
        echo -e "${DIM}(following — Ctrl+C to stop)${NC}\n"
        if [[ -n "$PATTERN" ]]; then
            tail -f "$log_file" | grep --line-buffered -E "$PATTERN" | colourize
        else
            tail -f "$log_file" | colourize
        fi
    else
        if [[ -n "$PATTERN" ]]; then
            tail -n "$LINES" "$log_file" | grep -E "$PATTERN" | colourize || true
        else
            tail -n "$LINES" "$log_file" | colourize
        fi
        echo ""
    fi
done
