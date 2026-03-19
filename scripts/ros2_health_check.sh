#!/usr/bin/env bash
# =============================================================================
# ros2_health_check.sh
# Verify all required devibot ROS2 nodes and topics are alive.
# Used in the AMR Boot System phase health checks (Phase 3 & 4).
#
# Usage:
#   ./scripts/ros2_health_check.sh [--quiet] [--json]
#
# Exit codes:
#   0  All checks passed
#   1  One or more checks failed
#   2  ROS2 environment not sourced
#
# Peribott Dynamic LLP — Hyderabad, India
# =============================================================================

set -euo pipefail

QUIET=false
JSON_OUTPUT=false
PASS=0
FAIL=0
RESULTS=()

# ── Colour codes ──────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ── Parse arguments ───────────────────────────────────────────────────────────
for arg in "$@"; do
    case $arg in
        --quiet)  QUIET=true ;;
        --json)   JSON_OUTPUT=true ;;
        --help)
            echo "Usage: $0 [--quiet] [--json]"
            echo "  --quiet   Suppress per-check output; only print summary"
            echo "  --json    Output results as JSON"
            exit 0
            ;;
    esac
done

# ── Check ROS2 environment ────────────────────────────────────────────────────
if ! command -v ros2 &>/dev/null; then
    echo -e "${RED}ERROR: ros2 command not found. Source your ROS2 workspace:${NC}"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  source ~/robot_ws/install/setup.bash"
    exit 2
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
    echo -e "${YELLOW}WARNING: ROS_DISTRO not set. Sourcing /opt/ros/jazzy/setup.bash${NC}"
    source /opt/ros/jazzy/setup.bash
fi

if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
    export ROS_DOMAIN_ID=0
fi

# ── Helper functions ──────────────────────────────────────────────────────────
check_pass() {
    local label="$1"
    PASS=$((PASS + 1))
    RESULTS+=("{\"check\":\"$label\",\"status\":\"PASS\"}")
    if [[ "$QUIET" == "false" && "$JSON_OUTPUT" == "false" ]]; then
        printf "  ${GREEN}✓${NC} %s\n" "$label"
    fi
}

check_fail() {
    local label="$1"
    local detail="${2:-}"
    FAIL=$((FAIL + 1))
    RESULTS+=("{\"check\":\"$label\",\"status\":\"FAIL\",\"detail\":\"$detail\"}")
    if [[ "$QUIET" == "false" && "$JSON_OUTPUT" == "false" ]]; then
        printf "  ${RED}✗${NC} %s"
        if [[ -n "$detail" ]]; then printf " — %s" "$detail"; fi
        printf "\n"
    fi
}

check_node() {
    local node_name="$1"
    if ros2 node list 2>/dev/null | grep -q "^${node_name}$"; then
        check_pass "Node: $node_name"
    else
        check_fail "Node: $node_name" "not running"
    fi
}

check_topic() {
    local topic_name="$1"
    local timeout_s="${2:-2}"
    # ros2 topic hz returns non-zero if no messages received within timeout
    if timeout "$timeout_s" ros2 topic hz "$topic_name" --window 1 &>/dev/null; then
        check_pass "Topic: $topic_name (active)"
    elif ros2 topic list 2>/dev/null | grep -q "^${topic_name}$"; then
        check_fail "Topic: $topic_name" "exists but no messages"
    else
        check_fail "Topic: $topic_name" "not published"
    fi
}

check_service() {
    local service_name="$1"
    if ros2 service list 2>/dev/null | grep -q "^${service_name}$"; then
        check_pass "Service: $service_name"
    else
        check_fail "Service: $service_name" "not available"
    fi
}

check_param() {
    local node="$1"
    local param="$2"
    if ros2 param get "$node" "$param" &>/dev/null; then
        check_pass "Param: $node/$param"
    else
        check_fail "Param: $node/$param" "not set"
    fi
}

# ── Run checks ────────────────────────────────────────────────────────────────
if [[ "$JSON_OUTPUT" == "false" ]]; then
    echo -e "\n${BOLD}${CYAN}devibot ROS2 Health Check${NC}"
    echo -e "  ROS_DISTRO:   ${ROS_DISTRO}"
    echo -e "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
    echo ""
    echo -e "${BOLD}Nodes${NC}"
fi

# Core navigation nodes
check_node "/nav2_bt_navigator"
check_node "/nav2_planner"
check_node "/nav2_controller"
check_node "/nav2_map_server"
check_node "/slam_toolbox"
check_node "/nav2_lifecycle_manager"

# Devibot-specific nodes
check_node "/devibot_bridge"
check_node "/map_manager"
check_node "/cloud_bridge"

if [[ "$JSON_OUTPUT" == "false" ]]; then echo -e "\n${BOLD}Topics${NC}"; fi

# Sensor topics (active — should be publishing)
check_topic "/scan"          3
check_topic "/odom"          3
check_topic "/imu/data"      3

# Navigation topics (may be latent at idle — just check existence)
if ros2 topic list 2>/dev/null | grep -q "^/map$"; then
    check_pass "Topic: /map (exists)"
else
    check_fail "Topic: /map" "not published"
fi

if ros2 topic list 2>/dev/null | grep -q "^/amcl_pose$"; then
    check_pass "Topic: /amcl_pose (exists)"
else
    check_fail "Topic: /amcl_pose" "not published"
fi

# Devibot internal topics
if ros2 topic list 2>/dev/null | grep -q "^/robot/status$"; then
    check_pass "Topic: /robot/status (exists)"
else
    check_fail "Topic: /robot/status" "not published"
fi

if [[ "$JSON_OUTPUT" == "false" ]]; then echo -e "\n${BOLD}Services${NC}"; fi

check_service "/map_manager/save_map"
check_service "/map_manager/load_map"
check_service "/map_manager/delete_map"

# ── Summary ───────────────────────────────────────────────────────────────────
TOTAL=$((PASS + FAIL))

if [[ "$JSON_OUTPUT" == "true" ]]; then
    echo "{"
    echo "  \"ros_distro\": \"${ROS_DISTRO}\","
    echo "  \"ros_domain_id\": ${ROS_DOMAIN_ID},"
    echo "  \"total\": ${TOTAL},"
    echo "  \"passed\": ${PASS},"
    echo "  \"failed\": ${FAIL},"
    echo "  \"status\": \"$([ $FAIL -eq 0 ] && echo 'OK' || echo 'DEGRADED')\","
    echo "  \"checks\": ["
    for i in "${!RESULTS[@]}"; do
        if [[ $i -lt $((${#RESULTS[@]} - 1)) ]]; then
            echo "    ${RESULTS[$i]},"
        else
            echo "    ${RESULTS[$i]}"
        fi
    done
    echo "  ]"
    echo "}"
else
    echo ""
    echo -e "${BOLD}Summary${NC}"
    if [[ $FAIL -eq 0 ]]; then
        echo -e "  ${GREEN}${BOLD}All ${TOTAL} checks passed.${NC}"
    else
        echo -e "  ${RED}${BOLD}${FAIL}/${TOTAL} checks failed.${NC}"
        echo ""
        echo -e "  Suggested actions:"
        echo -e "  • Check systemd service: ${CYAN}systemctl status robot-dashboard${NC}"
        echo -e "  • Check ROS2 logs:       ${CYAN}journalctl -u robot-dashboard -n 50${NC}"
        echo -e "  • Run log viewer:        ${CYAN}./scripts/log_viewer.sh --filter ERROR${NC}"
    fi
    echo ""
fi

exit $([[ $FAIL -eq 0 ]] && echo 0 || echo 1)
