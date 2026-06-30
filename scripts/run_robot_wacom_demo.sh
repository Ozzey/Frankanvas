#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MIOS_DIR="${PROJECT_ROOT}/mios"
DEMO_DIR="${PROJECT_ROOT}/Drawing_Demo/Visit/build"
WACOM_BRIDGE="${PROJECT_ROOT}/Drawing_Demo/scripts/wacom_udp_bridge.py"
RUN_ID="$(date +%Y%m%d-%H%M%S)"
LOG_DIR="${PROJECT_ROOT}/Drawing_Demo/logs/${RUN_ID}"

ROBOT_IP="${ROBOT_IP:-192.168.3.200}"
MIOS_HOST="${MIOS_HOST:-127.0.0.1}"
MIOS_PORT="${MIOS_PORT:-12000}"
LOCAL_IP="${LOCAL_IP:-192.168.3.1}"
START_POSE="${START_POSE:-0}"
WACOM_DEVICE_ID="${WACOM_DEVICE_ID:-}"
WACOM_DISPLAY="${WACOM_DISPLAY:-:0}"
WACOM_XAUTHORITY="${WACOM_XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"

MIOS_PID=""
DEMO_PID=""
WACOM_PID=""

die() {
    echo "error: $*" >&2
    exit 1
}

cleanup() {
    echo
    echo "Stopping demo processes..."
    if [[ -n "${WACOM_PID}" ]] && kill -0 "${WACOM_PID}" 2>/dev/null; then
        kill -INT "${WACOM_PID}" 2>/dev/null || true
    fi
    if [[ -n "${DEMO_PID}" ]] && kill -0 "${DEMO_PID}" 2>/dev/null; then
        kill -INT "${DEMO_PID}" 2>/dev/null || true
    fi
    sleep 1
    if [[ -n "${MIOS_PID}" ]] && kill -0 "${MIOS_PID}" 2>/dev/null; then
        kill -INT "${MIOS_PID}" 2>/dev/null || true
    fi
}
trap cleanup INT TERM EXIT

wait_for_tcp_port() {
    local port="$1"
    local label="$2"
    for _ in $(seq 1 120); do
        if ss -ltn 2>/dev/null | grep -q ":${port} "; then
            echo "${label} is listening on TCP ${port}"
            return 0
        fi
        sleep 0.5
    done
    return 1
}

wait_for_log() {
    local log_file="$1"
    local pattern="$2"
    local label="$3"
    for _ in $(seq 1 180); do
        if grep -q "${pattern}" "${log_file}" 2>/dev/null; then
            echo "${label}"
            return 0
        fi
        if [[ -n "${DEMO_PID}" ]] && ! kill -0 "${DEMO_PID}" 2>/dev/null; then
            return 1
        fi
        sleep 0.5
    done
    return 1
}

[[ -x "${MIOS_DIR}/mios/bin/mios.sh" ]] || die "MIOS wrapper not found at ${MIOS_DIR}/mios/bin/mios.sh"
[[ -x "${DEMO_DIR}/Drawing_Demo" ]] || die "Drawing_Demo binary not found at ${DEMO_DIR}/Drawing_Demo"
[[ -x "${WACOM_BRIDGE}" ]] || die "Wacom bridge not found at ${WACOM_BRIDGE}"
[[ "$(ulimit -r)" != "0" ]] || die "realtime priority is 0. Run from a fresh login shell after joining the realtime group."
systemctl is-active --quiet mongod || die "mongod is not active. Start it with: sudo systemctl start mongod"

if pgrep -f "${MIOS_DIR}/mios/bin/mios" >/dev/null; then
    die "MIOS already appears to be running. Stop it before using this launcher."
fi
if pgrep -f "${DEMO_DIR}/Drawing_Demo" >/dev/null; then
    die "Drawing_Demo already appears to be running. Stop it before using this launcher."
fi

mkdir -p "${LOG_DIR}" "${MIOS_DIR}/logs"
echo "Logs: ${LOG_DIR}"

echo "Starting MIOS..."
(
    cd "${MIOS_DIR}"
    desk=false \
    robot_ip="${ROBOT_IP}" \
    websocket_port="${MIOS_PORT}" \
    database_name=mios \
    verbosity=info \
    ./mios/bin/mios.sh
) >"${LOG_DIR}/mios.log" 2>&1 &
MIOS_PID="$!"

wait_for_tcp_port "${MIOS_PORT}" "MIOS" || die "MIOS did not open TCP ${MIOS_PORT}. See ${LOG_DIR}/mios.log"

echo "Starting Drawing_Demo..."
(
    cd "${DEMO_DIR}"
    tail -f /dev/null | ./Drawing_Demo \
        --robot-ip "${MIOS_HOST}" \
        --mios-port "${MIOS_PORT}" \
        --local-ip "${LOCAL_IP}" \
        --start-pose "${START_POSE}"
) >"${LOG_DIR}/drawing_demo.log" 2>&1 &
DEMO_PID="$!"

wait_for_log "${LOG_DIR}/drawing_demo.log" "Tablet UDP input server listening" "Drawing_Demo is listening for tablet UDP" \
    || die "Drawing_Demo did not reach tablet listening state. See ${LOG_DIR}/drawing_demo.log"

echo "Starting Wacom bridge..."
WACOM_ARGS=(--host "${MIOS_HOST}" --port 3333 --rate 60)
if [[ -n "${WACOM_DEVICE_ID}" ]]; then
    WACOM_ARGS+=(--device-id "${WACOM_DEVICE_ID}")
fi
DISPLAY="${WACOM_DISPLAY}" XAUTHORITY="${WACOM_XAUTHORITY}" \
python3 "${WACOM_BRIDGE}" \
    "${WACOM_ARGS[@]}" >"${LOG_DIR}/wacom.log" 2>&1 &
WACOM_PID="$!"

echo
echo "All processes are running."
echo "Use the Wacom pen now. Press Ctrl-C here to stop everything."
echo
tail -F "${LOG_DIR}/drawing_demo.log" "${LOG_DIR}/wacom.log" "${LOG_DIR}/mios.log" &
TAIL_PID="$!"
wait "${WACOM_PID}" || true
kill "${TAIL_PID}" 2>/dev/null || true
