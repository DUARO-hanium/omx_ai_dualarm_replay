#!/usr/bin/env bash
set -euo pipefail
replay_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
replay_container=omx-ai-dualarm-replay
replay_image=omx-ai-dualarm-replay:1.1.0
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    cat <<'EOF'
Usage: ./replay.sh [TAKE] [--speed 1] [--align 10]
       ./replay.sh package_data --mock --speed 20 --align 1
       ./replay.sh package_data --inspect|--check
       ./replay.sh --status
       ./torque_off.sh
Replay CLI returns after playback; the final pose stays held until torque_off.sh.
Run another replay while holding to reuse the controllers without releasing torque.
TAKE defaults to package_data; local recordings belong in data/TAKE/.
EOF
    exit
fi
if ! command -v docker >/dev/null 2>&1; then
    echo 'Installing Docker from its official Ubuntu repository (sudo may be requested).'
    bash "$replay_root/install_docker.sh"
fi
docker_prefix=(docker)
if [[ -S /var/run/docker.sock && ! -w /var/run/docker.sock && $EUID -ne 0 ]]; then
    if [[ " $(id -nG "$(id -un)") " == *' docker '* ]]; then
        docker_prefix=(sg docker -c)
    else
        docker_prefix=(sudo docker)
    fi
fi
docker_call() {
    if [[ ${docker_prefix[0]} == sg ]]; then
        local replay_docker_command
        printf -v replay_docker_command '%q ' docker "$@"
        "${docker_prefix[@]}" "$replay_docker_command"
    else
        "${docker_prefix[@]}" "$@"
    fi
}
if ! docker_call info >/dev/null 2>&1; then
    if (( EUID == 0 )); then systemctl start docker; else sudo systemctl start docker; fi
    docker_call info >/dev/null
fi
if [[ ${1:-} == --status ]]; then
    docker_call exec "$replay_container" cat /results/status.json
    exit
fi
if [[ ${1:-} == --torque-off ]]; then
    shift
    echo 'Support both arms. Disabling arm and gripper torque.'
    if docker_call container inspect "$replay_container" >/dev/null 2>&1; then
        if [[ $(docker_call inspect --format '{{.State.Running}}' "$replay_container") == true ]]; then
            docker_call kill --signal USR1 "$replay_container" >/dev/null
        fi
        replay_exit=$(docker_call wait "$replay_container")
        docker_call logs --tail 12 "$replay_container"
        docker_call rm "$replay_container" >/dev/null
        if [[ $replay_exit == 0 ]]; then exit; fi
        echo 'Retrying direct torque OFF verification after controller shutdown.' >&2
    fi
    docker_call build -t "$replay_image" "$replay_root"
    docker_call run --rm --init --privileged --pid=host --network host -v /dev:/dev \
        -v "$replay_root/data:/data:ro" "$replay_image" --torque-off-only "$@"
    exit
fi
docker_call build -t "$replay_image" "$replay_root"
if [[ ${1:-} == --build-only ]]; then exit 0; fi
mkdir -p "$replay_root/results"
if [[ " $* " == *' --inspect '* || " $* " == *' --check '* ]]; then
    runtime_flags=()
    if [[ " $* " == *' --check '* && " $* " != *' --mock '* ]]; then
        runtime_flags+=(--privileged --pid=host -v /dev:/dev)
    fi
    docker_call run --rm --init "${runtime_flags[@]}" --network host --ipc host \
        -v "$replay_root/data:/data:ro" "$replay_image" "$@"
    exit
fi
exec {replay_lock}>"/tmp/omx-ai-dualarm-replay-$(id -u).lock"
flock -n "$replay_lock" || { echo 'Another replay CLI is running.' >&2; exit 1; }
replay_reuse=false
if docker_call container inspect "$replay_container" >/dev/null 2>&1; then
    if [[ $(docker_call inspect --format '{{.State.Running}}' "$replay_container") != true ]]; then
        echo 'Previous session stopped. Run ./torque_off.sh to verify shutdown first.' >&2
        exit 1
    fi
    replay_status=$(docker_call exec "$replay_container" cat /results/status.json)
    if [[ $replay_status != *'"protocol": 2'* ]]; then
        echo 'Active session uses an older version. Support the arms and run ./torque_off.sh once before using this version.' >&2
        exit 1
    fi
    replay_data_root=$(docker_call inspect --format '{{range .Mounts}}{{if eq .Destination "/data"}}{{.Source}}{{end}}{{end}}' "$replay_container")
    if [[ $replay_data_root != "$replay_root/data" ]]; then
        echo "Run the next replay from the active workspace: ${replay_data_root%/data}" >&2
        exit 1
    fi
    replay_reuse=true
fi
runtime_flags=()
if [[ " $* " != *' --mock '* ]]; then
    printf '%s\n' '양팔과 그리퍼가 움직입니다. 이동 경로를 비워 주세요.'
    read -r -p 'Enter: 양팔 재생 시작 (취소: Ctrl+C): ' replay_answer
    [[ -z $replay_answer ]] || { echo 'Cancelled; press Enter without text to start.'; exit; }
    runtime_flags+=(--privileged --pid=host -v /dev:/dev)
fi
replay_request="$(date +%s%N)-$$"
if [[ $replay_reuse == true ]]; then
    docker_call exec "$replay_container" /app/entrypoint.sh "$@" \
        --confirmed --enqueue --request-id "$replay_request"
else
    rm -f "$replay_root/results/status.json"
    docker_call run -d --name "$replay_container" --init "${runtime_flags[@]}" \
        --network host --ipc host -e "HOST_UID=$(id -u)" -e "HOST_GID=$(id -g)" \
        -v "$replay_root/data:/data:ro" -v "$replay_root/results:/results" \
        "$replay_image" "$@" --confirmed --request-id "$replay_request" >/dev/null
fi
interrupt_replay() {
    docker_call kill --signal INT "$replay_container" >/dev/null 2>&1 || true
    echo 'Replay interrupted; controllers remain active. Support the arms and run ./torque_off.sh.' >&2
    exit 130
}
trap interrupt_replay INT TERM
replay_phase=''
while true; do
    if [[ -f "$replay_root/results/status.json" ]]; then
        replay_status=$(< "$replay_root/results/status.json")
        if [[ $replay_status == *'"phase": "off"'* ]]; then
            echo '토크 OFF 및 제어 종료가 완료되었습니다.'
            exit 0
        fi
        if [[ $replay_status == *"\"request_id\": \"$replay_request\""* && $replay_status == *'"phase": "holding"'* ]]; then
            docker_call logs --tail 8 "$replay_container"
            echo '재생 종료: 마지막 자세에서 토크 유지 중입니다. 다음 재생: ./replay.sh TAKE / 해제: ./torque_off.sh'
            exit
        fi
        if [[ $replay_status == *'"phase": "holding_error"'* || $replay_status == *'"phase": "error"'* ]]; then
            docker_call logs --tail 20 "$replay_container"
            echo 'Replay failed. Inspect ./replay.sh --status; release with ./torque_off.sh.' >&2
            exit 1
        fi
        if [[ $replay_status == *"\"request_id\": \"$replay_request\""* && $replay_status == *'"phase": "replaying"'* && $replay_phase != replaying ]]; then
            echo '첫 기록 자세로 접근한 뒤 양팔 궤적을 재생 중입니다.'
            replay_phase=replaying
        fi
    fi
    if [[ $(docker_call inspect --format '{{.State.Running}}' "$replay_container") != true ]]; then
        docker_call logs --tail 20 "$replay_container"
        echo 'Replay server stopped. Check ./replay.sh --status and ./torque_off.sh.' >&2
        exit 1
    fi
    sleep 1
done
