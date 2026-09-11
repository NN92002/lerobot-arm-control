#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REMOTE_SSH="${LEROBOT_SERVER_SSH:-itri2026@140.114.58.2}"
REMOTE_DATASET_ROOT="${LEROBOT_SERVER_DATASET_PATH:-/home/itri2026/lerobot_datasets}"
REMOTE_PROJECT="${LEROBOT_SERVER_PROJECT_PATH:-/home/itri2026/lerobot-arm-control}"
REMOTE_CHECKPOINT_ROOT="${LEROBOT_SERVER_CHECKPOINT_PATH:-/home/itri2026/lerobot_checkpoints}"

usage() {
    cat <<'EOF'
Usage:
  ./server.sh sync <local-dataset> [remote-name]
  ./server.sh deploy
  ./server.sh train <remote-name> <checkpoint-name> [training options]

Environment overrides:
  LEROBOT_SERVER_SSH
  LEROBOT_SERVER_DATASET_PATH
  LEROBOT_SERVER_PROJECT_PATH
  LEROBOT_SERVER_CHECKPOINT_PATH
EOF
}

if [[ $# -lt 1 ]]; then
    usage >&2
    exit 2
fi

case "$1" in
    sync)
        if [[ $# -lt 2 || $# -gt 3 ]]; then
            usage >&2
            exit 2
        fi
        local_dataset="$(realpath "$2")"
        [[ -d "$local_dataset" ]] || { echo "Dataset directory not found: $local_dataset" >&2; exit 1; }
        remote_name="${3:-$(basename "$local_dataset")}"
        ssh "$REMOTE_SSH" "mkdir -p '$REMOTE_DATASET_ROOT/$remote_name'"
        rsync -av --partial --progress "$local_dataset/" \
            "$REMOTE_SSH:$REMOTE_DATASET_ROOT/$remote_name/"
        ;;
    deploy)
        rsync -av --delete \
            --exclude '.conda/' --exclude '.git/' --exclude '__pycache__/' \
            "$PROJECT_ROOT/" "$REMOTE_SSH:$REMOTE_PROJECT/"
        ;;
    train)
        if [[ $# -lt 3 ]]; then
            usage >&2
            exit 2
        fi
        remote_name="$2"
        checkpoint_name="$3"
        shift 3
        ssh "$REMOTE_SSH" "cd '$REMOTE_PROJECT' && ./run.sh -m rgbd.train --data '$REMOTE_DATASET_ROOT/$remote_name' --output '$REMOTE_CHECKPOINT_ROOT/$checkpoint_name' $*"
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac