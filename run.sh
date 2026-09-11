#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -x "$PROJECT_ROOT/.conda/bin/python" ]]; then
    echo '尚未建立專案環境，請參考 COMMANDS.md。' >&2
    exit 1
fi
unset PYTHONPATH PYTHONHOME
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export PATH="$PROJECT_ROOT/.conda/bin:$PATH"
cd "$PROJECT_ROOT"
if [[ $# -eq 0 ]]; then
    set -- -m rgbd.gui
fi
exec "$PROJECT_ROOT/.conda/bin/python" "$@"
