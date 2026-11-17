#!/usr/bin/env bash
# Run once on each computer that launches the policy; no robot nodes are started.
set -eo pipefail
osp_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
osp_gh="$(command -v gh || true)"
[[ -n "$osp_gh" ]] || osp_gh="$HOME/.local/bin/gh"
"$osp_gh" auth status >/dev/null 2>&1 || { echo 'Run gh auth login on this computer first.' >&2; exit 1; }
"$osp_gh" api repos/peter010905/simul --jq '.permissions.push' | rg -q '^true$' || { echo 'GitHub repository write permission is required.' >&2; exit 1; }
mkdir -p "$HOME/.config/systemd/user" "$osp_repo_dir/run_logs"
/usr/bin/python3 - "$osp_repo_dir" "$HOME/.config/systemd/user" <<'PY'
import json
from datetime import datetime,timezone
from pathlib import Path
import sys,time
root,destination=map(Path,sys.argv[1:])
for name in ['osp-policy-log-sync.service','osp-policy-log-sync.timer']:
    text=(root/'scripts/systemd'/name).read_text().replace('%h/simul',str(root))
    (destination/name).write_text(text)
config=root/'run_logs/sync.json'
if not config.exists():
    config.write_text(json.dumps(dict(enabled_unix=time.time(),enabled_utc=datetime.now(timezone.utc).isoformat()),indent=2)+'\n')
PY
systemctl --user daemon-reload
systemctl --user enable --now osp-policy-log-sync.timer
if command -v loginctl >/dev/null; then
    loginctl --no-ask-password enable-linger "$(id -un)" || echo 'User session logout persistence was not enabled; configure systemd linger for background retries.' >&2
fi
echo 'Policy log retries enabled. Runs upload immediately at exit; pending/native logs sync every two minutes.'
