"""Package a team-sim policy candidate as a standalone TurtleBot4 run package.

The package has the same layout and commands as the teammate's
`osp-policy-v1.5.2.57.tar.gz` handoff (run.sh build/check/start/stop/status) so
it runs in the same robot-connected Ubuntu 24.04 + ROS 2 Jazzy environment.

The `runtime/` directory (robot_session.py, run_policy.sh, robot profiles and
the face_intent_v2 policy file) is copied unchanged from a checksum-verified
runtime package, so only the policy workspace differs from that package.
Compiled helpers (_raycast.so, export_slam_graph_poses) are not shipped: `run.sh
build` compiles them on the target, whatever its CPU architecture.

    python3 scripts/package_robot_policy.py --policy frontier-tour --version 1.5.7 \
        --runtime-package osp-policy-v1.5.2.57.tar.gz \
        --runtime-sha256 f959bde5... --out DIR
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

REPO = Path(__file__).resolve().parents[1]
SKIP_NAMES = {'__pycache__', 'build', 'install', 'log', '_raycast.so', 'export_slam_graph_poses'}
EPOCH = 1764201600  # fixed member mtime and owner so archive contents do not depend on the build host


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def runtime_from(package, expected, scratch):
    """Extract and verify the runtime/ directory of a published run package."""
    if sha256(package) != expected:
        raise SystemExit(f'{package}: SHA-256 mismatch')
    tmp = Path(tempfile.mkdtemp(prefix='runtime_', dir=scratch))
    with tarfile.open(package) as tar:
        tar.extractall(tmp, filter='data')
    (root,) = [p for p in tmp.iterdir() if p.is_dir()]
    sums = (root / 'SHA256SUMS').read_text().split('\n')
    for line in filter(None, sums):
        digest, name = line.split(None, 1)
        if sha256(root / name) != digest:
            raise SystemExit(f'{package}: {name} does not match its SHA256SUMS')
    return root / 'runtime', root.name


def build_workspace(policy, target):
    """Build a fresh workspace from committed candidate sources with team-sim setup."""
    subprocess.run(['bash', '-c', 'source /opt/ros/jazzy/setup.bash && exec python3 "$@"', '_',
                    str(REPO / 'tools/team_sim/setup.py'), str(target), '--policy', policy], check=True)


def ignore(_directory, names):
    return [n for n in names if n in SKIP_NAMES or n.endswith('.pyc')]


RUN_SH = r'''#!/bin/bash
# TurtleBot4 policy v@VERSION@ (@TITLE@) on an Ubuntu 24.04 / ROS 2 Jazzy
# computer wired to the robot network (192.168.185.x).
#   ./run.sh build    build the policy workspace once (compiles native helpers on this computer)
#   ./run.sh check    read-only readiness check (never moves the robot)
#   ./run.sh start    start a 600 s mission (the robot moves)
#   ./run.sh stop     stop: cancel actions, zero velocity, verify stationary odometry
#   ./run.sh status   read-only live state
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ws="$here/@WORKSPACE@"
repo="$here/runtime"
action="${1:-check}"
if (($#)); then shift; fi
ntp="${OSP_ROBOT_NTP_SERVER:-192.168.185.3}"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy가 없습니다 (/opt/ros/jazzy). README의 준비 항목을 확인하세요." >&2; exit 2
fi
set +u; source /opt/ros/jazzy/setup.bash; set -u

if [[ "$action" == build ]]; then
  pkg="$ws/src/osp_mission"
  c++ -std=c++17 -O3 -ffp-contract=off -fno-fast-math -shared -fPIC \
    "$pkg/osp_mission/raycast.cpp" -o "$pkg/osp_mission/_raycast.so"
  g++ -std=c++17 -O1 -I/opt/ros/jazzy/include -I/usr/include/eigen3 \
    "$pkg/tools/export_slam_graph_poses.cpp" -L/opt/ros/jazzy/lib -Wl,-rpath,/opt/ros/jazzy/lib \
    -lkartoSlamToolbox -lboost_serialization -lboost_thread -lpthread -o "$pkg/tools/export_slam_graph_poses"
  cd "$ws" && colcon build --symlink-install --packages-select osp_mission
  echo "빌드 완료. 다음: ./run.sh check"; exit 0
fi
if [[ ! -f "$ws/install/setup.bash" ]]; then
  echo "먼저 ./run.sh build 를 실행하세요." >&2; exit 2
fi

# FastDDS profile bound to this computer's address on the robot network.
ip="${OSP_ROBOT_LOCAL_IP:-$(ip -4 -o addr show 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | grep '^192\.168\.185\.' | head -1 || true)}"
if [[ -z "$ip" ]]; then
  echo "192.168.185.x 주소가 없습니다. 로봇 랜선을 연결하고 네트워크를 확인하세요 (또는 OSP_ROBOT_LOCAL_IP 지정)." >&2; exit 2
fi
profile="$here/fastdds_local.xml"
sed "s/192\.168\.185\.6/$ip/" "$repo/robot_profiles/wired_flow_v3/fastdds_vm.xml" > "$profile"

set +u; source "$ws/install/setup.bash"; set -u
export ROS_DOMAIN_ID=4 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE="$profile" \
       OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
session=(python3 -u "$repo/scripts/robot_session.py")
common=(--repo "$repo" --workspace "$ws" --domain 4 --ntp-server "$ntp")

case "$action" in
  check|status|stop)
    exec "${session[@]}" "$action" "${common[@]}" "$@" ;;
  start)
    exec bash "$repo/run_policy.sh" --mode robot --workspace "$ws" \
      --policy "$repo/policy_releases/2026-11-13_face_intent_v2/policy.json" -- \
      "${session[@]}" start "${common[@]}" --duration 600 "$@" ;;
  *)
    echo "사용법: ./run.sh {build|check|start|stop|status}" >&2; exit 2 ;;
esac
'''


README = '''# TurtleBot4 정책 v@VERSION@ 실행 패키지

@TITLE@ 정책이다. 후보 설명은 저장소의 `policy_candidates/@CANDIDATE@/README.md`에 있다.
`osp-policy-v1.5.2.57.tar.gz`와 같은 구성·같은 명령으로, 로봇에 연결한 Ubuntu 컴퓨터에서 빌드하고 실행한다.

## 준비

- Ubuntu 24.04와 ROS 2 Jazzy. 아래 패키지가 필요하다.
  - Nav2, slam_toolbox, robot_localization
  - TurtleBot4와 Create 3 메시지(`irobot_create_msgs`)
  - `rmw_fastrtps_cpp`
  - `python3-opencv`, `python3-scipy`
  - 빌드 도구: `build-essential`, `libeigen3-dev`, `libboost-serialization-dev`, `libboost-thread-dev`, `python3-colcon-common-extensions`
- 로봇과 랜선으로 연결해 이 컴퓨터가 `192.168.185.x` 주소를 받아야 한다. 로봇 쪽 NTP 서버는 `192.168.185.3`이다.
- 시계를 로봇과 맞춘다. 시작 점검이 카메라 시각 오차를 0.65초 이하로 제한한다(chrony 등으로 `192.168.185.3`에 동기화).

## 실행

```bash
sha256sum -c @PACKAGE@.tar.gz.sha256
tar xzf @PACKAGE@.tar.gz && cd @PACKAGE@
./run.sh build     # 처음 한 번. 이 컴퓨터에서 네이티브 도우미도 컴파일한다
./run.sh check     # 읽기 전용 점검. 로봇을 움직이지 않는다
./run.sh start     # 600초 미션 시작. 로봇이 움직인다
./run.sh stop      # 정지. 다른 터미널에서도 쓸 수 있다
```

- `check`에서 `"check": "passed"`가 나와야 출발할 수 있다.
- 로봇이 도크에 있으면 출발하지 않는다. 도크 밖 출발 위치로 옮긴다.
- `start`는 센서, SLAM, Nav2가 준비될 때까지 기다린 뒤 시작한다. Ctrl-C로도 정지한다.
- 주소를 직접 지정하려면 `OSP_ROBOT_LOCAL_IP=192.168.185.20 ./run.sh check`처럼 실행한다.

## 구성

- `@WORKSPACE@/`: 정책 작업공간. `src/osp_mission`, `frozen-config/`, 변경 매니페스트, `package_source.json`(만든 커밋과 출처).
- `runtime/`: `@RUNTIME_PACKAGE@`의 `runtime/`을 **바꾸지 않고** 그대로 넣었다. 시작·점검·정지 동작과 실물 Nav2 설정이 그 패키지와 같으므로 두 정책을 같은 조건에서 비교할 수 있다.
- `run.sh`: 위 구성으로 build, check, start, stop, status를 실행한다.

## v1.5.2.57 패키지와 다른 점

- 정책 작업공간만 다르다(v@VERSION@). SLAM 레이저 거리는 12 m다(.57은 5 m).
- 컴파일된 도우미(`_raycast.so`, `export_slam_graph_poses`)를 넣지 않고 `./run.sh build`에서 컴파일한다. 그래서 x86_64와 aarch64 모두에서 빌드된다.

## 검증 상태

@STATUS@
'''


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--policy', required=True, help='team-sim --policy name')
    p.add_argument('--version', required=True)
    p.add_argument('--title', required=True)
    p.add_argument('--candidate', required=True, help='policy_candidates/ directory name')
    p.add_argument('--status', required=True, type=Path, help='Markdown file with the verification status')
    p.add_argument('--runtime-package', required=True, type=Path)
    p.add_argument('--runtime-sha256', required=True)
    p.add_argument('--out', required=True, type=Path)
    a = p.parse_args()

    out = a.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    scratch = Path(tempfile.mkdtemp(prefix='.package_', dir=out))
    runtime, runtime_name = runtime_from(a.runtime_package, a.runtime_sha256, scratch)
    name = f'osp-policy-v{a.version}'
    workspace_name = 'v' + a.version.replace('.', '') + '_' + a.policy.replace('-', '_') + '_ws'
    root = out / name
    if root.exists():
        raise SystemExit(f'{root} exists. Choose a new output directory.')

    build = scratch / 'ws'
    build_workspace(a.policy, build)

    ws = root / workspace_name
    ws.mkdir(parents=True)
    shutil.copytree(build / 'src', ws / 'src', ignore=ignore, symlinks=False)
    shutil.copytree(build / 'frozen-config', ws / 'frozen-config')
    for manifest in sorted(build.glob('*_manifest.json')):
        shutil.copy2(manifest, ws / manifest.name)
    commit = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=REPO, capture_output=True, text=True,
                            check=True).stdout.strip()
    dirty = bool(subprocess.run(['git', 'status', '--porcelain', '--', 'policy_candidates', 'policy_releases',
                                 'tools/team_sim'], cwd=REPO, capture_output=True, text=True).stdout.strip())
    (ws / 'package_source.json').write_text(json.dumps(dict(
        version=a.version, team_sim_policy=a.policy, candidate=f'policy_candidates/{a.candidate}',
        repository='RSH-17/simul', commit=commit, sources_dirty=dirty,
        runtime_from=dict(package=runtime_name, sha256=a.runtime_sha256)), indent=2) + '\n')
    shutil.copytree(runtime, root / 'runtime')

    fill = {'@VERSION@': a.version, '@TITLE@': a.title, '@WORKSPACE@': workspace_name, '@PACKAGE@': name,
            '@CANDIDATE@': a.candidate, '@RUNTIME_PACKAGE@': runtime_name + '.tar.gz',
            '@STATUS@': a.status.read_text().strip()}
    def render(text):
        for key, value in fill.items():
            text = text.replace(key, value)
        return text
    (root / 'run.sh').write_text(render(RUN_SH))
    (root / 'run.sh').chmod(0o755)
    (root / 'README.md').write_text(render(README))

    files = sorted(f for f in root.rglob('*') if f.is_file())
    sums = ''.join(f'{sha256(f)}  ./{f.relative_to(root)}\n' for f in files)
    (root / 'SHA256SUMS').write_text(sums)

    archive = out / f'{name}.tar.gz'
    def normalize(info):
        info.uid = info.gid = 0
        info.uname = info.gname = ''
        info.mtime = EPOCH
        return info
    with tarfile.open(archive, 'w:gz') as tar:
        for path in [root] + sorted(root.rglob('*')):
            tar.add(path, arcname=str(path.relative_to(out)), recursive=False, filter=normalize)
    (out / f'{name}.tar.gz.sha256').write_text(f'{sha256(archive)}  {archive.name}\n')
    shutil.rmtree(scratch, ignore_errors=True)
    print(json.dumps(dict(archive=str(archive), sha256=sha256(archive), files=len(files),
                          commit=commit, sources_dirty=dirty), indent=2))


if __name__ == '__main__':
    main()
