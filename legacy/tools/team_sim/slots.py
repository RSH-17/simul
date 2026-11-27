"""Simulator slots: at most two Gazebo benchmarks on this host, one per ROS domain.

Each team run holds one per-domain lock for its whole life. While it counts the
benchmarks already running, it also holds a short admission lock, and it keeps
that lock until its own processes exist (`Slot.admitted()`). This stops two runs
that start at the same moment from both passing the host-wide limit.
"""
import contextlib
import fcntl
from pathlib import Path

DOMAINS = (174, 175)            # automatic slots; 173 stays available by explicit --domain
BENCH_DOMAINS = ('173', '174', '175')
HOST_LIMIT = 2                  # AGENTS.md: at most two native Gazebo benchmarks at once


def is_gazebo_server(cmdline):
    """`gz sim` that runs a world. A GUI-only client (`gz sim -g`) is not a benchmark."""
    if len(cmdline) < 2 or Path(cmdline[0]).name not in ('gz', 'ruby') or 'sim' not in cmdline[:3]:
        return False
    return '-g' not in cmdline


def active_benchmarks(processes):
    """Distinct running benchmarks from (environ, cmdline) pairs.

    Returns {key: domain}. A team or native benchmark is keyed by BENCH_RUN. A Gazebo
    world server started outside the benchmark tools counts on its own."""
    found = {}
    for index, (environ, cmdline) in enumerate(processes):
        run, domain = environ.get('BENCH_RUN'), environ.get('ROS_DOMAIN_ID')
        if run and domain in BENCH_DOMAINS:
            found.setdefault('run:'+run, domain)
        elif not run and is_gazebo_server(cmdline):
            found['gz:'+str(index)] = domain
    return found


def host_processes():
    import psutil
    for process in psutil.process_iter():
        try:
            yield process.environ(), process.cmdline()
        except psutil.Error:
            pass


def _try_lock(path):
    handle = path.open('a')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return handle
    except BlockingIOError:
        handle.close()
        return None


class Slot:
    def __init__(self, domain, concurrent, admission):
        self.domain = domain
        self.concurrent = concurrent        # benchmarks running when this one was admitted, itself included
        self._admission = admission

    def admitted(self):
        """Call once this run's own processes exist (they carry BENCH_RUN)."""
        if self._admission:
            self._admission.close()
            self._admission = None


@contextlib.contextmanager
def acquire(root, requested=None, processes=host_processes, limit=HOST_LIMIT):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    admission = (root/'simulator-admission.lock').open('a')
    fcntl.flock(admission, fcntl.LOCK_EX)
    slot_lock = None
    try:
        running = active_benchmarks(processes())
        if len(running) >= limit:
            raise SystemExit(f'{len(running)} Gazebo benchmarks are already running on this host '
                             f'(limit {limit}); wait for one to finish.')
        busy = set(running.values())
        for domain in ([requested] if requested else DOMAINS):
            if str(domain) in busy:
                continue
            slot_lock = _try_lock(root/f'simulator-domain-{domain}.lock')
            if slot_lock:
                break
        if not slot_lock:
            wanted = f'domain {requested}' if requested else 'a free simulator slot'
            raise SystemExit(f'No {wanted}: another simulation is using it.')
        slot = Slot(domain, len(running)+1, admission)
        admission = None
        yield slot
    finally:
        if admission:
            admission.close()
        if slot_lock:
            slot.admitted()
            slot_lock.close()
