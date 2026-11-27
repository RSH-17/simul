"""Install a presentation-only fix for two producers of the same 8780 trace."""
import argparse
import os
from pathlib import Path
import shutil
import tempfile


BEFORE = """    def ingest(self, sample):
        if self.first is None:
            self.first = sample
        t = float(sample['elapsed'])
        xy = (float(sample['x']), float(sample['y']))"""
AFTER = """    def ingest(self, sample):
        t = float(sample['elapsed'])
        if not math.isfinite(t):
            return
        if self.last is not None:
            previous = float(self.last['elapsed'])
            if t < previous:
                return
            if t == previous:
                # The camera observer and board relay can publish the same
                # physical sample. A final-state or timing update is still useful.
                if self.last.get('state') == 'FINISHED' and sample.get('state') != 'FINISHED':
                    return
                merged = {**self.last, **sample, 'x': self.last['x'], 'y': self.last['y']}
                if sample.get('elapsed_wall') is None and 'elapsed_wall' in self.last:
                    merged['elapsed_wall'] = self.last['elapsed_wall']
                if merged == self.last:
                    return
                sample = merged
        if self.first is None:
            self.first = sample
        xy = (float(sample['x']), float(sample['y']))"""


def install(directory):
    path = directory / 'sim_board.py'
    original = path.read_text()
    if original.count(AFTER) == 1:
        return 'already installed'
    if original.count(BEFORE) != 1:
        raise ValueError('Unrecognized sim_board.Run.ingest, leaving the viewer as found')
    changed = original.replace(BEFORE, AFTER)
    compile(changed, str(path), 'exec')
    backup = path.with_name('sim_board.py.before_monotonic_display')
    if backup.exists():
        if backup.read_text() != original:
            raise ValueError('The existing backup belongs to a different viewer')
    else:
        shutil.copy2(path, backup)
    with tempfile.NamedTemporaryFile(dir=directory, mode='w', delete=False) as out:
        temporary = Path(out.name)
        out.write(changed)
    try:
        shutil.copymode(path, temporary)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
    return 'display reader updated'


if __name__ == '__main__':
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('directory', type=Path, help='Existing 8780 dashboard containing sim_board.py')
    print(install(cli.parse_args().directory))
