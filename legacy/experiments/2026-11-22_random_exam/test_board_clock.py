import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from board_clock import AFTER, BEFORE, install


class BoardClockTest(unittest.TestCase):
    def test_physical_distance_and_final_sample_survive_overlapping_publications(self):
        code = 'class Run:\n' + AFTER + """
        if self.last is not None:
            self.distance += math.dist(xy, (self.last['x'], self.last['y']))
        self.last = sample
"""
        namespace = {'math': math}
        exec(code, namespace)
        view = namespace['Run']()
        view.first = view.last = None
        view.distance = 0.
        for t, x, state, wall in [(0, 0, 'running', 0), (1, 1, 'running', 2),
                                  (2, 2, 'running', 4), (1, 1, 'running', 3),
                                  (2, 2, 'running', None), (2, 4, 'FINISHED', None),
                                  (2, 2, 'running', 5)]:
            view.ingest(dict(elapsed=t, x=x, y=0, state=state, elapsed_wall=wall))
        self.assertEqual(view.distance, 2.)
        self.assertEqual((view.last['state'],view.last['x'],view.last['elapsed_wall']),
                         ('FINISHED',2,4))
        self.assertEqual(view.first['elapsed'], 0)
        view.ingest(dict(elapsed=math.nan,x=50,y=50))
        self.assertEqual(view.distance, 2.)

    def test_installer_recognizes_existing_reader_and_preserves_original(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp)/'sim_board.py'
            original = 'import math\nclass Run:\n'+BEFORE+'\n'
            path.write_text(original)
            self.assertEqual(install(path.parent), 'display reader updated')
            self.assertEqual(install(path.parent), 'already installed')
            self.assertEqual((path.parent/'sim_board.py.before_monotonic_display').read_text(), original)
            path.write_text('class DifferentViewer: pass\n')
            with self.assertRaises(ValueError):
                install(path.parent)
            self.assertEqual(path.read_text(),'class DifferentViewer: pass\n')


if __name__ == '__main__':
    unittest.main()
