"""Simulator slots keep at most two Gazebo benchmarks on the host, one per ROS domain."""
import sys
import tempfile
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import slots  # noqa: E402

TEAM = lambda run, domain: ({'BENCH_RUN': run, 'ROS_DOMAIN_ID': domain}, ['python3'])
GZ_SERVER = ({}, ['gz', 'sim', '-s', '-r', 'world.sdf'])
GZ_GUI = ({}, ['gz', 'sim', '-g', '-v', '2'])


class SlotTest(unittest.TestCase):
    def test_counts_each_run_once_and_ignores_gui(self):
        processes = [TEAM('/r/a', '174'), TEAM('/r/a', '174'), GZ_GUI, ({'ROS_DOMAIN_ID': '174'}, ['ros2'])]
        self.assertEqual(slots.active_benchmarks(processes), {'run:/r/a': '174'})
        self.assertEqual(len(slots.active_benchmarks(processes+[GZ_SERVER])), 2)

    def test_picks_the_free_domain(self):
        with tempfile.TemporaryDirectory() as tmp:
            with slots.acquire(tmp, processes=lambda: [TEAM('/r/a', '174')]) as slot:
                self.assertEqual((slot.domain, slot.concurrent), (175, 2))

    def test_refuses_a_third_benchmark(self):
        with tempfile.TemporaryDirectory() as tmp:
            busy = lambda: [TEAM('/r/a', '174'), TEAM('/r/b', '175')]
            with self.assertRaises(SystemExit):
                with slots.acquire(tmp, processes=busy):
                    pass
            # A Gazebo server started by hand also uses up a place.
            with self.assertRaises(SystemExit):
                with slots.acquire(tmp, processes=lambda: [TEAM('/r/a', '174'), GZ_SERVER]):
                    pass

    def test_refuses_a_requested_domain_in_use(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(SystemExit):
                with slots.acquire(tmp, 174, processes=lambda: [TEAM('/r/a', '174')]):
                    pass

    def test_locks_keep_two_starts_apart(self):
        """Two runs in one installation get different domains even before they appear as processes."""
        with tempfile.TemporaryDirectory() as tmp:
            with slots.acquire(tmp, processes=lambda: []) as first:
                first.admitted()
                with slots.acquire(tmp, processes=lambda: []) as second:
                    self.assertEqual({first.domain, second.domain}, {174, 175})
                    second.admitted()
                    with self.assertRaises(SystemExit):
                        with slots.acquire(tmp, processes=lambda: []):
                            pass

    def test_admission_waits_until_the_first_run_is_visible(self):
        with tempfile.TemporaryDirectory() as tmp:
            order = []
            with slots.acquire(tmp, processes=lambda: []) as first:
                def second():
                    with slots.acquire(tmp, processes=lambda: []):
                        order.append('second admitted')
                thread = threading.Thread(target=second)
                thread.start()
                thread.join(0.3)
                order.append('first visible')
                first.admitted()
                thread.join(5)
            self.assertEqual(order, ['first visible', 'second admitted'])


if __name__ == '__main__':
    unittest.main()
