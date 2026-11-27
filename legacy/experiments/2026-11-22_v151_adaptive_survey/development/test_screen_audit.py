"""Nonmotion checks for the development-only evidence and admission boundary."""
import contextlib
import importlib.util
import io
import sys
import unittest
from pathlib import Path
from types import ModuleType
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('dev_screen_audit',HERE/'screen_audit.py')
audit=importlib.util.module_from_spec(spec)
validate=ModuleType('validate')
validate.checksum=validate.identity=validate.measure=None
with patch.dict(sys.modules, {'validate':validate}):
    spec.loader.exec_module(audit)


class Admission(unittest.TestCase):
    def test_only_explicit_development_recordings(self):
        for group,seed,name in [('candidate',98765532,'unrelated'),
                                ('baseline',112601,'v151-survey-baseline-track-m112601-1122'),
                                ('candidate',112801,'v151-survey-candidate-heldout-m112801-private')]:
            with self.assertRaises(ValueError):
                audit.run_path(group,'R2',seed,name)

    def test_closed_on_incomplete_reversed_or_invalid_admission(self):
        for allowed in (True,False):
            with patch.object(sys,'argv',['audit','R2','--require-eligible']), \
                 patch.object(audit,'audit_revision',return_value={'allow_independent_dispatch':allowed}), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(audit.main(),0 if allowed else 1)

    def test_snapshot_cannot_admit(self):
        with patch.object(sys,'argv',['audit','R2','--require-eligible','--snapshot-first-two']), \
             contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            audit.main()


if __name__=='__main__':
    unittest.main()
