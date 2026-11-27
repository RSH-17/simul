import json

import pytest

from launch_official_local_grid_matrix import prepare


def test_interim_escape_and_unverified_local_grid_cannot_start_matrix(tmp_path):
    comparison=tmp_path/'screen';comparison.mkdir()
    for state in [dict(status='running',jobs=[dict(workspace='/frozen')]),
                  dict(status='completed',jobs=[dict(workspace='/frozen',start_screen_passed=True)]),
                  dict(status='completed',jobs=[dict(workspace='/frozen',navigation_grid_gate=True,start_screen_passed=False)])]:
        (comparison/'state.json').write_text(json.dumps(state))
        with pytest.raises(ValueError):
            prepare(tmp_path/'new',tmp_path/'matrix',tmp_path/'previous',comparison)
        assert not (tmp_path/'new').exists()
