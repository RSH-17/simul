from rescreen import early_screen


def pair(seed,strict=False,bumps=0,unexpected=(),during=()):
    return dict(seed=seed,candidate=dict(strict=strict,bumps=bumps,unexpected_ids=unexpected,unexpected_ids_ever=during))


def test_futility_requires_one_unambiguous_win_and_no_safety_regression():
    assert not early_screen([pair(112601),pair(112605)])['eligible_to_continue']
    assert early_screen([pair(112601,True),pair(112605)])['eligible_to_continue']
    assert not early_screen([pair(112601,True),pair(112605,bumps=1)])['eligible_to_continue']
    assert not early_screen([pair(112601,True),pair(112605,during=[1023])])['eligible_to_continue']
    assert not early_screen([pair(112601,True,unexpected=[1023]),pair(112605)])['eligible_to_continue']


def test_incomplete_pair_is_not_a_futility_verdict():
    import pytest
    with pytest.raises(ValueError):
        early_screen([pair(112601)])
