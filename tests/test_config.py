import pytest

from osp import config


def test_layers_merge_in_order_and_overrides_win():
    cfg = config.load('fast_sim', 'face_intent_v2', ['backend.settings.camera_width=640', 'mission.duration_s=300'])
    assert cfg['backend']['settings']['camera_width'] == 640
    assert cfg['backend']['settings']['seed_offset'] == 123456          # untouched backend layer
    assert cfg['mission'] == {'marker_count': 3, 'duration_s': 300}
    assert cfg['criteria']['approach_m'] == 1.5                         # base layer
    assert cfg['layers'][0].endswith('configs/base.yaml')


def test_policy_without_backend_implementation_is_rejected():
    with pytest.raises(config.ConfigError, match='no .gazebo. implementation'):
        config.load('gazebo', 'face_intent_v2')


def test_unknown_names_list_the_known_ones():
    with pytest.raises(config.ConfigError, match='known: .*mixed_v151'):
        config.load('fast_sim', 'nope')


def test_a_config_cannot_declare_itself_production():
    with pytest.raises(config.ConfigError, match='policy.status'):
        config.load('fast_sim', 'face_intent_v2', ['policy.status=production'])


@pytest.mark.parametrize('bad', ['criteria.approach_m=0', 'mission.marker_count=0', 'criteria.max_collisions=-1'])
def test_invalid_values_are_rejected(bad):
    with pytest.raises(config.ConfigError):
        config.load('fast_sim', 'face_intent_v2', [bad])


def test_every_policy_config_is_valid_for_each_backend_it_declares():
    import yaml
    for name in config.available('policies'):
        data = yaml.safe_load((config.CONFIGS / 'policies' / f'{name}.yaml').read_text())
        for backend in data['policy']['implementations']:
            config.load(backend, name)
