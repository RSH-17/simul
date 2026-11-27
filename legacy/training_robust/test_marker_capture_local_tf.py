from types import SimpleNamespace as NS

import numpy as np
import pytest

from capture_marker_range_inputs_v2 import transform_payload


def transform():
    return NS(transform=NS(translation=NS(x=.1,y=.2,z=.3),rotation=NS(x=0.,y=0.,z=0.,w=1.)))


def test_delayed_map_transform_preserves_original_local_sensor_geometry():
    requested=[]
    def lookup(parent,child,stamp):
        requested.append((parent,child,stamp))
        if parent=='map':raise RuntimeError('Extrapolation 13 ms into the future')
        return transform()
    p=transform_payload(NS(lookup_transform=lookup),'scan','camera',123)
    assert p['map_transform_available'] is False
    assert 'map_from_camera_translation' not in p
    np.testing.assert_allclose(p['scan_from_camera_translation'],[.1,.2,.3])
    assert requested==[('scan','camera',123),('map','camera',123)]


def test_missing_local_transform_is_not_replaced_by_latest_or_identity():
    def lookup(*args):raise RuntimeError('Local geometry unavailable')
    with pytest.raises(RuntimeError,match='Local geometry'):
        transform_payload(NS(lookup_transform=lookup),'scan','camera',123)


def test_available_map_transform_is_preserved_as_optional_evidence():
    p=transform_payload(NS(lookup_transform=lambda *args:transform()),'scan','camera',123)
    assert p['map_transform_available'] is True
    np.testing.assert_allclose(p['map_from_camera_translation'],p['scan_from_camera_translation'])
