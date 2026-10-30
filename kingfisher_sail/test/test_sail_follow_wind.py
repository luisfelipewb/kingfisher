from math import pi

from kingfisher_sail.sail_follow_wind import feather_angle
import pytest


@pytest.mark.parametrize('vx, vy, expected', [
    # From ahead: nose forward
    (-5.0, 0.0, 0.0),
    # From starboard, going to port: nose to starboard
    (0.0, 5.0, -pi / 2),
    # From port
    (0.0, -5.0, pi / 2),
    # From the starboard bow
    (-5.0, 5.0, -pi / 4),
])
def test_feathered(vx, vy, expected):
    assert feather_angle(vx, vy) == pytest.approx(expected)


def test_from_astern():
    assert abs(feather_angle(5.0, 0.0)) == pytest.approx(pi)


def test_angle_of_attack_turns_clockwise():
    assert feather_angle(-5.0, 0.0, 0.3) == pytest.approx(-0.3)


def test_wrapped():
    # From port quarter, with an angle of attack past -pi
    assert feather_angle(5.0, -5.0, -1.0) == pytest.approx(3 * pi / 4 + 1.0 - 2 * pi)
