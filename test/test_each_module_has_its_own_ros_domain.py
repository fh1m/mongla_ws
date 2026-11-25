"""The root conftest puts each test module in its own ROS domain, never 0."""
import os

_SEEN = {}


def test_this_module_is_off_the_vehicles_domain():
    d = os.environ.get('ROS_DOMAIN_ID')
    assert d is not None and 1 <= int(d) <= 99
    _SEEN['a'] = d


def test_and_every_test_in_it_shares_one():
    assert os.environ.get('ROS_DOMAIN_ID') == _SEEN['a']
