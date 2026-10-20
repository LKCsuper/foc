"""Validate bounded I/F scheduling tests without claiming shaft-speed verification."""
import math
import re


def validate_blocked_if_reply(reply):
    match = re.search(r'rpm obs=([-\d.]+).*?tgt=([-\d.]+).*?iq=([-\d.]+).*?tk=(\d).*?fault=(\d+)', reply)
    if not match:
        raise ValueError('Incomplete scheduling-test safety reply')
    observer, target, current, takeover, fault = map(float, match.groups())
    if not all(math.isfinite(value) for value in (observer, target, current, takeover, fault)):
        raise ValueError('Nonfinite scheduling-test safety value')
    if target != 300 or fault or takeover or abs(current) > 3 or abs(observer) > 1200:
        raise ValueError('Scheduling-test safety check failed')
    return {'observer_rpm': observer, 'command_rpm': target, 'iq_a': current,
            'shaft_speed_verified': False, 'scope': 'I/F scheduling with takeover blocked; observer may be stopped'}
