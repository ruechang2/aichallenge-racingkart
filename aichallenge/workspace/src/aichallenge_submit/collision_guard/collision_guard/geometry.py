"""Path geometry for the collision guard.

Deliberately pure Python with no ROS imports, mirroring
``multi_purpose_mpc_ros.v2x_vehicle_tracker``: the arc projection is the part
most likely to hide a sign error, and keeping it importable on a bare host makes
it cheap to unit-test.
"""

import math


def project_onto_path(lon, lat, kappa):
    """Locate a point relative to the arc the ego is following.

    ``lon``/``lat`` are the point's coordinates in the ego frame (forward, left),
    ``kappa`` the signed path curvature (+ turning left).

    Returns ``(along, offset)``: distance travelled along the arc to the point of
    closest approach, and how far the point sits off that arc. With ``kappa == 0``
    this degenerates to the straight-ahead ``(lon, |lat|)``. ``along`` is negative
    when the point is behind the ego.
    """
    if abs(kappa) < 1e-4:
        return lon, abs(lat)

    # Instantaneous centre of rotation is at (0, R) in the ego frame, R signed
    # with +left. Mirror right-hand turns onto the left-hand case.
    radius = 1.0 / kappa
    sign = 1.0 if radius > 0.0 else -1.0
    r = radius * sign
    y = lat * sign

    d = math.hypot(lon, y - r)
    offset = abs(d - r)
    # Angle swept from the ego (which sits at -90 deg about the centre) to the
    # point, positive in the direction of travel.
    theta = math.atan2(lon, r - y)
    return r * theta, offset
