import numpy

from tf import TransformListener
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped


def modernizePoint(listener, ps, static_frame='odom'):
    """
    :param TransformListener listener: The transform source
    :param PointStamped ps: The point to transform
    :param static_frame: The frame to use assume is static
    :return PointStamped:
    """
    legal_time = listener.getLatestCommonTime(static_frame, ps.header.frame_id)
    if legal_time <= ps.header.stamp:
        # Point is already modernized as well as possible
        return ps

    translation, rotation = listener.lookupTransformFull(ps.header.frame_id, ps.header.stamp,
                                                         ps.header.frame_id, legal_time, static_frame)

    mat44 = listener.fromTranslationRotation(translation, rotation)

    xyz = tuple(numpy.dot(mat44, numpy.array([ps.point.x, ps.point.y, ps.point.z, 1.0])))[:3]
    r = PointStamped(
        header=Header(seq=ps.header.seq, stamp=legal_time, frame_id=ps.header.frame_id),
        point=Point(*xyz))

    return r
