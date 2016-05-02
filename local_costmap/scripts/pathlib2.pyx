import numpy as np

cdef extern from "math.h":
    float cosf(float theta)
    float sinf(float theta)
    float acosf(float theta)

def arc_step_fast(double arc_radius, double travel_distance, double start_x, double start_y, double start_heading):
    """Fast version of arc_step."""
    cdef double x_o, y_o, heading_o
    cdef double radius, travel_angle
    cdef double x_1, y_1
    cdef double x, y, heading
    cdef double s, c
    # Step along the arc as if starting from the origin.
    # (inlined arc_step_origin)
    if arc_radius == 0.:
        # Special case for straight ahead.
        x_o, y_o, heading_o = travel_distance, 0., 0.
    else:
        radius = abs(arc_radius)
        # Angle of the turning circle spanned by the arc that the car takes.
        travel_angle = travel_distance / radius
        x_o = radius * sinf(travel_angle)
        y_o = radius * (1 - cosf(travel_angle))
        heading_o = travel_angle
        if arc_radius < 0:
            y_o *= -1
            heading_o *= -1

    # Rotate. (inlined rotate2d)
    s = sinf(start_heading)
    c = cosf(start_heading)
    x_1 = x_o * c - y_o * s
    y_1 = y_o * c + x_o * s

    # Translate.
    x, y, heading = x_1 + start_x, y_1 + start_y, heading_o + start_heading

    return (x, y, heading)
