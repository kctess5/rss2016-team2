cdef extern from "math.h":
    float cosf(float theta)
    float sinf(float theta)
    float acosf(float theta)
    float sqrtf(float x)

def calc_range(map_info, omap, _x, _y, _heading, max_range):
    """
    Calculate the expected laser reading at a given point and angle.
    Do this by walking along the ray until you reach something that the map thinks is filled.

    Copied from https://bitbucket.org/alexbuyval/ardroneum
    """

    cdef float robot_x, robot_y, robot_a
    cdef int x0, y0, x1, y1, deltax, deltay, error, deltaerr, x, y, xstep, ystep

    robot_x, robot_y, robot_a = _x, _y, _heading

    # Threshold value. Above this probability, the cell is expected filled.
    cdef float filled_threshold = 0.5

    # given the robot's pose in the 'map' frame, compute the corresponding index in
    # the occupancy grid map
    def map_to_grid(map_x, map_y):
        grid_x = int((map_x - map_info.origin.position.x) / map_info.resolution)
        grid_y = int((map_y - map_info.origin.position.y) / map_info.resolution)

        return grid_x, grid_y

    x0, y0 = map_to_grid(robot_x, robot_y)
    x1, y1 = map_to_grid(robot_x + max_range*cosf(robot_a),
                         robot_y + max_range*sinf(robot_a))

    # compute the real world distance given a hit point which is a map grid index
    def _calc_range(_x, _y):
        xd = (_x - x0)
        yd = (_y - y0)
        d = sqrtf(xd*xd + yd*yd)
        d_world = d * map_info.resolution
        return d_world


    if abs(y1-y0) > abs(x1-x0):
        steep = True
    else:
        steep = False

    if steep:
        x0, y0 = y0, x0
        x1, y1 = y1, x1

    deltax = abs(x1-x0)
    deltay = abs(y1-y0)
    error = 0
    deltaerr = deltay
    x = x0
    y = y0

    if x0 < x1:
        xstep = 1
    else:
        xstep = -1
    if y0 < y1:
        ystep = 1
    else:
        ystep = -1

    cdef int w = omap.shape[0]
    cdef int h = omap.shape[1]

    while x != (x1 + xstep*1):
        x += xstep
        error += deltaerr
        if error*2 >= deltax:
            y += ystep
            error -= deltax

        if not steep:
            if 0 <= y < w and 0 <= x < h:
            # if is_valid(y, x):
                if omap[y][x] > filled_threshold:
                    return _calc_range(x, y)
                    # return (sqrtf((x-x0)*(x-x0) + (y-y0)*(y-y0)) / scale)
        else:
            if 0 <= x < w and 0 <= y < h:
            # if is_valid(x, y):
                if omap[x][y] > filled_threshold:
                    return _calc_range(x, y)
                    # return (sqrtf((x-x0)*(x-x0) + (y-y0)*(y-y0)) / scale)

    return max_range