import subprocess
from subprocess import check_output
import sys
import numpy as np
proc = './../../../../../../devel/lib/skimap_ros/kd_test'

is_debug = False
current_arg_index = 1


def arg(dtype=float):
    global current_arg_index
    v = dtype(sys.argv[current_arg_index])
    current_arg_index += 1
    return v


bench_type = arg(str)

if bench_type == 'points':

    N_DIM = arg(int)
    N_POINTS_LIST = np.arange(arg(float), arg(float), arg(float))
    MAX_COORDINATES = arg(float)
    RESOLUTION = arg(float)
    RADIUS = arg(float)
    ALGO = arg(str)

    for points in N_POINTS_LIST:
        command = proc + " {} {} {} {} {} {} {}".format(
            N_DIM,
            points,
            MAX_COORDINATES,
            RESOLUTION,
            RADIUS,
            ALGO,
            is_debug
        )
        # print command
        p = subprocess.Popen(command.split(' '), stdout=subprocess.PIPE)
        out, err = p.communicate()
        print out
    sys.exit(0)

if bench_type == 'dimensions':

    N_DIMS = range(arg(int), arg(int), arg(int))
    N_POINTS = arg(float)
    MAX_COORDINATES = arg(float)
    RESOLUTION = arg(float)
    RADIUS = arg(float)
    ALGO = arg(str)

    for dim in N_DIMS:
        command = proc + " {} {} {} {} {} {} {}".format(
            dim,
            N_POINTS,
            MAX_COORDINATES,
            RESOLUTION,
            RADIUS,
            ALGO,
            is_debug
        )
        # print command
        p = subprocess.Popen(command.split(' '), stdout=subprocess.PIPE)
        out, err = p.communicate()
        print out
    sys.exit(0)

# step = 2000
# coords = 20000
# coords_max = coords * coords

# points = [1000, 10000, 20000, 50000, 100000, 150000,
#           200000, 300000, 500000, 1000000, 1500000, 2000000,  3000000, 3500000, 5000000, 10000000, 20000000]  # , 1000000, 2000000, 5000000, 10000000, 20000000, 25000000]


# for i in points:
#     n_points = i
#     coords = coords
#     algo = sys.argv[1]
#     command = proc + " {} {} {} 0".format(n_points, coords, algo)
#     p = subprocess.Popen(command.split(' '), stdout=subprocess.PIPE)
#     out, err = p.communicate()
#     print out
