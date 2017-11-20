import subprocess
from subprocess import check_output
import sys

proc = './../../../work/ros/skimap_ws/devel/lib/skimap_ros/grid2d'

step = 2000
coords = 20000
coords_max = coords * coords

points = [1000, 10000, 20000, 50000, 100000, 150000,
          200000, 300000, 500000, 1000000, 1500000, 2000000,  3000000, 3500000, 5000000, 10000000, 20000000]  # , 1000000, 2000000, 5000000, 10000000, 20000000, 25000000]


for i in points:
    n_points = i
    coords = coords
    algo = sys.argv[1]
    command = proc + " {} {} {} 0".format(n_points, coords, algo)
    p = subprocess.Popen(command.split(' '), stdout=subprocess.PIPE)
    out, err = p.communicate()
    print out
