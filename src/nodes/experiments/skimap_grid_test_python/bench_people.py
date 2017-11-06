import subprocess
from subprocess import check_output
import sys

proc = './../../../work/ros/skimap_ws/devel/lib/skimap_ros/grid2d'

step = 2000
coords = 24000
coords_max = coords * coords

points = [60000000]


for i in points:
    n_points = i
    coords = coords
    algo = sys.argv[1]
    command = proc + " {} {} {} 0".format(n_points, coords, algo)
    p = subprocess.Popen(command.split(' '), stdout=subprocess.PIPE)
    out, err = p.communicate()
    print out
