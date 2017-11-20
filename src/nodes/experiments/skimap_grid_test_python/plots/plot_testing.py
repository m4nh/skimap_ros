import numpy as np
import matplotlib.pyplot as plt
import glob
import os
import csv
from plotutils import Bench

folder = "/home/daniele/work/ros/skimap_ws/src/skimap_ros/src/nodes/experiments/skimap_grid_test_python/matlab/bench_octree_points"

files = glob.glob(os.path.join(folder, "*.csv"))

name_composition = ""  # "0_str:1_int:4_int"
columns = "DIMENSIONS:POINTS:MAX_COORDINATES:RESOLUTION:TIME_INTEGRATION:TIME_RADIUS_SEARCH:MEMORY"


benchs = []
for f in files:
    bench = Bench(f, name_composition=name_composition,
                  columns=columns, start_column=1)
    benchs.append(bench)
    print bench.name

benchs.sort(key=lambda x: x.name)

x_data = benchs[0].getDataByName('POINTS')

for i in range(0, len(benchs)):
    y_data = benchs[i].getDataByName('TIME_RADIUS_SEARCH')
    plt.plot(x_data, y_data, label=benchs[i].name)
plt.legend()
plt.show()
