import pandas as pd
from matplotlib import pyplot as plt
import numpy
import os

plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True

names = [
        "QuadrupedSync",
        "QuadrupedWalk",
        "QuadrupedWheel",
        "QuadrupedTrot",
        # "HexapodStair",
        # "HexapodWheel",
        # "HexapodWalk",
        # "SWhegTripod",
        # "SWhegRipple",
        # "RhexTripod",
        # "RhexRipple",
        # "Wheel"
        ]

ranges = {
        "QuadrupedSync":[240,840],
        "QuadrupedWalk":[126,726],
        "QuadrupedWheel":[0,600],
        "QuadrupedTrot":[123,723],
        # "HexapodStair":[240,840],
        # "HexapodWheel":[0,600],
        # "HexapodWalk":[0,600]
        # "SWhegTripod":[0,120],
        # "SWhegRipple":[0,120],
        # "RhexTripod":[0,120],
        # "RhexRipple":[0,120],
        # "Wheel":[0,600]
}

data = {}

for name in names:
    data[name] = pd.read_csv("/home/infas12/Desktop/wheelleg_ws/src/util/csvlogs/"+name+".csv", usecols=["cost"])[ranges[name][0]:ranges[name][1]]
    data[name].reset_index(drop=True, inplace=True)
    print(data[name])

time = numpy.linspace(0.0,6.0,600)

# quad walk: 126-726
# quad scyn: 0-600
# quad trot: 123-723
# quad wheel:0-600
# hex  stair:240-840
# hex  walk: 0-600
# hex  wheel:0-600

plt.figure(figsize=(10,3))

for name in names:
    plt.plot(time,data[name])
    plt.legend(names,loc="upper left",fontsize=12)
    plt.xlabel('time (s)', fontsize=15)
    plt.ylabel('cost (rad$^2$)', fontsize=15)

plt.title('Quadruped', fontsize=15)
plt.rc('xtick', labelsize=15)
plt.rc('ytick', labelsize=15)

plt.show()