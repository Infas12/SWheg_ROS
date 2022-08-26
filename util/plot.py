import pandas as pd
from matplotlib import pyplot as plt
import os

plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True

names = [
        # "QuadrupedStair",
        # "QuadrupedWalk",
        # "QuadrupedWheel",
        # "QuadrupedTrot",
        # "HexapodStair",
        # "HexapodWheel",
        # "HexapodWalk",
        "LegTripod",
        "LegGait2",
        "Wheel"
        ]

ranges = {
        # "QuadrupedStair":[240,840],
        # "QuadrupedWalk":[126,726],
        # "QuadrupedWheel":[0,600],
        # "QuadrupedTrot":[123,723],
        # "HexapodStair":[240,840],
        # "HexapodWheel":[0,600],
        # "HexapodWalk":[0,600]
        "LegTripod":[0,120],
        "LegGait2":[0,120],
        "Wheel":[0,120]
}

data = {}

for name in names:
    data[name] = pd.read_csv("/home/infas12/Desktop/wheelleg_ws/src/util/csvlogs/"+name+".csv", usecols=["cost"])[ranges[name][0]:ranges[name][1]]
    data[name].reset_index(drop=True, inplace=True)
    print(data[name])
    
# quad walk: 126-726
# quad scyn: 0-600
# quad trot: 123-723
# quad wheel:0-600
# hex  stair:240-840
# hex  walk: 0-600
# hex  wheel:0-600

for name in names:
    plt.plot(data[name])
    plt.legend(names)
plt.show()