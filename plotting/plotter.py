import numpy
import math
from lxml import etree
from matplotlib import pyplot as plt
import os

fig = plt.figure()
ax1 = fig.add_subplot(111)

data = numpy.genfromtxt("telemetry.csv", delimiter=",", names = ["lat", "lon", "bearing"])
# print(data)


# for f_p in os.listdir("osm_ways"):
#     lats = []
#     lons = []
#     with open(os.path.join("osm_ways", f_p), "r") as f:
#         tree = etree.fromstring(f.read())
#     for elem in tree.iter("node"):
#         lats.append(float(elem.get("lat")))
#         lons.append(float(elem.get("lon")))

#     ax1.scatter(lons, lats, color = "black")

ax1.scatter(data["lon"], data["lat"], s = 2)

for lat, lon, bearing in data:
    bearing = math.radians(bearing)
    r = 0.00001
    plt.arrow(lon, lat, r * math.cos(bearing), r * math.sin(bearing), width = 0.00001)

plt.show()