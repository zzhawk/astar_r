import matplotlib.pyplot as plt
import numpy as np

map = open("map.csv", "r")
plan = open("plan.csv", "r")
search = open("search.csv", "r")

xmin = 10000
xmax = 0
ymin = 10000
ymax = 0

for grid in map:
    item = grid.split(',')
    x = float(item[0])
    y = float(item[1])
    xmin = min(xmin, x)
    xmax = max(xmax, x)
    ymin = min(ymin, y)
    ymax = max(ymax, y)
    if '1' in item[3]: plt.plot(x, y, ".k")

for sr in search:
    item = sr.split(',')
    x = float(item[0])
    y = float(item[1])
    plt.plot(x, y, "xb")

pl_x = []
pl_y = []
for pl in plan:
    item = pl.split(',')
    pl_x.append(float(item[0]))
    pl_y.append(float(item[1]))
plt.plot(np.asarray(pl_x), np.asarray(pl_y))




xmin = xmin-1
xmax = xmax+1
ymin = ymin-1
ymax = ymax+1

xbond=[xmin, xmax, xmax, xmin, xmin]
ybond=[ymin, ymin, ymax, ymax, ymin]
plt.plot(np.asarray(xbond), np.asarray(ybond))

plt.show()
map.close()
plan.close()
search.close()