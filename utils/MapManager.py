import numpy as np
import time
import random
from heapq import heapify, heappop, heappush
import matplotlib.pyplot as plt
from matplotlib.pyplot import inferno

np.set_printoptions(precision=4)

from utils.UtilsMisc import *

class World:
    def __init__(self, clip_prec=1, start_weight=8, cull_threshold=10000):
        self.pose = None
        self.clip_prec = clip_prec
        self.raw_points = None
        self.start_weight = start_weight
        self.grid = CellGrid()
        self.cull_threshold = cull_threshold

    def update(self, pts):
        tstart = time.process_time()
        print("Running update operation")
        if self.raw_points is None:
            self.raw_points = pts
        else:
            # single vstack is o(n) rather than appending in a loop, which is o(n^2)
            self.raw_points = np.vstack((self.raw_points, pts))

        tstart = time.process_time()
        times = []
        for point in pts:
            # pixelate
            clipped_x = self.clip(point[0])
            clipped_y = self.clip(point[1])
            times.append(self.grid.addCell(clipped_x, clipped_y, self.start_weight))
        avg_time = sum(times) / len(times)
        print("Avg march time was " + str(avg_time))
        print("# of pts: " + str(len(times)))

        pt_ct = self.raw_points.shape[0]
        if pt_ct > self.cull_threshold:
            victims = np.random.choice(pt_ct, size=pt_ct - self.cull_threshold, replace=False)
            # should also be o(n) operation
            self.raw_points = np.delete(self.raw_points, victims, axis=0)

        print("Update took " + str(time.process_time() - tstart))

    def clip(self, z):
        return math.floor(z * math.pow(10, self.clip_prec))

    def export(self, path):
        with open(path + "raw.npz", "wb+") as f:
            np.save(f, self.raw_points)
        wpts = []
        for _, val in self.grid.cellmap.items():
            for _, c in val.items():
                wpts.append([c.x, c.y, c.weight])
        with open(path + "pixels.npz", "wb+") as f:
            np.save(f, np.array(wpts))


class CellGrid:
    def __init__(self):
        self.cellmap = {}

    def addCell(self, x, y, w):
        if x not in self.cellmap:
            self.cellmap[x] = {}
        if y not in self.cellmap[x]:
            c = Cell(x, y, w)
            self.cellmap[x][y] = c
        else:
            # cell already created, but was either a placeholder or marched/infected
            c = self.getCell(x, y)
            if c.weight < w:
                # overwrite
                c.weight = w
        tstart = time.process_time()
        self.march(c, w)
        return time.process_time() - tstart

    def getNeighbors(self, c, parent=None):
        return [self.getCell(c.x + 1, c.y + 1, parent),
                self.getCell(c.x + 1, c.y, parent),
                self.getCell(c.x + 1, c.y - 1, parent),
                self.getCell(c.x, c.y + 1, parent),
                self.getCell(c.x, c.y - 1, parent),
                self.getCell(c.x - 1, c.y + 1, parent),
                self.getCell(c.x - 1, c.y, parent),
                self.getCell(c.x - 1, c.y - 1, parent)]

    # returns empty by default, assumes LiDAR has not discovered it or the location is unoccupied
    def getCell(self, x, y, parent=None):
        if x not in self.cellmap:
            self.cellmap[x] = {}
        if y not in self.cellmap[x]:
            # default weight is 1
            self.cellmap[x][y] = Cell(x, y, 1, parent)
        return self.cellmap[x][y]

    def march(self, c, start_weight):
        if (start_weight < 1):
            return
        for cell in self.getNeighbors(c):
            if cell.weight < start_weight:
                cell.weight = start_weight
                self.march(cell, self.decay(start_weight))

    def decay(self, w):
        return w/2

    def astar(self, start_cell, end_cell):
        if math.isinf(end_cell.weight):
            print("Destination occupied")
            return
        if math.isinf(start_cell.weight):
            print("Start occupied")
            return
        # index 1 added because pq starts eating into subsequent indices of the tuple when comparison of 1st item is equal
        pq = [(0, 0, start_cell)]
        heapify(pq)
        visited = set()
        modified = set()

        vis = False
        if vis:
            pt = np.array([[start_cell.x, start_cell.y], [end_cell.x, end_cell.y]])
            plt.show()
        framecount = 0
        skip = 10
        while pq:
            _, _, curr_cell = heappop(pq)
            framecount += 1

            # debug
            if curr_cell.weight > 4:
                print(f"Why are we navigating here: {curr_cell.x}, {curr_cell.y} | {curr_cell.weight} + {curr_cell.f} + {curr_cell.h}")

            if curr_cell in visited:
                continue
            else:
                if curr_cell is start_cell:
                    modified.add(curr_cell)
                    curr_cell.f = 0
                    curr_cell.g = 0
                    curr_cell.h = 0
                    curr_cell.parent = curr_cell
            visited.add(curr_cell)
            # visualization for debugging
            if vis and framecount % skip == 0:
                pt = np.vstack((pt, [curr_cell.x, curr_cell.y]))
                plt.clf()
                data = np.load("world/pixels.npz")
                cp = []
                for i in data[:]:
                    if i[2] == 0:
                        continue
                    i[2] = math.sqrt(i[2])
                    i[2] /= 64
                    cp.append([i[0], i[1], i[2]])
                cp = np.array(cp)
                plt.scatter(cp[:, 0], cp[:, 1], s=4, c=cp[:, 2], cmap="gray_r")
                plt.scatter(pt[:, 0], pt[:, 1], s=4, c="red")
                plt.pause(0.01)
            # print("Visiting " + str(curr_cell.x) + ", " + str(curr_cell.y))

            for neighbor in self.getNeighbors(curr_cell, curr_cell):
                if neighbor is end_cell:
                    neighbor.parent = curr_cell
                    print("Found path")

                    # rebuild path
                    path = []
                    ccell = end_cell
                    while ccell is not start_cell and ccell is not None:
                        path.append([ccell.x, ccell.y])
                        ccell = ccell.parent
                    self.flush(modified)
                    return path
                gn = curr_cell.g + neighbor.weight
                hn = math.sqrt(math.pow(neighbor.x - end_cell.x, 2) + math.pow(neighbor.y - end_cell.y, 2))
                fn = gn + hn
                if neighbor not in visited or neighbor.f > fn:
                    neighbor.g = gn
                    neighbor.h = hn
                    neighbor.f = fn
                    neighbor.parent = curr_cell
                    heappush(pq, (fn, hn + random.random(), neighbor))
                    modified.add(neighbor)
        print("No path found")
        self.flush(modified)

    def flush(self, modded):
        for cell in modded:
            cell.f = float("inf")
            cell.g = float("inf")
            cell.h = 0
            cell.parent = None

    def generateWaypoints(self, x1, x2, y1, y2):
        startCell = self.getCell(x1, x2)
        endCell = self.getCell(y1, y2)
        print("Beginning path search")
        tstart = time.process_time()
        path = self.astar(startCell, endCell)
        print("End of search, took " + str(time.process_time() - tstart))
        print(path)
        return path


class Cell:
    def __init__(self, x, y, weight, parent=None):
        self.x = x
        self.y = y
        self.weight = weight
        self.parent = parent
        self.f = float("inf")
        self.h = 0
        self.g = float("inf")