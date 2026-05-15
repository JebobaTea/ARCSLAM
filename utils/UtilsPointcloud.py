import os 
import random
import numpy as np

def random_sampling(orig_points, num_points):
    # assert orig_points.shape[0] > num_points
    ptsfinal = np.copy(orig_points)
    # force suppress insufficient points
    if (orig_points.shape[0] < num_points):
        # randomly clone existing points until threshold reached
        duplicates = orig_points[np.random.randint(orig_points.shape[0], size=(num_points - orig_points.shape[0] + 1)), :]
        ptsfinal = np.vstack((ptsfinal, duplicates))
    points_down_idx = random.sample(range(ptsfinal.shape[0]), num_points)
    down_points = ptsfinal[points_down_idx, :]
    return down_points

    # print("Insufficient points: " + str(orig_points.shape[0]))

    # return []

# force add Z, for now
def readScan(bin_path):
    scan = np.load(bin_path)
    scan_xyz = []
    for pt in scan:
        scan_xyz.append([pt[0], pt[1], 2.0])
    return np.array(scan_xyz)
    

class ARCScanDirManager:
    def __init__(self, scan_dir):
        self.scan_dir = scan_dir
        
        self.scan_names = os.listdir(scan_dir)
        self.scan_names.sort()    
        
        self.scan_fullpaths = [os.path.join(self.scan_dir, name) for name in self.scan_names]
  
        self.num_scans = len(self.scan_names)

    def __repr__(self):
        return ' ' + str(self.num_scans) + ' scans in the sequence (' + self.scan_dir + '/)'

    def getScanNames(self):
        return self.scan_names
    def getScanFullPaths(self):
        return self.scan_fullpaths
    def printScanFullPaths(self):
        return print("\n".join(self.scan_fullpaths))

