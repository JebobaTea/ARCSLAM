import os
import sys
import csv
import copy
import time
import random
import argparse

import numpy as np
np.set_printoptions(precision=4)

from utils.ScanContextManager import *
from utils.PoseGraphManager import *
from utils.UtilsMisc import *
from utils.MapManager import *
from utils.Control import Vehicle
from utils.Scanner import *
import utils.UtilsPointcloud as Ptutils
import utils.ICP as ICP

# params
parser = argparse.ArgumentParser(description='PyICP SLAM arguments')

# roughly 500 points can be expected
# quirky error suppression used to handle insufficient point count by random duplication
parser.add_argument('--num_icp_points', type=int, default=500) # 5000 is enough for real time
parser.add_argument('--num_rings', type=int, default=20) # same as the original paper
parser.add_argument('--num_sectors', type=int, default=60) # same as the original paper
parser.add_argument('--num_candidates', type=int, default=10) # must be int
parser.add_argument('--try_gap_loop_detection', type=int, default=10) # same as the original paper
parser.add_argument('--loop_threshold', type=float, default=0.07) # 0.11 is usually safe (for avoiding false loop closure)
parser.add_argument('--data_dir', type=str,
                    default='data/')
parser.add_argument('--sequence_idx', type=str, default='00')
parser.add_argument('--save_gap', type=int, default=1)
parser.add_argument('--icp_tries', type=int, default=50)
parser.add_argument('--base_result_dir', type=str,
                    default='POSE/')
parser.add_argument('--clip_prec', type=int, default=1)
parser.add_argument('--icp_tolerance', type=float, default=0.00000001)
parser.add_argument('--target_x', type=float, default=2.5)
parser.add_argument('--target_y', type=float, default=1.5)

args = parser.parse_args()

# Pose Graph Manager (for back-end optimization) initialization
PGM = PoseGraphManager()
PGM.addPriorFactor()

def homogenize(pts):
    m = pts.shape[1]
    res = np.ones((m + 1, pts.shape[0]))
    res[:m, :] = np.copy(pts.T)
    return res

# Result saver
save_dir = args.base_result_dir + args.sequence_idx
if not os.path.exists(save_dir): os.makedirs(save_dir)
ResultSaver = PoseGraphResultSaver(init_pose=PGM.curr_se3,
                             save_gap=args.save_gap,
                             num_frames=-1,
                             seq_idx=args.sequence_idx,
                             save_dir=save_dir)

# Scan Context Manager (for loop detection) initialization
SCM = ScanContextManager(shape=[args.num_rings, args.num_sectors],
                                        num_candidates=args.num_candidates,
                                        threshold=args.loop_threshold)

# mapping class
world = World(clip_prec=args.clip_prec, start_weight=8, cull_threshold=10000)

# used to apply controls
vehicle = Vehicle()

# init lidar
scanner = YDScanner(rep=2)
if not scanner.activate():
    raise RuntimeError("Lidar did not wake up, check USB and init parameters")

# @@@ MAIN @@@: data stream
for_idx = 0
clk = time.time()
while True:
    try:
        if time.time() - clk < 0.1:
            continue
        else:
            clk = time.time()
        # testing placeholder
        #if for_idx > 8:
        #    break

        #print(f"Reading scan no. {for_idx}, starting timer (measured in process time)")
        tstart = time.process_time()
        # grab scan, currently placeholder for lidar scan call
        #curr_scan_pts = Ptutils.readScan(f"../SCAN0/c_{for_idx}.npz")

        # actual data
        _, curr_scan_pts = scanner.run_scan()
        
        curr_scan_down_pts = Ptutils.random_sampling(curr_scan_pts, num_points=args.num_icp_points)
        
        #if not curr_scan_down_pts.all():
        #    for_idx += 1
        #    continue
        # save current node
        PGM.curr_node_idx = for_idx # make start with 0
        SCM.addNode(node_idx=PGM.curr_node_idx, ptcloud=curr_scan_down_pts)
        if(PGM.curr_node_idx == 0):
            PGM.prev_node_idx = PGM.curr_node_idx
            prev_scan_pts = copy.deepcopy(curr_scan_pts)
            icp_initial = np.eye(4)
            for_idx += 1
            print("c")
            continue

        dnn = None
        prev_scan_down_pts = Ptutils.random_sampling(prev_scan_pts, num_points=args.num_icp_points)

        #print("Read & downsampling complete: time since start is " + str(time.process_time() - tstart))

        #print("Using custom ICP")
        odom_transform, dnn, _ = ICP.icp(curr_scan_down_pts, prev_scan_down_pts, init_pose=icp_initial, max_iterations=args.icp_tries, tolerance=args.icp_tolerance)

        #print("ICP complete: time since start is " + str(time.process_time() - tstart))
        # update the current (moved) pose
        PGM.curr_se3 = np.matmul(PGM.curr_se3, odom_transform)
        icp_initial = odom_transform # assumption: constant velocity model (for better next ICP converges)

        # ARC TESTING PORTION
        base = homogenize(curr_scan_pts)
        pose = PGM.curr_se3
        transformed = pose @ base
        transformed = transformed.T
        base = base.T

        #print("Starting map build operation")
        world.update(transformed)
        #print("Map built & propagated in " + str(time.process_time() - tstart))
        # add the odometry factor to the graph
        PGM.addOdometryFactor(odom_transform)

        # apply controls
        #print(odom_transform)
        vehicle_pos = [pose[0][3], pose[1][3], pose[2][3]]
        print("Current position: " + str(vehicle_pos))
        if (for_idx < 5 or for_idx % 3 == 0):
            targx = int(args.target_x * math.pow(10, args.clip_prec))
            targy = int(args.target_y * math.pow(10, args.clip_prec))
            wpts = world.grid.generateWaypoints(world.clip(pose[0][3]), world.clip(pose[1][3]), targx, targy)
            # wpts = world.grid.generateWaypoints(-20, 50, 24, -14)
            # ideally, slice waypoints ::4 for smoother path
            print(wpts)

            vehicle.replace_waypoints(wpts[::4], args.clip_prec)
        vehicle.drive(vehicle_pos)

        # renewal the prev information
        PGM.prev_node_idx = PGM.curr_node_idx
        prev_scan_pts = copy.deepcopy(curr_scan_pts)

        # loop detection and optimize the graph
        if(PGM.curr_node_idx > 1 and PGM.curr_node_idx % args.try_gap_loop_detection == 0):
            # 1/ loop detection
            loop_idx, loop_dist, yaw_diff_deg = SCM.detectLoop()
            if(loop_idx == None): # NOT FOUND
                pass
            else:
                print("Loop event detected: ", PGM.curr_node_idx, loop_idx, loop_dist)
                # 2-1/ add the loop factor
                loop_scan_down_pts = SCM.getPtcloud(loop_idx)
                loop_transform, _, _ = ICP.icp(curr_scan_down_pts, loop_scan_down_pts, init_pose=yawdeg2se3(yaw_diff_deg), max_iterations=20)
                PGM.addLoopFactor(loop_transform, loop_idx)

                # 2-2/ graph optimization
                PGM.optimizePoseGraph()

                # 2-2/ save optimized poses
                ResultSaver.saveOptimizedPoseGraphResult(PGM.curr_node_idx, PGM.graph_optimized)

        # save the ICP odometry pose result (no loop closure)
        ResultSaver.saveUnoptimizedPoseGraphResult(PGM.curr_se3, PGM.curr_node_idx)
        #print("Loop closure and final I/O complete, full iteration took " + str(time.process_time() - tstart))
        print()
        for_idx += 1
    except KeyboardInterrupt:
        print("Attempting to kill")
        scanner.deactivate()
        vehicle.kill()
        world.export("world/")
        print("Killed")

#world.export("world/")
#wa = world.grid.generateWaypoints(-24, 50, 24, -14)
#with open("path.npz", "wb+") as f:
#    np.save(f, np.array(wa))

scanner.deactivate()
vehicle.kill()
