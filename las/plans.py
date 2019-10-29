import time
import math
import numpy as np
from scipy import stats
import bluesky.plan_stubs as bps
from bluesky.preprocessors import (run_decorator, stage_decorator)
import logging
#from pswalker.plans import measure_centroid, measure_average, measure
#from pswalker.las_plans import point_center_adaptive, nf_ff_point_center

#from las.db import cam4, cam3, cam2, hole_m5, hole_m4, hole_m2, hole_m1, nf, ff
from las.db import cam3, cam2, hole_m5, hole_m4, hole_m2, hole_m1, nf, ff

def cam_centroid(n=1, cam=cam2):
    """Get the centroid of cam2."""
    fps = cam2.stats2.array_rate.get()
    rate = 1/fps
    cents = []
    for i in range(n):
        cents.append(cam2.stats2.centroid.get())
        time.sleep(rate)

    xvals = [c.x for c in cents]
    yvals = [c.y for c in cents]

    xave = np.average(xvals)
    yave = np.average(yvals)

    centroid = {'x': xave, 'y': yave}

    return centroid
        
class pointing_goal():
    """Data structure for pointing/centering loop goals."""

    def __init__(self, assembly, target, calibration=None, max_step=None,
                 deadband=None):

        self.assembly = assembly
        self.target = target
        self.max_step = max_step
        self.calibration = calibration
        self.deadband = deadband

nf_goal_1 = pointing_goal(nf, {'x':594, 'y':526}, max_step=30, deadband=1)
nf_goal_2 = pointing_goal(nf, {'x':314, 'y':264}, max_step=30, deadband=5)
ff_goal_1 = pointing_goal(ff, {'x':672, 'y':538}, max_step=30, deadband=1)
ff_goal_2 = pointing_goal(ff, {'x':867, 'y':595}, max_step=30, deadband=5)

def heinz_nf_ff_point_center():

    goals = [nf_goal_1, ff_goal_1]
    correlated_point_center_loop(goals, 20)

def nf_ff_point_center_loop(nf_goal, ff_goal, ntries):
    """Loop over multiple different, correlated goals until all are satisfied
    at once. Used for pointing and centering on NF/FF cameras."""

    goals = [nf_goal, ff_goal]

    correlated_point_center_loop(goals, ntries)

def correlated_point_center_loop(goals, ntries):
    """Loop over multiple different, correlated goals until all are satisfied
    at once."""

    # Update assembly from goal as needed
    for goal in goals:
        # Check assembly for calibration
        if goal.calibration:
            goal.assembly.cal = goal.calibration
        elif goal.assembly.cal is None:
            goal.assembly.calibrate()
        else: # No requested calibration, and assembly is already calibrated
            pass

        # Update max_step, if requested
        if goal.max_step: goal.assembly.max_step = goal.max_step

        # Update deadband, if requested
        if goal.deadband: goal.assembly.deadband = goal.deadband

    # Loop over all pointing/centering goals
    for i in range(ntries+1):
        if i >= ntries:
            raise Exception("Couldn't converge!") 
        for goal in goals:
            goal.assembly.point_center(ntries, goal.target, goal.assembly.deadband)
  
        # Check each goal for convergence 
        converged = [goal.assembly.converged for goal in goals]
        if all(goal == True for goal in converged): break
    
def point_center_loop(goals, ntries):
    """Loop over pointing/centering goals. Used to tie many un-correlated
    pointing/centering loops together."""

    for goal in goals:
        # Check assembly for calibration
        if goal.calibration:
            goal.assembly.cal = goal.calibration
        elif goal.assembly.cal is None:
            goal.assembly.calibrate()
        else: # No requested calibration, and assembly is already calibrated
            pass

        # Update max_step, if requested
        if goal.max_step: goal.assembly.max_step = goal.max_step

        # Update deadband, if requested
        if goal.deadband: goal.assembly.deadband = goal.deadband

        for i in range(ntries+1):
            if i >= ntries:
                raise Exception("Couldn't converge!") 
            goal.assembly.point_center(ntries, goal.target, goal.assembly.deadband)
    
            if goal.assembly.converged: break
