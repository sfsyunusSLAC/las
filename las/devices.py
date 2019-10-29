import time
import numpy as np
from scipy import optimize
from scipy.optimize import newton
from math import sqrt
import logging

from ophyd.device import Component as Cpt
from ophyd.device import Device 
from ophyd.epics_motor import EpicsMotor
from ophyd.signal import EpicsSignal, EpicsSignalRO
from ophyd.positioner import PositionerBase
#from ophyd.status import wait as status_wait
#from ophyd.status import (MoveStatus as status_wait)
from ophyd.status import MoveStatus

from pcdsdevices.mv_interface import FltMvInterface, MvInterface

waveplate_data = [[20.0, 9.29], [25.0, 8.69], [30.0, 7.7], [35.0, 6.4], [40.0, 4.8], [45.0, 3.1], [50.0, 1.7], [55.0, 0.5], [60.0, 0.099], [62.0, 0.0], [64.0, 0.0], [66.0, 0.0], [68.0, 0.099], [70.0, 0.20], [75.0, 0.90], [80.0, 2.20], [85.0, 3.80], [15.0, 9.0], [5.0, 7.0]]

class LASEnergyControl(object):
    """Class to implement energy control of the laser system. Uses a waveplate
    for control, and reads the energy from an instance of the TuttiFrutti
    diagnostic stack.

    Parameters
    ----------
    waveplate : SmarAct rotation stage 
        Rotation stage used to control a waveplate.

    tuttifrutti : TuttiFrutti instance
        Instance of a TuttiFrutti diagnostic stack. Used to retrieve the 
        current energy measurement.

    calibration : list
        Dictionary of calibration data for the waveplate.
    """
   
    #def __init__(self, waveplate, tuttifrutti, calibration=None):
    #    self.__waveplate = waveplate
    #    self.__ttf = tuttifrutti
    #    self.__cal = calibration # Calibration constants
 
    def calibrate(self, start, finish, npoints, energies, angles):
        """Calibrate the energy control waveplate.
        
        """
        #angles = []
        #energies = []
        
        logging.debug("Calibrating ... ")
        #for i in range(npoints):
        #    angle = start + ((finish-start)/npoints)*i
        #    angles.append(angle)
        #    self.__waveplate.move(angle)

        #    energy = self.__ttf.energy()
        #    energies.append(energy)

        # Store valid energy/angle ranges to validate requests
        self.__max_energy = max(energies)
        self.__min_energy = min(energies)
        self.__max_angle = max(angles)
        self.__min_angle = min(angles)
        logging.debug("min Energy: {}, max Energy: {}".format(self.__min_energy, self.__max_energy))
        logging.debug("min Angle: {}, max Angle: {}".format(self.__min_angle, self.__max_angle))
        
        self.__fit_calibration(angles, energies)

        logging.debug("Calibration complete!")
        logging.debug("Calibration: {}".format(self.__cal))

    def set_energy(self, energy):

        #TODO: Allow calibration to be passed to object. 
        if energy < self.__max_energy and energy > self.__min_energy:
            angle = self.__calc_angle(energy)
        else:
            raise ValueError("Requested energy is outside of calibrated range!")

        logging.debug("Calculated angle: {}".format(angle))

        if angle < self.__max_angle and angle > self.__min_angle:
            logging.debug("Calculated angle: {}".format(angle))
            print("Moving waveplate to {} degrees...".format(angle))
            #self.__waveplate.move(angle)
        else:
            raise ValueError("Requested angle is outside of calibrated range!")

        #TODO: Add tweak routine that adjusts the energy even closer to request?

    def __fit_calibration(self, angles, energies):
        """Fit waveplate calibration data."""

        p = np.polyfit(angles, energies, 10) # Use 10th order polynomial
        logging.debug("Calibration fit parameters: {}".format(p))

        self.__cal = list(p)

    def __calc_angle(self, energy):
        """Calculate position to move waveplate to based on calibration."""

        assert self.__cal is not None, "Waveplate is not calibrated!"
       
        logging.debug("Requested energy of {}".format(energy)) 
        # Do lookup table for initial guess
        lt = {}
        for i in range(int(self.__min_angle), int(self.__max_angle)): 
             lt[str(i)] = np.polyval(self.__cal, i) 

        # Find guess
        sorted_keys = sorted(lt.keys())
        for i in range(len(sorted_keys)): 
             if i == 0: continue 
             key = sorted_keys[i] 
             lastkey = sorted_keys[i-1] 
             if ((lt[key] > energy and lt[lastkey] < energy) or 
                 (lt[key] < energy and lt[lastkey] > energy)): 
                 guess = float(sorted_keys[i])
                 logging.debug("Initial value for minimization: {}".format(guess))
                 break

        # Minimization function
        def f(x, p, e):
            return p[0]*x**10+p[1]*x**9+p[2]*x**8+p[3]*x**7+p[4]*x**6+p[5]*x**5+p[6]*x**4+p[7]*x**3+p[8]*x**2+p[9]*x**1+p[10]-e

        a = (self.__cal, energy) # Arguments for minimization routine
        # Calculate angle using Newton's method
        angle = newton(f, guess, args=a)
        
        return angle

class SmarAct_OpenLoop(Device,MvInterface, PositionerBase):
    """
    Class to implement open loop features of SmarAct piezo stages.
 
    The idea is to implement open loop version of the SmarAct stage, similar
    to the ophyd epics_motor interface, and then use this device for SmarAct
    stages that do not have encoders. Since they do not have encoders, and
    never will, using the motor record with closed loop PVs doesn't make
    much sense.
    """

    # Voltage for sawtooth ramp
    step_voltage = Cpt(EpicsSignal, ':STEP_VOLTAGE')
    # Frequency of steps
    step_freq = Cpt(EpicsSignal, ':STEP_FREQ')
    # Number of steps per step forward, backward command
    step_count = Cpt(EpicsSignal, ':STEP_COUNT')
    # Jog forward 
    step_fwd_cmd = Cpt(EpicsSignal, ':STEP_FORWARD')
    # Jog backward 
    step_rev_cmd = Cpt(EpicsSignal, ':STEP_REVERSE')
    # Total number of steps counted
    total_step_count = Cpt(EpicsSignal, ':TOTAL_STEP_COUNT')
    # Reset steps ("home")
    step_clear_cmd = Cpt(EpicsSignal, ':CLEAR_COUNT')
    # Scan move?
    scan_move_cmd = Cpt(EpicsSignal, ':SCAN_MOVE')
    # Scan pos? 
    scan_pos = Cpt(EpicsSignal, ':SCAN_POS')

    motor_is_moving = Cpt(EpicsSignalRO, '.MOVN', kind='omitted')

    SUB_START = 'start_moving'
    SUB_DONE = 'done_moving'
    SUB_READBACK = 'readback'
    _SUB_REQ_DONE = '_req_done' # requested move finished subscription

    @property
    def egu(self):
        """The engineering units (EGU) for positions"""
        return 'steps'

    def move(self, position, timeout=None, moved_cb=None, wait=False):
        """Move the stage in open loop to the specified number of steps.
            
        Parameters
        ----------
        position : int
            The position to move to, in absolute number of steps.
        """

        if timeout is not None:
            raise ValueError("timeout is not implmeneted yet!")
        if moved_cb is not None:
            raise ValueError("moved_cb is not implmeneted yet!")
        if wait is not False:
            raise ValueError("Wait is not implmeneted yet!")

        current_position = self.position
        diff = position-current_position # Get absolute difference
        self.step_count.put(abs(int(diff)))
        if diff > 0:
            self.step_fwd_cmd.put(1)
        else:
            self.step_rev_cmd.put(1)

        self._run_subs(sub_type=self._SUB_REQ_DONE, success=False)
        self._reset_sub(self._SUB_REQ_DONE)

        status = MoveStatus(self, position, timeout=timeout,
                            settle_time=self._settle_time)

        self.subscribe(status._finished, event_type=self._SUB_REQ_DONE,
                       run=False)
        return status

    @property
    def moving(self):
        """Whether or not the motor is moving
        Returns
        -------
        moving : bool
        """
        return bool(self.motor_is_moving.get(use_monitor=False))
 
    @property
    def position(self):
        """Return the current open loop position of the motor."""
        return self.total_step_count.get() 
       
    def home(self):
        """Clear the step counter. Set current position to 0 steps."""
        self.step_clear_cmd.put(1) 

    def mvr(self, step):
        """Move the stage a relative number of steps."""
        abs_pos = int(self.position + step) # In steps
        self.move(abs_pos)
    
class SmarAct(FltMvInterface, EpicsMotor):
    """
    Mike's MCS2 Motor Record with bonus PVs attached
    """
    # Voltage for sawtooth ramp
    step_voltage = Cpt(EpicsSignal, ':STEP_VOLTAGE')
    # Frequency of steps
    step_freq = Cpt(EpicsSignal, ':STEP_FREQ')
    # Number of steps per step forward, backward command
    step_count = Cpt(EpicsSignal, ':STEP_COUNT')
    # Jog forward 
    step_fwd_cmd = Cpt(EpicsSignal, ':STEP_FORWARD')
    # Jog backward 
    step_rev_cmd = Cpt(EpicsSignal, ':STEP_REVERSE')
    # Total number of steps counted
    total_step_count = Cpt(EpicsSignal, ':TOTAL_STEP_COUNT')
    # Reset steps ("home")
    step_clear_cmd = Cpt(EpicsSignal, ':CLEAR_COUNT')
    # Scan move?
    scan_move_cmd = Cpt(EpicsSignal, ':SCAN_MOVE')
    # Scan pos? 
    scan_pos = Cpt(EpicsSignal, ':SCAN_POS')

class PointCenter_Assembly(object):
    """Base class to be inherited by other pointing/centering assembly
    objects."""
    
    def __init__(self, Target=None, Cal=None, DeadBand=None):
        self.cal = Cal
        self.target = Target
        self.deadband = DeadBand 

    def check_status(self):
        """Method to be implemented by inheriting class."""
        raise Exception("Method not implemented yet!")

    def centroid(self):
        """Method to be implemented by inheriting class."""
        raise Exception("Method not implemented yet!")

    def calibrate(self):
        """Method to be implemented by inheriting class."""
        raise Exception("Method not implemented yet!")

    def point(self):
        """Method to be implemented by inheriting class."""
        raise Exception("Method not implemented yet!")

    @property
    def converged(self):
        """Method to be implemented by inheriting class."""
        # Get centroid            
        cent = self.centroid()

        # Test for convergence
        if abs(cent['x']-self.target['x']) < self.deadband:
            if abs(cent['y']-self.target['y']) < self.deadband:
                return True
            else:
                return False
        else:
            return False

    def point_center(self, ntries, target=None, deadband=None):
        """Function to point the assembly closer to the target on the camera.

        This method is intended to be general. The assembly object must provide
        the logic to calculate the move needed, since this will differ
        depending on the motor stack used to do the move.
    
        Parameters
        ----------
        ntries : int
            The number of times to attempt the pointing and centering loop.

        target : dict
            A dictionary containing the target position to reach on the camera.
            Typically takes the form target={'x':<xtarg>, 'y':<ytarg>}.

        deadband : int
            The deadband for the routine. The pointing and centering loop will
            terminate once all target positions are met within the deadband,
            e.g.: xtarg-deadband < x < xtarg+deadband &&
                  ytarg-deadband < y < ytarg+deadband"""

        if deadband == None: deadband = self.deadband # Grab from sub class
        if target == None: target = self.target # Grab from sub class

        logging.debug("Running point_center method!")
        logging.debug("ntries: {}".format(ntries))
        logging.debug("target: {}".format(target))
        logging.debug("deadband: {}".format(deadband))

        for i in range(ntries+1):
            self.check_status() # Check that everything looks OK

            if i == ntries:
                raise Exception("Exceeded the number of tries!")

            logging.debug("Current attempt: {}".format(i))

            # Get initial centroid            
            cent = self.centroid()

            logging.debug("Centroid: {}".format(cent))

            # Make sure a target has been defined
            assert target is not None, "Please define a target position!"

            #TODO: Loop over target keys, rather than explicit x and y
            # Test for convergence
            if abs(cent['x']-target['x']) < deadband:
                if abs(cent['y']-target['y']) < deadband:
                    print("Converged!")
                    break

            # Check for calibration
            if self.cal is None:
                logging.debug("No calibration found! Calibrating!")
                self.calibrate()

            logging.debug("Current calibration: {}".format(self.cal))

            # Use inheriting class method to point assembly closer to target.
            # Position calculation and motion required will differ depending
            # on the motor stack, so do all of this in the inheriting class.

            logging.debug("Issuing point towards target...")
            self.point(target) 

            # Now loop back and check for convergence

class TipTilt(PointCenter_Assembly):
    """Class for doing tip-tilt beam pointing with un-encoded SmarAct stages.
    Combination of stages and a camera (areaDetector gigEcam), with methods
    specific for doing pointing/centering.

    Required Parameters
    -------------------
    Tip : SmarAct axis (unencoded)
        SmarAct axis corresponding to the Tip (y-motion) on the camera image.

    Tilt : SmarAct axis (unencoded)
        SmarAct axis corresponding to the Tilt (x-motion) on the camera image.

    Cam : camera
        Camera to point the camera on. Requires areaDetector Stats2 plugin for
        centroiding.

    Optional Parameters
    -------------------
    Max_Step : int 
        The maximum allowed move, in piezo steps. This step size is also used
        for calibration. Higher steps results in a better calibration and 
        faster pointing, but can also lead to pointing the beam off of the
        camera.

    Target : dict
        A dictionary containing the target position to reach on the camera.
        Typically takes the form target={'x':<xtarg>, 'y':<ytarg>}.

    Cal : dict
        A dictionary containing a calibration for the TipTilt assembly. Takes
        the form cal={'tip_fwd': <tip_cal_fwd>, 'tilt_fwd': <tilt_cal_fwd>,
                      'tip_rev': <tip_cal_rev>, 'tilt_rev': <tilt_cal_rev>}

    Deadband : int
        The deadband for the routine. The pointing and centering loop will
        terminate once all target positions are met within the deadband,
        e.g.: xtarg-deadband < x < xtarg+deadband &&
              ytarg-deadband < y < ytarg+deadband
    """

    def __init__(self, Tip, Tilt, Cam, Max_Step=20, *args, **kwargs):

        # Assembly components, properties
        self.tip = Tip
        self.tilt = Tilt
        self.cam = Cam
    
        self.max_step = Max_Step

        super().__init__(*args, **kwargs)

    def check_status(self):
        """Check on the status of the camera and motors, and make sure that
        the assembly is ready for pointing/centering.

        Parameters
        ----------
        None
        """
        # Make sure stats plugin is enabled
        stats_enabled = self.cam.stats2.enable.get()
        assert stats_enabled == 1.0, "Make sure stats plugin is enabled!"

        # Make sure centroiding is turned on
        centroiding = self.cam.stats2.compute_centroid.get()
        assert centroiding == 'Yes', "Make sure centroiding is turned on!"

        # Make sure the camera is acquiring images
        fps = self.cam.stats2.array_rate.get() # Camera frames per second
        assert fps > 0.0, "Make sure camera is acquiring images!"

        #TODO: Check that image has not run off the camera. How? Old centroid
        #      appears to remain.

        logging.debug("Check status found stats_enabled={}, centroiding={}, fps={}".format(stats_enabled, centroiding, fps))

    def centroid(self, n_centroid=1):
        """Return the centroid of the assembly camera, averaging over the 
        number of requested centroids. Implement any camera-specific code
        necessary here. Return centroid as dictionary.

        Parameters
        ----------
        n_centroid : int
            The number of centroids to measure. Default is 1.
        """

        # Check everything is working first
        self.check_status()

        # Get rate of centroid data.    
        fps = self.cam.stats2.array_rate.get()
        rate = 1.0/fps
    
        logging.debug("Measured centroid with rate={}, n_centroid={}".format(rate, n_centroid))

        # Get data
        centroids = []
        for i in range(n_centroid):
            centroids.append(self.cam.stats2.centroid.get())
            time.sleep(rate) # Allow centroid to update

        # Average data
        x_vals = [c.x for c in centroids]
        x_ave = np.average(x_vals)
        y_vals = [c.y for c in centroids]
        y_ave = np.average(y_vals)

        centroid = {'x': x_ave, 'y': y_ave}

        logging.debug("Measured centroid: {}".format(centroid))

        return centroid

    def point(self, target=None):
        """Method to move the tip-tilt assembly closer to the target
        position. Calculates the number of required steps, then makes the
        move.

        Required Parameters
        -------------------
        None

        Optional Parameters
        -------------------
        target : dict
            A dictionary containing the target position to reach on the camera.
            Typically takes the form target={'x':<xtarg>, 'y':<ytarg>}. If not
            specified, then the method will use the assembly .target attribute.
        """

        # Check everything is working first
        self.check_status()

        if target == None: 
            target = self.target
        else: # Update target attribute
            self.target = target

        # Check for calibration, target
        assert self.cal is not None, "Assembly needs to be calibrated!"
        assert target is not None, "Please specify a target position!"

        # Calculate difference from target
        current = self.centroid()

        x_err = target['x'] - current['x']
        logging.debug("x_err: {}".format(x_err))

        y_err = target['y'] - current['y']
        logging.debug("y_err: {}".format(y_err))

        # Calculate steps and coerce to reasonable value
        # Determine which calibration factors to use
        if x_err > 0:
            if self.cal['tilt_rev'] > 0 and self.cal['tilt_fwd'] > 0:
                xcal = self.cal['tilt_fwd']
            elif self.cal['tilt_rev'] < 0 and self.cal['tilt_fwd'] < 0:
                xcal = self.cal['tilt_rev']
            else:
                raise ValueError("Calibration signs don't match!")
        else:
            if self.cal['tilt_rev'] > 0 and self.cal['tilt_fwd'] > 0:
                xcal = self.cal['tilt_rev']
            elif self.cal['tilt_rev'] < 0 and self.cal['tilt_fwd'] < 0:
                xcal = self.cal['tilt_fwd']
            else:
                raise ValueError("Calibration signs don't match!")
        if y_err > 0:
            if self.cal['tip_rev'] > 0 and self.cal['tip_fwd'] > 0:
                ycal = self.cal['tip_fwd']
            elif self.cal['tip_rev'] < 0 and self.cal['tip_fwd'] < 0:
                ycal = self.cal['tip_rev']
            else:
                raise ValueError("Calibration signs don't match!")
        else:
            if self.cal['tip_rev'] > 0 and self.cal['tip_fwd'] > 0:
                ycal = self.cal['tip_rev']
            elif self.cal['tip_rev'] < 0 and self.cal['tip_fwd'] < 0:
                ycal = self.cal['tip_fwd']
            else:
                raise ValueError("Calibration signs don't match!")

        logging.debug("xcal: {}".format(xcal))
        logging.debug("ycal: {}".format(ycal))

        calc_x_step = x_err/xcal # err(pix) / cal(pix/step) = step
        x_step = self.__coerce_step(calc_x_step, self.max_step)
        logging.debug("x_step: {}".format(x_step))

        calc_y_step = y_err/ycal # err(pix) / cal(pix/step) = step
        y_step = self.__coerce_step(calc_y_step, self.max_step)
        logging.debug("y_step: {}".format(x_step))

        # Make relative moves to stages based on steps, wait
        logging.debug("Moving stages...")
        self.tilt.mvr(x_step)
        self.tip.mvr(y_step)
        time.sleep(1)

    def calibrate(self, step=None, n_average=5):
        """Calibrate the tip and tilt stages using the camera centroid.
        
        Required Parameters
        -------------------
        None

        Optional Parameters
        -------------------
        step : int
            The maximum step size to calibrate with. Does not affect current
            assembly max_step attribute, which is used for pointing. If not
            specified, then the calibration is done with the current max_step.

        n_average : int
            The number of centroids to average when calibrating. Default is 5.
        """

        # Pull in abs value of step, either argument or from instance
        if step == None: 
            step = abs(self.max_step) # pass class attr to method
        else:
            step = abs(step)

        # Check everything is working first
        self.check_status()

        logging.debug("Calibrating with step={}, n_average={}".format(step, n_average))

        # Do calibrations
        tip_cal_fwd = self.__calibrate_axis(self.tip, step, n_average)
        tilt_cal_fwd = self.__calibrate_axis(self.tilt, step, n_average) 
        tip_cal_rev = self.__calibrate_axis(self.tip, -1*step, n_average)
        tilt_cal_rev = self.__calibrate_axis(self.tilt, -1*step, n_average) 

        # Store calibration
        self.cal = {'tip_fwd': tip_cal_fwd, 'tilt_fwd': tilt_cal_fwd,
                    'tip_rev': tip_cal_rev, 'tilt_rev': tilt_cal_rev}

        logging.debug("Calibration: {}".format(self.cal))

    def __calibrate_axis(self, axis, step, n_average):
        """Function to move a stage and generate calibration."""

        # Check everything is working first
        self.check_status()

        # Make small half step first to deal with "backlash" weirdness of piezo
        axis.mvr(step/2)

        # Give stages time to move. (Need a better ophyd device.)
        time.sleep(1)

        # Measure first centroid
        c1 = self.centroid(n_average)        

        # Make the calibration step
        axis.mvr(step)

        # Give stages time to move. (Need a better ophyd device.)
        time.sleep(1)
        
        c2 = self.centroid(n_average)

        # Calculate slopes (calibration factors)
        x_slope = (c2['x'] - c1['x'])/step 
        y_slope = (c2['y'] - c1['y'])/step

        # The calibration factor for a single axis should be the result that had
        # the largest effect, e.g. the largest absolute value. Test for this, and
        # return a single factor. Not ideal, and may be done differently later,    
        # but this makes the calibration routine more general, and not axis
        # specific.

        if abs(x_slope) >= abs(y_slope):
            return x_slope
        else:
            return y_slope

    def __coerce_step(self, des_step, max_step):
        """Function to keep abs(step) <= max_step, and keep step > 0."""
        if des_step > max_step:
            curr_step = max_step
        elif 0 < des_step < 1: # Small positive number; coerce to 1
            curr_step = 1
        elif -1 < des_step < 0: # Small negative number; coerce to -1
            curr_step = -1
        elif des_step < -1 * max_step:
            curr_step = -1 * max_step
        else: # -max_step < curr_step < max_step, abs(des_step) > 1
            curr_step = des_step

        return curr_step
