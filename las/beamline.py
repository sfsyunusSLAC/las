from ophyd.device import Component as Cpt
from ophyd.epics_motor import EpicsMotor
from ophyd.signal import EpicsSignal

from hutch_python.utils import safe_load
from pcdsdevices.mv_interface import FltMvInterface
from pcdsdevices.areadetector.detectors import PCDSDetector
from .devices import SmarAct
from .devices import SmarAct_OpenLoop
from .devices import TipTilt


with safe_load('las motors 1'):
    hole_m1 = SmarAct_OpenLoop('LAS:MCS2:01:m1', name='hole_m1')
    hole_m2 = SmarAct_OpenLoop('LAS:MCS2:01:m2', name='hole_m2')
    #hole_m1 = SmarAct('LAS:MCS2:01:m1', name='hole_m1')
    #hole_m2 = SmarAct('LAS:MCS2:01:m2', name='hole_m2')
    hole_m3 = SmarAct('LAS:MCS2:01:m3', name='hole_m3')
    #hole_m4 = SmarAct('LAS:MCS2:01:m4', name='hole_m4')
    hole_m4 = SmarAct_OpenLoop('LAS:MCS2:01:m4', name='hole_m4')
    hole_m5 = SmarAct_OpenLoop('LAS:MCS2:01:m5', name='hole_m5')
    #hole_m5 = SmarAct('LAS:MCS2:01:m5', name='hole_m5')
    hole_m6 = SmarAct('LAS:MCS2:01:m6', name='hole_m6')
    hole_m7 = SmarAct('LAS:MCS2:01:m7', name='hole_m7')
    hole_m8 = SmarAct('LAS:MCS2:01:m8', name='hole_m8')
    hole_m9 = SmarAct('LAS:MCS2:01:m9', name='hole_m9')

with safe_load('las motors 2'):
    hole_m10 = SmarAct('LAS:MCS2:02:m1', name='hole_m10')
    hole_m11 = SmarAct('LAS:MCS2:02:m2', name='hole_m11')
    hole_m12 = SmarAct('LAS:MCS2:02:m3', name='hole_m12')

with safe_load('las camera 1'):
    cam1 = PCDSDetector('LAS:GIGE:HOLE:01:', name='hole_cam1')
with safe_load('las camera 2'):
    cam2 = PCDSDetector('LAS:GIGE:HOLE:02:', name='hole_cam2')
with safe_load('las camera 3'):
    cam3 = PCDSDetector('LAS:GIGE:HOLE:03:', name='hole_cam3')
with safe_load('las camera 4'):
    cam4 = PCDSDetector('LAS:GIGE:HOLE:04:', name='hole_cam4')
with safe_load('las plans'):
    from .plans import *
with safe_load('tip-tilt'):
    nf = TipTilt(hole_m1, hole_m2, cam4)
    ff = TipTilt(hole_m5, hole_m4, cam3)
