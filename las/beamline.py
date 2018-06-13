from ophyd.device import Component as Cpt
from ophyd.epics_motor import EpicsMotor
from ophyd.signal import EpicsSignal

from hutch_python.utils import safe_load
from pcdsdevices.mv_interface import FltMvInterface


class SmarAct(FltMvInterface, EpicsMotor):
    """
    Mike's MCS2 Motor Record with bonus PVs attached
    """
    step_voltage = Cpt(EpicsSignal, ':STEP_VOLTAGE')
    step_freq = Cpt(EpicsSignal, ':STEP_FREQ')
    step_count = Cpt(EpicsSignal, ':STEP_COUNT')
    step_fwd_cmd = Cpt(EpicsSignal, ':STEP_FORWARD')
    step_rev_cmd = Cpt(EpicsSignal, ':STEP_REVERSE')
    total_step_count = Cpt(EpicsSignal, ':TOTAL_STEP_COUNT')
    step_clear_cmd = Cpt(EpicsSignal, ':CLEAR_COUNT')
    scan_move_cmd = Cpt(EpicsSignal, ':SCAN_MOVE')
    scan_pos = Cpt(EpicsSignal, ':SCAN_POS')


with safe_load('las motors'):
    hole_m1 = SmarAct('LAS:MCS2:01:m1', name='hole_m1')
    hole_m2 = SmarAct('LAS:MCS2:01:m2', name='hole_m2')
    hole_m3 = SmarAct('LAS:MCS2:01:m3', name='hole_m3')
    hole_m4 = SmarAct('LAS:MCS2:01:m4', name='hole_m4')
    hole_m5 = SmarAct('LAS:MCS2:01:m5', name='hole_m5')
    hole_m6 = SmarAct('LAS:MCS2:01:m6', name='hole_m6')
    hole_m7 = SmarAct('LAS:MCS2:01:m7', name='hole_m7')
    hole_m8 = SmarAct('LAS:MCS2:01:m8', name='hole_m8')
    hole_m9 = SmarAct('LAS:MCS2:01:m9', name='hole_m9')
