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
    zero_piezo_cmd = Cpt(EpicsSignal, ':PIEZO_OFF.PROC')


with safe_load('las motors'):
    motorname = SmarAct('PV:NAME', name='motorname')
