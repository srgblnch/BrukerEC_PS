#!/usr/bin/env python
# -*- coding: utf-8 -*-
# state.py
# TANGO Device Server (http://sourceforge.net/projects/tango-ds/)

'''Bruker power supply state and status handling.
'''

# Standard Library
import copy

# Extra Packages
import ps_standard as PS
from PyTango import DevState
import PyTango as Tg
import traceback



PSSL = PS.StateLogic

class Port(object):

    def __init__(self, name, err, mask=0):
        self.name = name
        self.errors = err
        self.mask = mask

    def __str__(self):
        return '%s' % self.name

    __repr__ = __str__

class RP(object):
      """Description of 1 RegulationParameter."""
      def __init__(self, cobj, index=0, w=1):
	  self.cobj = cobj
	  self.index = index
	  self.rw = Tg.READ_WRITE if w else Tg.READ


class PSType(object):
    '''Description of the different types of power supply
       in particular interpretation of error flags.
    '''

    has_trigger_mask = False

    REG_PARAM = {
        'BuckV' : RP(0x2041, w=0),
        'BuckV_Kp' : RP(0x2055),
        'BuckV_Ki' : RP(0x2053),
        'MainI_Kp' : RP(0x2026),
        'MainI_Ki' : RP(0x2024),
        'MainV_Kp' : RP(0x2029),
        'MainV_Ki' : RP(0x2027)
    }

    def __init__(self, name, *ports, **kwargs):
        self.name = name
        self.ports = ports
        self.XI = kwargs.get('XI',0)
        self.states_off = kwargs.get('off', (3,) )
        self.states_on = kwargs.get('on', (10, ) )
        self.states_standby = kwargs.get('standby', () )
        self.mask_cab = kwargs.get('mask_cab', 0)
        self.use_waveforms = True

    def __str__(self):
        return self.name + " PS "+ str(self.ports)

    def __repr__(self):
        return self.__class__.__name__ + repr( (self.name, self.ports) )

    def query_Voltage(self, impl):
	    cmd = impl.cab.command
	    master = impl.Port
	    cmd_adv = lambda p: float(cmd(p, 'ADV/'))
	    adv = cmd_adv(master)
	    return PS.VDQ(adv, q=PS.AQ_VALID)


class PSType_4Q(PSType):

    has_trigger_mask = True

    REG_PARAM = {
    'BuckV' : RP(0x200B,1, w=0),
    'BuckI_Kp' : RP(0x2026,1),
    'BuckI_Ki' : RP(0x2024,1),
    'BuckV_Kp' : RP(0x2029,1),
    'BuckV_Ki' : RP(0x2027,1),

    'MainI_Kp' : RP(0x2026,0),
    'MainI_Ki' : RP(0x2024,0),
    'MainV_Kp' : RP(0x2029,0),
    'MainV_Ki' : RP(0x2027,0),
    }

    def query_Voltage(self, impl):
        cmd = impl.cab.command
        master = impl.Port+1
        cmd_adv = lambda p: float(cmd(p, 'ADV/'))
        adv = cmd_adv(master)
        return PS.VDQ(adv, q=PS.AQ_VALID)

class PSType_4QC(PSType_4Q):

    has_trigger_mask = True


    def query_Voltage(self, impl):
        cmd = impl.cab.command
        cmd_adv = lambda p: float(cmd(p, 'ADV/'))
        adv_master = cmd_adv(impl.Port+1)
        adv_slave = cmd_adv(impl.Port+3)
        return PS.VDQ(adv_master+adv_slave, q=PS.AQ_VALID)

# the integers are used as symbolic constants
ERRORS_CORR = [
    ## Error byte, LSB, 0x00 ... 0x08
    'earth leak',
    'EPROM / DC Filter',
    'load overcurrent / overvoltage',
    'ADC voltage / watchdog',
    'DC on fault',
    'communication',
    'DCCT',
    'IGBT',

    ## Error byte, MSB, 0x00 ... 0x08
    'power stage overtemperature',
    'switcher overtemperature',
    'rectifier overtemperature',
    'transformer overtemperature',
    'extern 1',
    'extern 2',
    'extern 3',
    'extern 4'
]

ERRORS_4QM = [
    ## Error byte, LSB, 0x00 ... 0x80
    'earth leak',
    'EPROM',
    'current or voltage maximum exceeded',
    'ADC Voltage / Watchdog',
    'DCCT Fault',
    'IGBT 2Q2 (2)',
    'temperature B',
    'oscillation',

    ## Error byte, MSB, 0x0100 ... 0x8000
    'unbalance C',
    'IGBT 2Q1 (1)',
    'IGBT 2Q2 (1)',
    'IGBT 2Q1 (2)',
    'unbalance 2Q2',
    'temperature L2',
    'unbalance 2Q1',
    'temperature L1'
]

ERRORS_QUAD = [
    ## Error byte, LSB, 0x00 ... 0x80
    'earth leak',
    'EPROM',
    'current or voltage maximum exceeded',
    'ADC Voltage / Watchdog',
    'DCCT Fault',
    'IGBT 2Q2 (2)',
    'temperature B',
    'oscillation',

    ## Error byte, MSB, 0x0100 ... 0x8000
    'unbalance C',
    'IGBT 2Q1 (1)',
    'IGBT 2Q2 (1)',
    'IGBT 2Q1 (2)',
    'ext.1 / unbalance 2Q2',
    'ext.2 / temperature L2',
    'ext.3 / unbalance 2Q1',
    'ext.4 / temperature L1'
]

EBIT = 'unspecified error (bit %d)'
ERRORS_4QS = copy.copy(ERRORS_4QM)
ERRORS_4QS[0] = EBIT % 0
ERRORS_4QS[4] = EBIT % 4
ERRORS_4QS[7] = EBIT % 7
ERRORS_4QS[9] = 'IGBT 2Q1 (2)'
ERRORS_4QS[10] = 'IGBT 2Q2 (1)'
ERRORS_4QS[11] = 'IGBT 2Q1 (1)'
ERRORS_4QS[13] = 'temperature L4'
ERRORS_4QS[15] = 'temperature L3'

ERRORS_BUCK = [
    'buck over voltage',
    'EEPROM error',
    'buck over current',
    'ADC U/watchdog',
    'rectifier over voltage',
    EBIT % 5,
    EBIT % 6,
    EBIT % 7,

## MSB
    EBIT % 8,
    EBIT % 9,
    EBIT % 10,
    'IGBT fault',
    EBIT % 12,
    EBIT % 13,
    EBIT % 14,
    EBIT % 15,
]

# global errors


ERRORS_CABINET = [
    'cabinet CAN communication',
    'cabinet EPROM',
    EBIT % 2,
    'cabinet watch dog',
    'communication cabinet',
    'phase',
    'door open',
    'water',
# MSB
    EBIT % 8,
    'cabinet fuse',
    'emergency / PSS',
    'temperature cabinet'
    'extern 1',
    'extern 2',
    'extern 3',
    'extern 4'
]

MACHINE_STATUS = (
    'GSM_CONFIG',
    'GSM_CONFIG_1',
    'SYNC',
    'IDLE (ready state)',
    'ADC_CAL',
    'INRUSH',
    'DC ON',
    'STANDBY',
    'PULSE ON',
    'REG ON',
    'RUNNING (DC On state)',
    'STOPPING',
    'FAULT',
    'FAULT_ZC',
    'unknown (29)'
)


PORT_RELAY = Port('Relay', ERRORS_CABINET)
PORT_RELAY.port = 19
PORT_Q = Port('4Q', ERRORS_QUAD)
PORT_QM = Port('4Q master', ERRORS_4QM)
PORT_BUCK = Port('buck', ERRORS_BUCK)
PORT_BUCK1 = Port('buck 1', ERRORS_BUCK)
PORT_BUCK2 = Port('buck 2', ERRORS_BUCK)
PORT_QS = Port('4Q slave', ERRORS_4QS)
PORT_C = Port('corrector', ERRORS_CORR)
PORT_S = Port('sextupole', ERRORS_CORR)

PORTS_BIG = PORT_QM, PORT_BUCK1, PORT_QS, PORT_BUCK2
CABINET_XI=xrange(12, 16)

# quadrupole types
PSTYPE_CODE_QUAD = 3
# quad with 2 sub-modules (QC340)
PSTYPE_SMALL_QUAD = PSType_4Q('small quadrupole', PORT_Q, PORT_BUCK)
# quad with 4 sub-modules (QC340)
PSTYPE_BIG_QUAD = PSType_4QC('big quadrupole', *PORTS_BIG)

# sextupole type
PSTYPE_SEX = PSType('sextupole', PORT_C, XI=xrange(12, 16) , mask_cab=0x08)
PSTYPE_SEX.REG_PARAM = copy.copy(PSType.REG_PARAM)
del PSTYPE_SEX.REG_PARAM['BuckV']

# maps software types to PSTypes
REG2PSTYPE = {
    1 : PSType('corrector', PORT_C, XI=xrange(12, 16), mask_cab=0x18), # correctors and quadrupoles?
    2 : PSType('LT bend', PORT_C, XI=xrange(12, 16)), # bending magnets of the LT cabinet
    3 : None, # is a quadrupole but could have 2 or 4 submodules, so further work required
    4 : None, #< buck for ???
    5 : None, # quadrupole relay board
    6 : None, #< buck for ???,
    7 : PSTYPE_SEX,
    8 : PSType_4QC('dipole', *PORTS_BIG, Isafe=470, XI=xrange(12,16), states_on=(4,), states_off=(1,)), # aka
    9 :  None, #< dipole relay board
}


# interpretation of cobj_BRKconfig 2 object
BRKconfig2_MASTER = 0
BRKconfig2_STANDALONES = (1,2)
BRKconfig2_SLAVE = 3

COBJ_MAIN_REGUL_FREQ = 0x200A
COBJ_MAIN_IREF_SCALED = 0x2020
COBJ_MAIN_IREF_LIMIT = 0x2021
COBJ_VERSION = 0x2034
COBJ_BRK_CONFIG2 = 0x2038

class StateLogic(PS.StateLogic):
    '''Encapsulates all logic required to update state, status and errors.
    '''

    FAULT = PSSL.FAULT
    MACHINE_STAT = {
        0x00: (PSSL.INIT_X, 'GSM_CONFIG' ),
        0x01: (PSSL.INIT_X, 'GSM_CONFIG_1' ),
        0x02: (PSSL.ALARM, 'SYNC'),
        0x03: (PSSL.OFF,),
        0x04: (PSSL.FAULT, 'ADC_CAL'),
        0x05: (PSSL.SWITCHING_ON, 'INRUSH'),
        0x06: (PSSL.ON_CURRENT,),
        0x07: (PSSL.STANDBY,),
        0x08: (PSSL.RUNNING, 'PULSE ON'),
        0x09: (PSSL.ALARM, 'REG ON'),
        0x0a: (PSSL.ON_CURRENT,),
        0x0b: (PSSL.SWITCHING_OFF,),
        0x0c: (PSSL.INTERLOCK,),
        0x0d: (PSSL.ALARM, 'FAULT_ZC'),
        29: (PSSL.ALARM, 'ALARM 29')
    }

FAULT_STATES = (0x0d,)
INTERLOCK_STATES = (0x0c,)

