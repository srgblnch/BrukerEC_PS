#!/usr/bin/env python
# -*- coding: utf-8 -*-
# state.py
# TANGO Device Server (http://sourceforge.net/projects/tango-ds/)

'''Bruker power supply state and status handling.
'''

# Standard Library
import copy

# Extra Packages
from PyTango import DevState
import PyTango as Tg
import traceback

import PowerSupply.standard as PS

PSSL = PS.StateLogic

class Module(object):

    def __init__(self, name, err):
        self.name = name
        self.errors = err

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

    def __init__(self, name, *module_list, **kwargs):
        self.name = name
        self.module = module_list
        self.XI = kwargs.get('XI', ())
        # if not on well, it is off
        self.states_on = (10, )
        self.states_switch_off = kwargs.get('swoff', (11, 14, 15) )
        self.states_switch_on = kwargs.get('swon', (7,8,9 ) )
        self.states_standby = kwargs.get('standby', () )
        self.use_waveforms = True

    def update_xi(self, msg_list):
        '''Updates the message displayed for the external interlocks of this
           power supply.
        '''
        p0 = self.module[0]
        for idx,msg in zip(self.XI,msg_list):
            p0.errors[idx] = msg

    def __str__(self):
        return self.name + " PS "+ str(self.module)

    def __repr__(self):
        return self.__class__.__name__ + repr( (self.name, self.module) )

    def query_Voltage(self, impl):
        cmd = impl.cab.command
        master = impl.Port
        cmd_adv = lambda p: float(cmd(p, 'ADV/'))
        adv = cmd_adv(master)
        return PS.VDQ(adv, q=PS.AQ_VALID)


class PSType_SmallQuad(PSType):

      def __init__(self, name, *ports, **kwargs):
          PSType.__init__(self, name, *ports, **kwargs)
          if not 'XI' in kwargs:
              self.XI = range(12,16)

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

class PSType_Big(PSType_SmallQuad):

      def query_Voltage(self, impl):
        cmd = impl.cab.command
        cmd_adv = lambda p: float(cmd(p, 'ADV/'))
        adv_master = cmd_adv(impl.Port+0)
        adv_slave = cmd_adv(impl.Port+2)
        return PS.VDQ(adv_master+adv_slave, q=PS.AQ_VALID)

# common error messages
EBIT = lambda x: 'unused bit %d' % x
E_EARTH = 'earth leakage'
E_EEPROM = 'EEPROM error'
E_BIG_LOAD = 'load over-current, voltage or RMS current'
E_LOAD = 'load over-current or over-voltage'
E_ADC = 'ADC voltage or watchdog'
E_DCCT = 'DCCT fault'
E_BUCK_TEMP = 'buck overtemperature'
E_EXTERN = [ 'extern 1', 'extern 2', 'extern 3', 'extern 4' ]
E_CAPV = 'capacitor voltage unbalanced'
E_IGBT = 'IGBT fault'

# the integers are used as symbolic constants
# errors for t1 (correctors and sextupoles)
ERRORS_CORR = [
    ## Error byte, LSB, 0x00 ... 0x08
    E_EARTH,
    'DC filter overvoltage (OR {0})'.format(E_EEPROM),
    E_LOAD,
    E_ADC,
    'DC on fault',
    EBIT(5),
    E_DCCT,
    E_IGBT,

    ## Error byte, MSB, 0x00 ... 0x08
    'power stage overtemperature',
    'switcher overtemperature',
    'rectifier overtemperature',
    'transformer overtemperature'
] + E_EXTERN
assert(len(ERRORS_CORR)==16)

# errors for t3 (quad 4Q)
ERRORS_QUAD = [
    ## Error byte, LSB, 0x00 ... 0x80
    E_EARTH,
    E_EEPROM,
    E_BIG_LOAD,
    E_ADC,
    E_DCCT,
    E_CAPV,
    'switcher overtemperature',
    '4Q IGBT fault (bit 7)',

    ## Error byte, MSB, 0x0100 ... 0x8000
    'unbalance',
    '4Q IGBT fault (bit 9)',
    E_BUCK_TEMP,
    EBIT(11),
] + E_EXTERN
assert(len(ERRORS_QUAD)==16)

# errors for t8 (bending 4Q)
ERRORS_BEND = [
    ## Error byte, LSB, 0x00 ... 0x80
    E_EARTH,
    E_EEPROM,
    E_BIG_LOAD,
    E_ADC,
    E_DCCT,
    'IGBT 2Q2 (2) fault',
    E_BUCK_TEMP,
    'oscillation',

    ## Error byte, MSB, 0x0100 ... 0x8000
    E_CAPV,
    'IGBT 2Q1 (2) fault',
    'IGBT 2Q2 (1) fault',
    'IGBT 2Q1 (2) fault',
    'unbalance 2Q2 (2)',
    'self 2 overtemperature',
    'unbalance 2Q1 (1)',
    'self 1 overtemperature',
]

# errors for t4 (buck)
ERRORS_BUCK = [
    'buck over voltage',
    'EEPROM error',
    'buck over current',
    'ADC U/watchdog',
    'rectifier over voltage',
    EBIT(5),
    EBIT(6),
    EBIT(7),

## MSB
    EBIT(8),
    EBIT(9),
    EBIT(10),
    E_IGBT,
    EBIT(12),
    EBIT(13),
    EBIT(14),
    EBIT(15),
]
assert(len(ERRORS_BUCK)==16)

# global errors
ERRORS_CABINET = [
    'cabinet CAN communication',
    'cabinet EPROM',
    EBIT(2),
    'cabinet watch dog',
    'communication cabinet',
    'phase',
    'door open',
    'water',
# MSB
    EBIT(8),
    'cabinet fuse',
    'emergency / PSS',
    'temperature cabinet'
] + E_EXTERN
assert(len(ERRORS_CABINET)==16)

MOD_RELAY = Module('Relay', ERRORS_CABINET)
MOD_RELAY.port = 19
MOD_Q = Module('4Q', ERRORS_QUAD)
MOD_BM = Module('4Q master', ERRORS_BEND)
MOD_BS = Module('4Q slave', ERRORS_BEND)
MOD_BUCK = Module('buck', ERRORS_BUCK)
MOD_BUCK1 = Module('buck 1', ERRORS_BUCK)
MOD_BUCK2 = Module('buck 2', ERRORS_BUCK)
MOD_C = Module('corrector', ERRORS_CORR)
MOD_S = Module('sextupole', ERRORS_CORR)
CABINET_XI=xrange(12, 16)

# quadrupole types
PSTYPE_CODE_QUAD = 3

# defining PsType objects
# quad with 2 sub-modules, ( QS120/180, QC320/340 )
PSTYPE_SMALL_QUAD = PSType_SmallQuad('small quadrupole',
  MOD_Q, MOD_BUCK
)
# quad with 4 sub-modules ( QC340 )
PSTYPE_BIG_QUAD = PSType_Big('big quadrupole',
  MOD_Q, MOD_BUCK1, MOD_Q, MOD_BUCK2
)

PSTYPE_BEND = PSType_Big('bending',
  MOD_BM, MOD_BUCK1, MOD_BS, MOD_BUCK2, Isafe=470)

# sextupole type
PSTYPE_SEX = PSType('sextupole', MOD_C, XI=xrange(12, 16))
PSTYPE_SEX.REG_PARAM = copy.copy(PSType.REG_PARAM)
del PSTYPE_SEX.REG_PARAM['BuckV']

# maps software types to PSTypes
REG2PSTYPE = {
    1 : PSType('corrector', MOD_C, XI=xrange(12, 16)), # correctors and quadrupoles?
    2 : PSType('LT bend', MOD_C, XI=xrange(12, 16)), # bending magnets of the LT cabinet
    3 : None, # is a quadrupole but could have 2 or 4 submodules, so further work required
    4 : None, #< buck for ???
    5 : None, # quadrupole relay board
    6 : PSType('BT', MOD_C, XI=xrange(12,16)), #< code used by both buck and BT,
    7 : PSTYPE_SEX,
    8 : PSTYPE_BEND,
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
        0x00: (PSSL.ALARM, 'synchronization required 0' ),
        0x01: (PSSL.ALARM, 'synchronization required 1' ),
        0x02: (PSSL.ALARM, 'awaiting synchronization trigger'),
        0x03: (PSSL.OFF,),
        0x04: (PSSL.FAULT, 'ADC_CAL'),
        0x05: (PSSL.SWITCHING_ON, 'INRUSH'),
        0x06: (PSSL.ON_CURRENT,),
        0x07: (PSSL.STANDBY,),
        0x08: (PSSL.RUNNING, 'PULSE ON'),
        0x09: (PSSL.ALARM, 'REG ON'),
        0x0a: (PSSL.ON_CURRENT,),
        0x0b: (PSSL.SWITCHING_OFF,),
        0x0c: (PSSL.ALARM, 'fault'),
        0x0d: (PSSL.ALARM, 'fault ZC'),
        29: (PSSL.ALARM, 'ALARM 29')
    }

# dummy object that is never equal to a 'real' state (integer)
STATE_NONE = object()

