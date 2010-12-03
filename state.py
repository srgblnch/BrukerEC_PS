#!/usr/bin/env python
# -*- coding: utf-8 -*-
# state.py
# TANGO Device Server (http://sourceforge.net/projects/tango-ds/)

'''Bruker power supply state and status handling.
'''

# Standard Library
from copy import copy, deepcopy

# Extra Packages
from PyTango import DevState
import PyTango as Tg

import PowerSupply.standard as PS
from PowerSupply.util import bitpack2, bitunpack, bit_filter_msg

PM = PS.MSG

TYPE_1 = {
    0x00: (DevState.ALARM, 'GSM_CONFIG'),
    0x01: (DevState.ALARM, 'GSM_CONFIG_1' ),
    0x02: (DevState.ALARM, 'SYNC' ),
    0x03: (DevState.OFF, 'IDLE: relay opened, pulse off (DC OFF)' ),
    0x04: (DevState.FAULT, 'ADC_CAL'),
    0x05: (DevState.INIT, 'INRUSH'),
    0x06: (DevState.ON, 'DCON: power on' ),
    0x07: (DevState.STANDBY, 'STANDBY' ),
    0x08: (DevState.INIT, 'PULSE ON'),
    0x09: (DevState.ALARM, 'REG ON'),
    0x0a: (DevState.ON, 'RUNNING: relay closed, pulse and regulation on (DC ON)'),
    0x0b: (DevState.INIT, 'STOPPING'),
    0x0c: (DevState.ALARM, 'FAULT: relay opened, pulse and regulation off, error pending'),
    0x0d: (DevState.ALARM, 'FAULT_ZC'),
}

TYPE_3 = deepcopy(TYPE_1)
TYPE_3.update({
    0x00: (DevState.INIT, 'GSM_CONFIG: waiting for synchronization command'),
    0x00: (DevState.INIT, 'GSM_CONFIG_1'),
    0x03: (DevState.OFF, 'IDLE: pulse off (DC OFF)' ),
    0x08: (DevState.INIT, 'PULSEON: activating 4Q PWM and regulation'),
    0x09: (DevState.INIT, 'REGON: send DC ON order, waiting weak board(s) to reach RUNNING state'),
    0x0a: (DevState.ON, 'RUNNING: pulse and regulation on (DC ON)'),
    0x0c: (DevState.ALARM, 'FAULT_PN: pulse and regulation on, error pending'),
    0x0d: (DevState.ALARM, 'FAULT_PF: pulse and regulation off, error pending'),
})

TYPE_4 = deepcopy(TYPE_1)
TYPE_6 = deepcopy(TYPE_1)
TYPE_7 = deepcopy(TYPE_1)
TYPE_8 = deepcopy(TYPE_3)
TYPE_8.update({
    0x03: (DevState.OFF, 'IDLE: relay opened, pulse off (DC OFF)' ),
})

class Module(object):
    """Represents a regulation board
    """
    error_code = None
    errors = None
    machine_stat = {
        0 : (DevState.UNKNOWN, 'not applicable'),
        None : (DevState.UNKNOWN, 'not applicable'),
    }
    alias = ()
    fullname = None


    def __init__(self, name, errors=None, machine_stat=None, fullname=None):
        if not name is None:
            self.name = name
        if not errors is None:
            self.errors = errors
        if not machine_stat is None:
            self.machine_stat = machine_stat
        if fullname is None:
            self.fullname = self.name
        else:
            self.fullname = fullname

        # required for update_stat2
        self.alarms = []
        self.state_id = None

    def __str__(self):
        return '%s' % self.name

    __repr__ = __str__

    def update_stat2(self):
        '''Updates stat attribute from self.state_id and self.error_code
        '''
        self.alarms = bit_filter_msg(self.error_code, self.errors)
        if not self.errors:
            self.stat = DevState.INIT, 'cabinet type not detected (yet)'
        elif self.alarms:
            status = '\n'.join(self.alarms)
            self.stat = DevState.ALARM, status
        elif self.state_id is None:
            self.stat = [ DevState.INIT, 'machine state not available (?)' ]
        elif self.state_id > len(self.machine_stat):
            self.stat = [ DevState.FAULT, 'unknown state [%02d]' % self.state_id ]
        else:
            self.stat = self.machine_stat[self.state_id]


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
EBIT = lambda x: PM.UNDOCUMENTED_BIT.format('error', x)
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

MOD_C = Module('corrector', ERRORS_CORR, TYPE_1)
MOD_C.alias = ('cor', 'c', 't1')
MOD_Q = Module('4Q', ERRORS_QUAD, TYPE_3, 'quadrupole 4Q standalone')
MOD_Q.alias = ( 'q', 't3', 'quad', 'quad standalone')
MOD_QM = Module('4Q master', ERRORS_BEND, TYPE_3, 'quadrupole 4Q master' )
MOD_QM.alias = ('qm', 'quad master')
MOD_QS = Module('4Q slave', ERRORS_BEND, TYPE_3, 'quadrupole 4Q slave' )
MOD_QS.alias = ('qs', 'quad slave')
MOD_BUCK = Module('buck', ERRORS_BUCK, TYPE_4 )
MOD_BUCK.alias = ('t4', )
MOD_BUCK1 = Module('buck 1', ERRORS_BUCK, TYPE_4 )
MOD_BUCK1.alias = ('buck1',)
MOD_BUCK2 = Module('buck 2', ERRORS_BUCK, TYPE_4 )
MOD_BUCK2.alias = ('buck2',)
MOD_S = Module('sextupole', ERRORS_CORR, TYPE_7 )
MOD_S.alias = ('sex', '6', 't7')
MOD_BM = Module('4Q master', ERRORS_BEND, TYPE_8, 'bending 4Q master' )
MOD_BM.alias = ( 'bend', 'bend master', 'bm', 't8')
MOD_BS = Module('4Q slave', ERRORS_BEND, TYPE_8, 'bending 4Q slave' )
MOD_BS.alias = ( 'bend', 'bend slave', 'bs')
CABINET_XI = xrange(12, 16)

# quadrupole types
PSTYPE_CODE_QUAD = 3

# defining PsType objects
# quad with 2 sub-modules, ( QS120/180, QC320/340 )
PSTYPE_SMALL_QUAD = PSType_SmallQuad('small quadrupole',
  MOD_Q, MOD_BUCK
)
# quad with 4 sub-modules ( QC340 )
PSTYPE_BIG_QUAD = PSType_Big('big quadrupole',
  MOD_QM, MOD_BUCK1, MOD_QS, MOD_BUCK2
)

PSTYPE_BEND = PSType_Big('bending',
  MOD_BM, MOD_BUCK1, MOD_BS, MOD_BUCK2, Isafe=470)

# sextupole type
PSTYPE_SEX = PSType('sextupole', MOD_C, XI=xrange(12, 16))
PSTYPE_SEX.REG_PARAM = copy(PSType.REG_PARAM)
del PSTYPE_SEX.REG_PARAM['BuckV']

# maps software types to PSTypes
REG2PSTYPE = {
    1 : PSType('corrector', MOD_C, XI=xrange(12, 16) ), # correctors and quadrupoles?
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

class __ModuleRegistry(object):


    def __init__(self):
        self.name_dict = dict()
        self.name_list = []
        self.module_list = []

    def register(self, *modules):
        for m in modules:
            self.module_list.append(m)
            self.name_dict[m.fullname.lower()] = m

            for a in m.alias:
                self.name_dict[a] = m

    def get(self, name):
        mod = self.name_dict.get(name.lower())
        return mod

    def ls1(self, m):
        names = [ m.fullname ]
        names +=  m.alias
        return names

    def ls(self):
        return [ ', '.join(self.ls1(m)) for m in self.module_list ]

MODULE_REGISTRY = __ModuleRegistry()
MODULE_REGISTRY.register(MOD_C, MOD_Q, MOD_QM, MOD_QS, MOD_BUCK, MOD_BUCK1,
    MOD_BUCK2, MOD_S, MOD_BM, MOD_BS)

# dummy object that is never equal to a 'real' state (integer)
STATE_NONE = object()

BITCOUNT = (7,24)
BITOVER = (0,0)

def synthesize_error_code(st, error_code):
    code, overflow = bitpack2(BITCOUNT, BITOVER, (st, error_code) )
    assert not overflow[0], 'state code {0} too big'.format(st)
    assert not overflow[1], 'error code {0} too big'.format(code)
    return code

def analyze_error_code(code):
    return bitunpack(BITCOUNT, BITOVER, code)[0]

def format_module_stat(mod):
    alarm_text = ''
    code = synthesize_error_code(mod.state_id, mod.error_code)
    if mod.alarms:
        for a in mod.alarms:
            alarm_text += '\n    ' + a
    else:
        alarm_text = 'no alarm'
    try:
        status = mod.machine_stat[mod.state_id][1]
    except (KeyError,IndexError):
        status = 'undocumented state code'
    return (
        'board: {0}\n'.format(mod.fullname)+
        'code: {0:#x}\n'.format(code)+
        'status: {0:#x} {1}\n'.format(mod.state_id, status) +
        'error code: {0:#x}\n'.format(mod.error_code) +
        'alarms: ' +  alarm_text
    )

BOARD_LIST = [
    MOD_C,
    MOD_Q,
    MOD_BUCK,
    MOD_S ,
]




