#!/usr/bin/python
# -*- coding: utf-8 -*-

# BrukerBend_PS.py
# This file is part of tango-ds (http://sourceforge.net/projects/tango-ds/)
#
# tango-ds is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# tango-ds is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with tango-ds.  If not, see <http://www.gnu.org/licenses/>.

"""
Used to provide a single Tango server for both bending magnet power supplies.
It allows also to synchronize dipoles and quadrupoles PS at the same time.
"""

META = """
    $URL: https://tango-ds.svn.sourceforge.net/svnroot/tango-ds/Servers/PowerSupply/BrukerEC_PS/1.8.6/BrukerBend_PS.py $
    $LastChangedBy: lkrause2 $
    $Date: 2010-07-09 12:52:22 +0200 (vie 09 de jul de 2010) $
    $Rev: 1842 $
    Author: Lothar Krause <lkrause@cells.es>
    License: GPL3+ $
"""

import PyTango as Tg
import sys
import ps_standard as PS
from ps_util import nop
import traceback
import re
from time import time
from pprint import pprint
from copy import deepcopy

AQ_ALARM = Tg.AttrQuality.ATTR_ALARM
AQ_WARNING = Tg.AttrQuality.ATTR_WARNING
AQ_VALID = Tg.AttrQuality.ATTR_VALID
AQ_INVALID = Tg.AttrQuality.ATTR_INVALID
AQ_CHANGING = Tg.AttrQuality.ATTR_CHANGING

DevState = Tg.DevState

# state definitions
CAB_OFF_STATE = 0x01
CAB_ON_STATE = 0x06
CAB_FAULT_STATE = 0x08

# None is used to mark states that have no fixed delay associated with them
CAB_T = {
    0x02 :   2.00,
    0x03 : 120.00, # or 78
    0x04 :   1.50,
    0x05 :   0.80,
    0x07 :  19.50,
    0x09 :   2.00,
    0x0a :  80.00,
    0x0b :  22.00,
    0x0c :   1.00,
    0x0d :   1.00,
    0x0e :   1.00,
    0x0f :   1.00,
    0x10 : 120.00
}
def CT(a,b=None):
    if b is None: b = a
    return sum(CAB_T[x] for x in range(a,b+1))

def BT(a,b=None):
    if b is None: b = a
    return sum(BEND_T[x] for x in range(a,b+1))

CAB_PENDING = frozenset( s for s,t in CAB_T.iteritems() if t)

# how long it takes to switch on cabinet ON when in various states
CAB_ON_T = {
    0x01 : 0.03 + CT(0x02, 0x05),
    0x02 : CT(0x02, 0x05),
    0x03 : CT(0x03, 0x05),
    0x04 : CT(0x04, 0x05),
    0x05 : CT(0x05, 0x05),
    0x06 : 0.0,
    0x07 : CT(0x07)+CT(0x10) + CT(0x02,0x05),
    0x09 : CT(0x09),
    0x0a : CT(0x0a),
    0x0b : CT(0x0b),
    0x0c : CT(0x0c),
    0x0d : CT(0x0d),
    0x0e : CT(0x0e),
    0x0f : CT(0x0f),
    0x10 : CT(0x10) + CT(0x02,0x05)
}

BEND_OFF_STATE = 0x03
BEND_ON_STATE = 0x0a
BEND_FAULT_STATES = frozenset( (0x0c, 0x0d) )

# times of transitory states
BEND_T = {
    0x08 :  1.20,
    0x09 : 77.60,
    0x0b :  7.50,
    0x0e :  8.00,
    0x0f :  4.50
}
BEND_PENDING = frozenset(s for s,t in CAB_T.iteritems() if t)

# how long it takes to switch on power supply ON when in various states
BEND_ON_T = {
    0x03 : BT(0x08, 0x09),
    0x08 : BT(0x08, 0x09),
    0x09 : BT(0x09),
    0x0a : 0,
    0x0b : BT(0x0b) + BT(0x0e, 0x0f) + BT(0x08, 0x09),
    0x0e : BT(0x0e, 0x0f) + BT(0x08, 0x09),
    0x0f : BT(0x0f) + BT(0x08, 0x09),
}

# states in which power supply is on, or going to be on
BEND_POSSIBLY_ON_STATES = frozenset( (4,5,6,7,8,9, 10, 12) )

# how long it takes to switch power supply off when in various states
BEND_OFF_T = {
    0x03 : 0.0,
    0x08 : BT(0x0b)+BT(0x0e,0x0f),
    0x09 : BT(0x0b)+BT(0x0e,0x0f),
    0x0a : BT(0x0b)+BT(0x0e,0x0f),
    0x0b : BT(0x0b)+BT(0x0e,0x0f),
    0x0e : BT(0x0e,0x0f),
    0x0f : BT(0x0f),
}

BEND_PENDING = frozenset(s for s,t in BEND_T.iteritems() if t)

# end of state definitions

SWITCH_NEUTRAL = 'sw 0'
SWITCH_ON = 'sw on'
SWITCH_OFF = 'sw off'
DB = PS.DATABASE

tv2time = lambda x: x.tv_sec + x.tv_usec/1000.0

SECONDS_PAT = re.compile(r'^(.*) (\d*.\d+)s$')

FAULT = object()

def strip_s(x):
    mat = SECONDS_PAT.match(x)
    if mat is None:
        return x,None
    else:
        return mat.group(1), float(mat.group(2))

def WaveStatus_okay(v):
    return 'ing...' in v or 'finished' in v or v=='ready'

def merge_str(v1,v2):
    tail = ''
    for x,y in zip(v1[::-1],v2[::-1]):
        if x==y:
            tail+=x
        else:
            break
    tail = tail[::-1]
    n = len(tail)
    return v1[:-n].strip()+' / '+v2[:-n].strip()+' '+tail.strip()

class BrukerBend_PS(PS.PowerSupply):

 #   PUSHED_AT
    PUSHED_ATTR_EXTRA = ['Voltage', 'Current', 'CurrentSetpoint',
        'WaveGeneration', 'WaveOffset', 'WaveLength', 'WaveDuration',
        'TriggerMask', 'WaveName']

    PUSHED_ATTR = ['State', 'Status'] + PUSHED_ATTR_EXTRA
    # Device and Class Properties
    Bend1 = None
    Bend2 = None
    QuadCabinet = None
    Trigger = None

    def __init__(self, cl, name):
        PS.PowerSupply.__init__(self,cl,name)
        BrukerBend_PS.init_device(self)

    def init_device(self):
        PS.PowerSupply.init_device(self)
        self.__shiver = {}
        self.switching = SWITCH_NEUTRAL
        self.switch_eta = 0
        self._aWaveGeneration = PS.VDQ(None)
        self.next_update_t = 0

        self.bend1 = Tg.DeviceProxy(self.Bend1)
        self.bend2 = Tg.DeviceProxy(self.Bend2)

        def find_cab(ec_name):
            """finds name of BrukerEC_Cabinet corresponding to
               the device name specified
            """
            serv_name = 'BrukerEC_PS/bo_b'+ec_name[-1]
            ls = DB.get_device_class_list(serv_name)
            dictum = dict(zip(ls[1::2],ls[::2]))
            return dictum['BrukerEC_Cabinet']

        self.bend1.cab = Tg.DeviceProxy(find_cab(self.Bend1))
        self.bend2.cab = Tg.DeviceProxy(find_cab(self.Bend2))

        # threshold when warning / alarms will be signalled
        self.AbsDiff = 0.1
        self.cabq = Tg.DeviceProxy(self.QuadCabinet)
        self.trig = Tg.DeviceProxy(self.Trigger)

        # prepares also DeviceProxy for the Synchronization
        self.sync_devices = [ self.bend1.cab, self.bend2.cab, self.cabq ]

    @PS.CommandExc
    def Lock(self):
        self.bend1.lock()
        self.bend2.lock()
        self.cab1.lock()
        self.cab2.lock()

    @PS.CommandExc
    def Unlock(self):
        self.bend1.unlock()
        self.bend2.unlock()
        self.cab1.unlock()
        self.cab2.unlock()

    def shiver(self, aname, x, y):
        """Every other second x is chosen, else y
	"""
	return x if int(time()) % 2 else y

    def distill_ne_quality(self, quality, x,y):
        if quality==AQ_INVALID: return AQ_INVALID
        r = x.value != y.value
        try:
            r = bool(r)
        except ValueError:
            r = any(r)
        return AQ_ALARM if r else quality

    def distill_waveform_quality(self, quality, x,y):
            return AQ_ALARM if tuple(x.value) != tuple(y.value) else quality

    @PS.AttrExc
    def read_attribute(self, attr, qual_fun=None, valfun=None):
        def read1(bend):
            aname = attr.get_name()
            try:
                    aval = bend.read_attribute(attr.get_name())
                    return aval
            except Exception:
                    self.log.error('read1 %s', aname, exc_info=1)
                    return None

        # read_attribute begins here
        try:
            if time()<self.next_update_t:
                attr.set_quality(AQ_INVALID)
                return
            aname = attr.get_name()

            aval1 = read1(self.bend1)
            aval2 = read1(self.bend2)
            if aval1 is None or aval2 is None:
                attr.set_quality(AQ_INVALID)
                return

            quality = PS.combine_aq(aval1.quality, aval2.quality)
            timestamp = min(tv2time(aval1.time),tv2time(aval2.time))

            if qual_fun is None:
                quality = self.distill_ne_quality(quality, aval1, aval2)
            else:
                quality = qual_fun(quality, aval1, aval2)

            if valfun is None:
                valfun = self.shiver
            if aval1.value is None or aval2.value is None:
                x = None
            else:
                x = valfun(aname, aval1.value, aval2.value)

            vdq = PS.VDQ(x, q=quality)
            setattr(self, '_a'+aname, vdq)
            vdq.set_attr(attr)

            if not aval1.w_value is None and not aval2.w_value is None:
                w = valfun(aname+'_W', aval1.w_value, aval2.w_value)
                if not w is None:
                        attr.set_write_value(w)

        except Tg.DevFailed:
            self.delay_update()
            raise

    def distill_I_quality(self, quality, aval1, aval2):
        diff = abs(aval1.value - aval2.value)
        if self._aWaveGeneration.value:
            quality = AQ_CHANGING
        elif quality == AQ_VALID:
            if diff > self.AbsDiff:
                quality = AQ_WARNING
            if diff > 2*self.AbsDiff:
                quality = AQ_ALARM
        return quality

    def is_Voltage_allowed(self, write):
        return not self._aWaveGeneration.value

    @PS.AttrExc
    def read_Current1(self, attr):
        self.read_attr1(self.bend1, attr, 'Current')

    @PS.ExceptionHandler
    def read_Current2(self, attr):
        self.read_attr1(self.bend2, attr, 'Current')

    def read_attr1(self, bend, attr, aname=None):
        if aname is None:
            aname = attr.get_name()
        aval = bend.read_attribute(aname)
        I = aval.value
        timestamp = tv2time(aval.time)
        quality = aval.quality
        attr.set_value_date_quality(I, timestamp, quality)

    PS.CommandExc
    def On(self):
        self.set_state(DevState.INIT)
        self.set_status('switching on...')
        if self.get_state()==Tg.DevState.ALARM:
            self.ResetInterlocks()
        self.switching = SWITCH_ON
        self.switch_eta = time() + self.estimate_switch_on_time()
        self.UpdateState()
        self.upswitch_on(self.bend1)
        self.upswitch_on(self.bend2)

    PS.CommandExc
    def Off(self):
        self.set_state(DevState.INIT)
        self.set_status('switching off...')
        self.switching = SWITCH_OFF
        self.switch_eta = time() + self.estimate_switch_off_time()
        self.upswitch_off(self.bend1)
        self.upswitch_off(self.bend2)

    def est1_on_t(self, b, c):
        '''estimates how long it takes from the current state to
           bending power supply on, depending on cabinet state c and
           power supply state b.
        '''
	if c in CAB_ON_T:
            t = CAB_ON_T[c]
            if b in BEND_ON_T:
                return t+BEND_ON_T[b]
            else:
                msg = 'can not switch on power supply when in state [%02x,%02x]' % (b,c)
                raise PS.PS_Exception(msg)
        else:
            msg = 'can not switch on cabinet when in state [%02x,%02x]' % (b,c)
            raise PS.PS_Exception(msg)


    def est1_off_t(self, b, c):
        '''estimates how long it takes from the current state to switch
           bending power supply off, depending on cabinet state c and
           power supply state b.
        '''
        return BEND_OFF_T.get(b, 0.0)

    def estimate_switch_on_time(self, mc1=None, mc2=None):
        if mc1 is None: mc1 = self.bend1.mac
        if mc2 is None: mc2 = self.bend2.mac
        on1_t = self.est1_on_t(*mc1)
        on2_t = self.est1_on_t(*mc2)

        return max(on1_t, on2_t)

    def estimate_switch_off_time(self, mc1=None, mc2=None):
        if mc1 is None: mc1 = self.bend1.mac
        if mc2 is None: mc2 = self.bend2.mac
        off1_t = self.est1_off_t(*mc1)
        off2_t = self.est1_off_t(*mc2)
        return max(off1_t, off2_t)

    @PS.CommandExc
    def UpdateState(self):
        if time()<self.next_update_t: return
        self.bend1.mac = self.mac_state(self.bend1)
        self.bend2.mac = self.mac_state(self.bend2)
        self.protect_fault_off()
        try:
            if self.switching == SWITCH_ON:
                fin1 = self.upswitch_on(self.bend1)
                fin2 = self.upswitch_on(self.bend2)
                self.update_switch_status('on', fin1, fin2 )
            elif self.switching == SWITCH_OFF:
                fin1 = self.upswitch_off(self.bend1)
                fin2 = self.upswitch_off(self.bend2)
                self.update_switch_status('off', fin1, fin2)

        except Exception:
            self.switching = SWITCH_NEUTRAL
            raise

        finally:
            if not self.switching in (SWITCH_ON, SWITCH_OFF):
                self.update_neutral()

    @PS.CommandExc
    def PushAttributes(self):
        if time()<self.next_update_t: return
        for aname in self.PUSHED_ATTR_EXTRA:
            try:
                pa = self._read(aname)
                if not pa.value is None:
                        self.push_change_event(aname, *pa.triple)
            except Exception:
                    self.log.error('PushAttr %s', aname, exc_info=1)

    def update_switch_status(self, switch, fin1, fin2):
        if fin1 and fin2 or fin1==FAULT or fin2==FAULT:
            self.switching = SWITCH_NEUTRAL
            self.update_neutral()
        else:
            rem_t = self.switch_eta - time()
            if rem_t > 0:
                self.set_status('switching %s, %.0fs remaining' % (switch, rem_t))
            else:
                self.set_status('switching %s' % switch)


    def update_neutral(self):
        state = DevState.FAULT
        try:
            # defines two shortcuts to improve readability later
            bend1, bend2 = self.bend1, self.bend2

            # first new state is determined
            b1_state, b2_state = bend1.State(), bend2.State()

            # if States are equal it is used as-is
            if b1_state == b2_state:
                state = b1_state

            elif b1_state == DevState.FAULT or b2_state == DevState.FAULT:
                state = DevState.FAULT

            elif b1_state == DevState.ALARM or b2_state == DevState.ALARM:
                state = DevState.ALARM

            elif b1_state == DevState.MOVING or b2_state==DevState.MOVING:
                state = DevState.MOVING

            else:
                state = DevState.ALARM

            b1_status = bend1.Status()
            b2_status = bend2.Status()

            if b1_status==b2_status:
                status = b1_status
            else:
                status = b1_status + ' / ' + b2_status

        except Tg.CommunicationFailed:
                status = 'communication failed'
                self.delay_update()

        except Tg.DevFailed, d:
                status = 'device failure'
                self.delay_update()

        except Exception, e:
            status = str(e)
            self.delay_update()

        finally:
            self.STAT.set_stat2(state, status)

    def is_fault_state(self, b, c):
        return b in BEND_FAULT_STATES or c == CAB_FAULT_STATE

    def delay_update(self, t=10):
          self.next_update_t = time()+t

    def upswitch_on(self, bend):
        '''Switches one bending power supply on.
           Returns True once PS are on otherwise None or False.
           Shall not raise exception when executing any command
        '''
        b,c = bend.mac

        if c == CAB_OFF_STATE:
            bend.cab.On()

        elif c == CAB_ON_STATE and b == BEND_OFF_STATE:
            bend.On()

        elif b == BEND_ON_STATE:
            return True

        elif self.is_fault_state(b,c):
            return FAULT

        # no action is taken until cabinet and power supply reached a 'stable' state: on, off or fault (interlock)
        elif c in CAB_PENDING or b in BEND_PENDING:
            return False

        else:
            self.log.error('not correctly switching on (mac_state = %s,%s)' % (b,c))
            return True

    def upswitch_off(self, bend):
        '''Switches one bending power supply off.
           Returns True once PS are on otherwise None or False.

        '''
        b,c = bend.mac

        # does nothing until cabinet reached a 'stable' state

        if b == BEND_ON_STATE:
            bend.Off()

        elif b == BEND_OFF_STATE:
            return True

        elif self.is_fault_state(b,c):
            return FAULT

        elif c in CAB_PENDING or b in BEND_PENDING:
            return False

        else:
            self.log.error('not correctly switching off (mac_state = %s,%s)' % (b,c))
            return True

    def protect_fault_off(self):
        bend1 = self.bend1
        bend2 = self.bend2
        self.switch_off_on_fault(bend1, bend2)
        self.switch_off_on_fault(bend2, bend1)

    def switch_off_on_fault(self, bendA, bendB):
        b,c = bendA.mac
        if self.is_fault_state(b,c):
            self.switching = SWITCH_NEUTRAL
            b2,c2 = bendB.mac
        if b2 in BEND_POSSIBLY_ON_STATES:
            msg = 'switching off %s because fault [%02x,%02x] in %s' %  (bendB.dev_name(), b,c, bendB.dev_name())
            self.log.info(msg)
            try:
                bendB['TriggerMask'] = 0
            except Exception:
                self.log.error('setting TriggerMask to 0', exc_info=1)
            bendB.Off()

    def mac_state(self, bend):
        baval = bend.read_attribute('MachineState')
        cabaval = bend.cab.read_attribute('MachineState')
        assert baval.quality == AQ_VALID
# TODO: possibly the value could be non if qualiy INVALID
#        assert cabaval.quality == AQ_VALID
        return baval.value, cabaval.value,

    ### Attributes ###
    @PS.AttrExc
    def write_attribute(self, attr):
        try:
          if time()<self.next_update_t:
              raise PS.PS_Exception('devices proxies are not available')
          aname = attr.get_name()
          data = attr.get_write_value()
          self.bend1.write_attribute_asynch(aname, data, nop)
          self.bend2.write_attribute_asynch(aname, data, nop)
        except Tg.DevFailed:
          self.delay_update()

    def read_WaveLength(self, attr):
        self.read_attribute(attr)

    def read_Waveform(self, attr):
        self.read_attribute(attr, self.distill_waveform_quality)

    def write_WaveGeneration(self, attr):
        self.write_attribute(attr)

    def write_Waveform(self, attr):
        self.write_attribute(attr)

    def read_WaveX(self, attr):
        self.read_attribute(attr)

    def write_WaveX(self,attr):
        self.write_attribute(attr)

    def read_Voltage(self, attr):
        def qfun(q, aval1, aval2):
            if q==AQ_VALID:
                return AQ_VALID if abs(aval1.value-aval2.value) < 0.25 else AQ_WARNING
            else:
                return q

        def valfun(aname, aval1, aval2):
            return (aval1+aval2) / 2
        self.read_attribute(attr, qfun, valfun=valfun)

    def read_Current(self, attr):
        self.read_attribute(attr, self.distill_I_quality)

    def write_CurrentSetpoint(self, attr):
        self.write_attribute(attr)

    def read_CurrentSetpoint(self, attr):
        self.read_attribute(attr)

    @PS.CommandExc
    def Sync(self):

        # checks state of power supplies
        for d in self.sync_devices:
            aval = d.read_attribute('MachineState')
            if aval.quality != AQ_VALID:
                raise PS.PS_Exception('MachineState attribute is not valid')
            if aval.value != STATE_SYNC:
                raise PS.PS_Exception('device %r not ready to be sync\'d' % self.get_name())

        # checks and switches off triggers
        evr_state = self.trig.State()
        if not evr_state in (DevState.ON, DevState.OFF):
            raise PS.PS_Exception('event receiver is in %s state, must be ON or OFF')

        try:
            self.trig.Off()

            # sends SYNC commands
            for d in self.sync_devices:
                d.Command( ['SYNC'] )


        finally:
            # re-enables trigger if they were enabled before
            if evr_state == DevState.ON:
                self.trig.On()

    @PS.AttrExc
    def read_Errors(self, attr):
        ls = []
        BE1 = self.bend1.read_attribute('Errors').value
        BE2 = self.bend2.read_attribute('Errors').value
        if BE1 is None:
            BE1 = []

        if BE2 is None:
            BE2 = []

        for e1 in BE1:
            if e1 not in BE2:
                ls.append('bend1: '+e1)
            else:
                ls.append(e1)

        for e2 in BE2:
            if e2 not in BE1:
                ls.append('bend2: '+e2)

        self.errors = ls
        attr.set_value(self.errors)


    def read_RemoteMode(self, attr):
        self.read_attribute(attr)

    def _command2(self, cmd):
        exc = None
        try:
            getattr(self.bend1, cmd)()
        except Exception,exc1:
            self._fault('%s bending 1 failed', cmd, exc_info=1)
            exc = exc1

        try:
            getattr(self.bend2, cmd)()
        except Exception,exc2:
            self._fault('%s bending 2 failed', cmd, exc_info=1)
            exc = exc2

        if exc: raise exc

    @PS.CommandExc
    def ResetInterlocks(self):
        PS.PowerSupply.ResetInterlocks(self)
        self._command2('ResetInterlocks')

    @PS.AttrExc
    def read_WaveStatus(self, attr):
        try:
            av1 = self.bend1['WaveStatus']
            av2 = self.bend2['WaveStatus']

        except Tg.DevFailed, df:
            err = df[-1]
            attr.set_value_date_quality(err.desc, time(), AQ_ALARM)
            return

        except Exception, exc:
            attr.set_value_date_quality(str(exc).desc, time(), AQ_ALARM)
            return


        value = 'not updated'
        timestamp = min(tv2time(av1.time),tv2time(av2.time))
        quality = PS.combine_aq(av1.quality, av2.quality)

        v1, seconds1 = strip_s(av1.value)
        v2, seconds2 = strip_s(av2.value)
        seconds = max(seconds1, seconds2)

        if v1==v2:
            value = v1
            if seconds:
                value += ' %.2fs' % seconds
        else:
            value = v1 + ' / ' + v2
        attr.set_value_date_quality(value, timestamp, quality)

    def read_WaveDuration(self, attr):
        self.read_attribute(attr)

    def read_RegulationFrequency(self,attr):
        self.read_attribute(attr)

    def read_WaveInterpolation(self, attr):
        self.read_attribute(attr)

    def write_WaveInterpolation(self, attr):
        self.write_attribute(attr)

    def read_WaveGeneration(self, attr):
        self.read_attribute(attr)

    def read_WaveOffset(self, attr):
        self.read_attribute(attr)

    def write_WaveOffset(self, attr):
        self.write_attribute(attr)

    def read_WaveName(self, attr):
        self.read_attribute(attr)

    def write_WaveName(self, wattr):
        self.write_attribute(wattr)

    def read_TriggerMask(self, attr):
        self.read_attribute(attr)

    def write_TriggerMask(self, wattr):
        self.write_attribute(wattr)


    def read_WaveId(self, attr):
        self.read_attribute(attr)

class BrukerBend_PS_Class(Tg.DeviceClass):

        FMT = '%6.4f'
        #       Class Properties
        class_property_list = {
                }


        #       Device Properties
        device_property_list = PS.gen_property_list()
        device_property_list.update({
            'Trigger' :
                [ Tg.DevString,
                    "Event receiver sending trigger for bending and quadrupoles", 'bo04/ti/evr-cpc1502-a'],

            'Bend1':
                [Tg.DevString,
                "Name of device controlling bending 1.",
            'bo/pc/bend-1' ],
            'Bend2':
                [Tg.DevString,
                "Name of device controlling bending 2.",
                'bo/pc/bend-2' ],
                'QuadCabinet':
            [Tg.DevString,
            "Name of the quadrupole cabinet, used for Sync command.",
            'bo/ct/pc-q' ],
                })


        #       Command definitions
        cmd_list = PS.gen_cmd_list(opt=('UpdateState',))
        cmd_list['UpdateState'][2]['polling period'] = 20000
        cmd_list.update({
          'PushAttributes' : [ [Tg.DevVoid],[Tg.DevVoid], {'polling period' : 20000 }],
          'On' : [ [Tg.DevVoid],[Tg.DevVoid]],
          'Off' : [ [Tg.DevVoid],[Tg.DevVoid]],
          'Sync' : [ [Tg.DevVoid, 'sync sync'],[Tg.DevVoid],
              { 'display level' : Tg.DispLevel.EXPERT }
          ],
       })

        #       Attribute definitions
        attr_list = PS.gen_attr_list(max_err=100,opt=('Current','CurrentSetpoint','Voltage'))
        attr_list.update({
                'Current1': deepcopy(attr_list['Current']),
                'Current2': deepcopy(attr_list['Current']),
                'Voltage': attr_list['Voltage'],
                'WaveGeneration' : [[ Tg.DevBoolean, Tg.SCALAR, Tg.READ_WRITE ]],
                'Waveform' : [[ Tg.DevDouble, Tg.SPECTRUM, Tg.READ_WRITE, 2**14] , {} ],
                'WaveX' : [[ Tg.DevDouble, Tg.SPECTRUM, Tg.READ_WRITE, 2**14] , {} ],
                'WaveLength' : [[ Tg.DevShort, Tg.SCALAR, Tg.READ], {'unit' : 'points'} ],
                'WaveStatus' : [[ Tg.DevString, Tg.SCALAR, Tg.READ ]],
                'WaveName' : [[ Tg.DevString, Tg.SCALAR, Tg.READ_WRITE ]],
                'WaveId' : [[ Tg.DevLong, Tg.SCALAR, Tg.READ ]],
                'WaveDuration' : [[ Tg.DevDouble, Tg.SCALAR, Tg.READ ], {'unit': 'ms'} ],
                'WaveOffset' : [[ Tg.DevDouble, Tg.SCALAR, Tg.READ_WRITE ], {'unit': 'A'} ],
                'WaveInterpolation' : [[ Tg.DevShort, Tg.SCALAR, Tg.READ_WRITE ], { 'unit': 'log2(periods/point)' } ],
                'RegulationFrequency' : [[ Tg.DevDouble, Tg.SCALAR, Tg.READ ], { 'unit' : 'kHz'} ],
                'TriggerMask': [[Tg.DevLong, Tg.SCALAR,  Tg.READ_WRITE],
                    { 'min value' : 0, 'max value' : 0x3fffffff,
    'description' : '''number of triggers to consider:
0:no wave generated, 1:generate single wave, 0x3fffffff:continously''' }
                    ]}
        )
        attr_list['CurrentSetpoint'][1]['format'] = FMT
        attr_list['Voltage'][1]['format'] = FMT
        attr_list['Current'][1]['format'] = FMT

        #  BrukerBend_PSClass Constructor
        def __init__(self, name):
                Tg.DeviceClass.__init__(self, name)
                self.set_type(name);


if __name__ == '__main__':
    classes = (BrukerBend_PS, )
    PS.tango_main( 'BrukerBend_PS', sys.argv, classes)
