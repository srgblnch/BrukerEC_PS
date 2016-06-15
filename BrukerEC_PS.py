#!/usr/bin/env python
# -*- coding: utf-8 -*-

# BrukerEC_PS.py
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

'''TANGO device server for Bruker EC protocol over TCP.
'''

from __future__ import print_function

META = """
Author: Lothar Krause <lkrause@cells.es>
License: GPL3+
$URL$
$LastChangedBy$
$Date$
$Rev$
"""
# comment for testing merge feature
# python standard imports
import socket
import traceback
from types import StringType
from pprint import pformat, pprint
from copy import deepcopy
from time import time, sleep
from errno import ECONNREFUSED, EHOSTUNREACH
from functools import partial

# TANGO imports
import PyTango as Tg
from PyTango import DevVoid, DevVarStringArray, DevString, DevDouble, \
  DevBoolean, DevShort, DevLong, DevUShort, DevULong, DevState, \
  SPECTRUM, SCALAR, READ_WRITE, READ, Except

# base imports
import PowerSupply.standard as PS
from PowerSupply.standard import AQ_VALID, AQ_INVALID, AQ_CHANGING, AQ_ALARM, \
    AQ_WARNING, DATABASE, VDQ, PseudoAttr
from PowerSupply.util import NAN, bit_filter_msg, UniqList
from PowerSupply import cycle

# relative imports
import wavl
import cabinet
import state
import factory
import tuning


PM = PS.MSG

class WaveDisabled(PS.PS_Exception):
    '''Raised when a wave form related feature that is requested
       and power supply does not support wave forms such as LT, BT
    '''
    pass

class CycleDisabled(PS.PS_Exception):
    '''Raised when a cycle related feature that is requested
       and power supply does not support cycling such as BO
    '''
    pass

class AttributeInvalid(PS.PS_Exception):
    '''Raised when the value an invalid attribute is needed.
    '''
    attribute_name = None
    def __init__(self, aname):
        PS.PS_Exception.__init__(self, repr(aname) + ' is invalid')
        self.attribute_name = aname
        self.stack = traceback.format_stack()
        traceback.print_stack()

# minimum amount of time that a device will be in MOVING state before
# returning to ON state.
MIN_MOVING_TIME = 0.1

# special return value that indicating that a command has been queued
# for later instead of being executed immediately
QUEUING = 'queued command'

def cfg_change_ev(props, aname, abs=None, rel=None):
    '''Updates props for attribute 'aname' for generating change events
       from the numerical abs and rel changes passed.
    '''
    ap =  {'abs_change' : '', 'rel_change' : ''}

    if abs:
        ap['abs_change'] = str(abs)

    if rel:
        ap['rel_change'] = str(rel)
    props[aname] =  ap

def cfg_ev_ps(dev):
    '''Configures the events of power supply Tango device.
    '''
    props = {}
    cfg_change_ev(props, 'Current', abs=0.005)
    cfg_change_ev(props, 'CurrentSetpoint', abs=0.005)
    cfg_change_ev(props, 'Voltage', abs=0.05)
    cfg_change_ev(props, 'MachineState', 0.1)
    cfg_change_ev(props, 'ErrorCode', 0.1)
    cfg_change_ev(props, 'WaveOffset', 0.1)
    DATABASE.put_device_attribute_property(dev, props)

def cfg_events(serv):
    '''Configures the events of the devices of 'serv' unless
       ConfigureEvents property of cabinet devices for server 'serv'
       is 0.
    '''
    # first run, yes
    want_cfg = True

    # finds name of the cabinet device
    cab_name = None
    dcl = DATABASE.get_device_class_list(serv)
    dcl = zip(dcl[::2], dcl[1::2])
    for cab_name, cl in dcl:
        if cl=='BrukerEC_Cabinet':
            break

    if cab_name is None:
        print('warning: no cabinet device defined')
        return

    # checks whether event configuration is wanted?
    props = DATABASE.get_device_property(cab_name, ('ConfigureEvents', ) )
    p = props.get('ConfigureEvents', ['1'])
    want_cfg = bool(int(p[0])) if p else True

    # leaves if no event configuration is needed
    if not want_cfg: return
    print('configuring change events...')
    for d,c in dcl:
        if c=='BrukerEC_PS':
            cfg_ev_ps(d)

    # disables the event configuration for future runs
    DATABASE.put_device_property(cab_name, {'ConfigureEvents':'0'})

class BrukerEC_PS(PS.PowerSupply):
    '''Representing individual power supplies.
    '''

    # device properties
    Port = None
    WaveName = ''

    tuner = None
    wave_load = None
    wave_load_staging = None
    __errors = []
    
    #cycling vbles
    _isCycling = False
    #__cycling_minValue = None
    #__cycling_maxValue = None
    #__cycling_setp = None
    #__cycling_step = None
    #__cycling_stepTimestamp = 0.0
    #stepWaitTime = 30#s FIXME: this value should be a property, 
    #                           but now can be mod using Exec()
    _cycleThread = None
    _backup_setpoint = None #FIXME: hackish to force to remember the current setpoint after a power off

    PUSHED_ATTR = (
        'Current', 'CurrentSetpoint', 'Voltage',
        'State', 'Status', 'RemoteMode', 'MachineState', 'ErrorCode',
        'WaveGeneration', 'WaveOffset',)
        #'WaveStatus',

    def __init__(self, cl, name):
        PS.PowerSupply.__init__(self, cl, name)
        self.init_device(cl, name)

    def init_device(self, cl=None, name=None):
        PS.PowerSupply.init_device(self, cl, name)

        # sets up cache of VDQ objects for safely manipulating and accessing
        # read values from within the device itself
        self.cache = dict(
            WaveGeneration = VDQ(False),
            WaveLength = VDQ(0),
            WaveDuration = VDQ(0.0),
            WaveStatus = VDQ(wavl.READY, q=AQ_VALID),
            WaveId = VDQ(0, q=AQ_INVALID),
            WaveName = VDQ(self.WaveName, q=AQ_VALID),
            Current = VDQ(NAN),
            CurrentSetpoint = VDQ(NAN),
            I = VDQ(NAN),
            V = VDQ(NAN),
            Voltage = VDQ(NAN),
            MachineState = VDQ(0),
            ErrorCode = VDQ(0),
        )
        self.cab = cabinet.instance()
        self.wavl = wavl.instance()

        self._pstype = None
        self.update_cycle = 0
        self.cab.updater.add(self.UpdateState)
        self.STAT.INITIALIZED()
        self.ERR_CODE = [ VDQ(0) ]
        self.STC_CODE = [ VDQ(0) ]

        # aid in troubleshooting TCP socket related problems
        self.socket_error_counter = 0


    # Basic Utilities
    def query(self, aname):
        '''Executes query to hardware for attribute 'aname' by calling the
           returning the result from calling the query_<aname> method
        '''
        qfun_name = 'query_'+aname
        try:
            return getattr(self, qfun_name)()
        except AttributeError:
            traceback.print_exc()
            msg = 'no method %r for querying %r'  % (qfun_name, aname)
            raise Exception(msg)

    def vdq_set(self, attr, **kwargs):
        '''Shortcut for self.vdq(attr.get_name()).set_attr(attr)
        '''
        vdq = self.vdq(attr.get_name(), **kwargs)
        vdq.set_attr(attr)
        return vdq

    def push_vdq(self, aname, *args, **kwargs):
        '''Updates the cached value of aname and generates an
           apropiate change event, returning the updated VDQ.
        '''
        vdq = self.cache.get(aname)
        if vdq is None:
            vdq = self.cache[aname] = VDQ(*args, **kwargs)
        else:
            vdq.update(*args, **kwargs)
        self.push_change_event(aname, *vdq.triple)

    def vdq(self, aname, query='auto', dflt=None, dflt_q=AQ_INVALID):
        '''Returns a VDQ object for aname, or throws an exception.
            @param aname attribute name for which vdq object is needed
            @param query whether the value should be queried from hardware can be
                        'auto', which will execute the query when necessary, or
                        'force' will always queried, or
                        'never' means that an exception will be generated if
                        unavdailable.
            @param dflt default value to return if value not in cache.
        '''
        if isinstance(aname, Tg.Attribute):
            aname = aname.get_name()

        if query=='force' or (query=='auto' and aname not in self.cache):
            vdq = self.query(aname)
            self.cache[aname] = vdq

        elif aname not in self.cache and dflt is not None:
            vdq = VDQ(dflt, q=dflt_q)

        else:
            vdq = self.cache[aname]

        return vdq

    def update_attr(self, aname):
        '''Updates attribute 'aname' by first querying its vdq,
           than saving it to the corresponding Attribute object and
           generating change events.
           Note that attributes that are updated in this manner must not
           be polled, in order to avoid concurrency problems of TANGO p
           polling thread, client threads and others
            @param aname which attribute to query and update
        '''

        vdq = self.cache[aname] = self.query(aname)
        if vdq.value is None:
            self._alarm('attempt to push change event for attribute %r w/o' +
              'value' % aname
            )
        else:
            self.push_change_event(aname, *vdq.triple)
        return vdq.value

    def value(self, aname, dflt=None, query='auto'):
        '''Returns the value of aname, or throws an Exception.
           @param aname name of the attribute desired
           @param query will be passed to @see vdq
           @param dflt passed on to @see vdq
        '''
        vdq = self.vdq(aname, query=query, dflt=dflt)
        if vdq.quality == AQ_INVALID:
            if dflt is None:
                raise AttributeInvalid(aname)
            else:
                return dflt
        return vdq.value

    def pstype(self):
        """Returns PSType object for power supply.
        """
        if self._pstype: return self._pstype

        # By executing the method it is also guranteed that certain
        # tasks (detecing CurrentNominal) have succeeded
        type_code = self.query('Version').value[0]
        if type_code==state.PSTYPE_CODE_QUAD:
            brk_config2 = self.ObjInt(state.COBJ_BRK_CONFIG2)
            if brk_config2==state.BRKconfig2_MASTER:
                pst = deepcopy(state.PSTYPE_BIG_QUAD)
            elif brk_config2 in state.BRKconfig2_STANDALONES:
                pst = deepcopy(state.PSTYPE_SMALL_QUAD)
            else:
                msg = 'bad Port %s configured, can not be slave module'%\
                      self.Port
                raise PS.PS_Exception(msg)
        else:
            pst = deepcopy(state.REG2PSTYPE[type_code])

        if pst is None:
            msg = 'failed to detect suitable PS, port %d has type code %d' \
                % (self.Port, type_code)
            self.cab.stop()
            raise PS.PS_Exception(msg)

        # updates messages related to external interlocks
        xi_msg = [ getattr(self,'Interlock%d' % d) for d in range(1,5) ]
        pst.update_xi(xi_msg)

        obj_cmd = 'OBJ='+hex(state.COBJ_MAIN_IREF_SCALED)
        rs = self.cmd_seq(obj_cmd,'ymax/','xmax/')
        ymax = float(rs[1])
        if rs[2].startswith('0x'):
            xmax = int(rs[2],16)
        else:
            xmax = float(rs[2])

        # nominal current is fixed to be the following...
        # 32 bit signed integer values
        XNOMINAL = 0x3FFFFFFF
        # determines the y nominal using the relationship between
        # xmax and xnominal
        tri = VDQ(round(XNOMINAL/xmax * ymax),q=AQ_VALID)
        self.cache['CurrentNominal'] = tri

        self.get_nominal_y()
        # using self.use_waveform is valid here because the
        # cabinet type is detected when that cabinet device connects
        # the socket and the Version query above ensure that this has
        # happened
        if self.cab.use_waveforms:
            self.tuner = tuning.Tuner(self, pst)

        # add extra attributes for ErrorCode where necessary
        for idx,mod in enumerate(pst.module[1:]):
            self.ERR_CODE.append(VDQ(0))
            self.STC_CODE.append(VDQ(0))
            def read_ErrorCodeX(inst, attr):
                self.ERR_CODE[idx].set_attr(attr)

            attrName = 'ErrorCode'+str(idx+1)
            attr = Tg.Attr(attrName, Tg.DevLong, Tg.READ)
            try:
                self.add_attribute(attr, r_meth=partial(read_ErrorCodeX, 1))
            except Exception,e:
                self.debug_stream("Dynamic attr %s fail to build using partial"
                                  %(attrName))
                self.add_attribute(attr, r_meth=read_ErrorCodeX)

        # after this line the pstype is considered fully detected
        self._pstype = pst

        # sets last written value for writeable attributes to their
        # current setting
        try:
            self.UploadWaveform()

            cur = float(self.cmd_seq('CUR/')[0])
            self._attr('CurrentSetpoint').set_write_value(cur)

            ramp = self.update_attr('CurrentRamp')
            self._attr('CurrentRamp').set_write_value(ramp)

            self.update_attr('RegulationFrequency')

            if self.cab.use_waveforms:
                  wgen = self.update_attr('WaveGeneration')
                  self._attr('WaveGeneration').set_write_value(wgen)

                  wint = self.update_attr('WaveInterpolation')
                  self._attr('WaveInterpolation').set_write_value(wint)

                  wof = self.update_attr('WaveOffset')
                  self._attr('WaveOffset').set_write_value(wof)


        except Exception, exc:
            self.log.exception(str(exc))

        self.cab.init_counter += 1
        return self._pstype

    @PS.ExceptionHandler
    def delete_device(self):
        PS.PowerSupply.delete_device(self)
        self.cab.updater.remove(self.UpdateState)

    ## Tango Commands ##
    @PS.CommandExc
    def Command(self, command_list):
        '''Executes a list of EC commands, returing a list of responses for
           each command.
        '''
        return self.cmd_seq(*command_list)

    def cmd(self, *commands, **kwargs):
        q = kwargs.get('q',False)
        return '\n'.join(c for c in self.cmd_seq(*commands, q=q) if c)

    def cmd_seq(self, *commands, **kwargs):
        '''Executes a list of EC commands, returing a list of responses for
           each command.
        '''
        q = kwargs.get('q',False)
        try:
            if self.wavl.busy and q:
                self.cab.command_q(self.STAT, self.Port, *commands)
                return QUEUING
            else:
                return self.cab.command_seq(self.Port, *commands)

        except socket.timeout:
            msg = 'control unit %r not responding' % self.cab.host
            raise PS.PS_Exception(msg)

        except socket.error, exc:
            e = exc.args[0]
            if e==EHOSTUNREACH:
                msg = 'control unit %r offline' % self.cab.host
            else:
                msg = 'control unit %r: %s (%d)' % (self.cab.host, exc.args[1], e)
            raise PS.PS_Exception(msg)
        except cabinet.CanBusTimeout,exc:
            # self.STAT.FAULT('Socket exception: %s'%(exc.args))
            # Not necessary to decay to fault, only alert that it have happen.
            self._alarm(str(msg))
        except Exception,e:
            msg = 'control unit %r: Exception! Review comunications' % (self.cab.host)
            raise PS.PS_Exception(msg)

    def post_load(self):
        """Clean up to be done after download failed or succeeded
        """

    def load_waveform(self):
        '''Executes up - and downloads of waveforms.
        '''
        if not self.cab.use_waveforms:
            self.push_vdq('WaveStatus', 'waveforms not supported', q=AQ_WARNING)
            return
        if not self.wavl.is_connected:
            self.push_vdq('WaveStatus', 'error: not connected', q=AQ_WARNING)
            return
        if not self.value('RemoteMode', dflt=False):
            self.push_vdq('WaveStatus', 'error: local mode', q=AQ_WARNING)
            return
        local_wave_load = self.wave_load
        if local_wave_load is None: return
        try:
            # invalidates previous Waveform read value
            self.push_vdq('Waveform' , [], q=AQ_CHANGING)
            msg = local_wave_load.BASE_MSG+'ing...'
            self.push_vdq('WaveStatus', msg, q=AQ_CHANGING)
            self.STAT.BUSY(msg)
            sleep(0.25)
            start_t = time()
            local_wave_load.run(self, self.wavl)
            t0 = time()
            self.wave_load = None
            try:
                self.update_attr('WaveLength')
            except PS.PS_Exception:
                self.log.warn('failed to update WaveLength after loading waveform',
                              exc_info=1)
            try:
                self.update_attr('WaveDuration')
            except PS.PS_Exception:
                self.log.warn('failed to update WaveDuration after loading waveform',
                              exc_info=1)
            Inominal = self.get_nominal_y()[1]
            waveform = self.value('Waveform')
            wavehash = wavl.calc_hash(Inominal, waveform)
            self.push_vdq('WaveId', wavehash, d=t0, q=AQ_VALID)
            diff_t = t0-start_t
            msg = local_wave_load.BASE_MSG+' finished %.2fs' % diff_t
            self.push_vdq('WaveStatus', msg, q=AQ_VALID)
            self.STAT.BUSY(msg)

        except wavl.LoadException, exc:
            msg = local_wave_load.BASE_MSG+": "+str(exc)
            self.STAT.FAULT(msg)
            self.cache['Waveform'] = VDQ([], q=AQ_INVALID)
            self.push_vdq('WaveStatus', msg, q=AQ_ALARM)

        except PS.PS_Exception, exc:
            msg = local_wave_load.BASE_MSG+' error: '+str(exc)
            self.cache['Waveform'] = VDQ([], q=AQ_INVALID)
            self.push_vdq('WaveStatus', msg, q=AQ_ALARM)
            self.log.debug('while loading waveform', exc_info=1)

        except socket.error, err:
            err_msg = str(err)
            self._alarm('waveforms %s' % err_msg)
            ws = local_wave_load.BASE_MSG+' failed: '+err_msg
            self.push_vdq('WaveStatus', ws, q=AQ_ALARM)

        # handling generic faults during wave up- and download
        except Exception, exc:
            msg = str(exc)
            self._fault(msg, exc_info=1)
            msg = local_wave_load.BASE_MSG+' failed: '+msg
            self.cache['Waveform'] = VDQ([], q=AQ_INVALID)
            self.push_vdq('WaveStatus', msg, q=AQ_ALARM)

        finally:
            sleep(0.75)
            self.cab.process_command_queue()
            self.cab.update()


    @PS.CommandExc
    def UpdateState(self):
        '''Updates the state of device.
        '''
        # nothing happens until next ResetInterlock
        if self.get_state()==DevState.FAULT:
            return

        try:
            STAT = self.STAT
            # resets alarms
            self._up_start_t = time()

            # performs wave up/down-loads first, if any
            self.load_waveform()
            cab = self.cab
            if not cab.is_connected:
                if cab.comm._exc_msg:
                    STAT.COMM_ERROR(self.cab.comm._exc_msg)
                else:
                    msg = 'no connection to control unit ' + str(cab.host)
                    STAT.COMM_ERROR(msg)
                return

            # detects power supply specific settings
            t = self.pstype()

            # status update
            # disabled this check because it causes problems whe
            # starting up DS for a cabinet with hung module
#            if not self.cab.all_initialized():
#                return

            # always updates Current readback
            Imeas = self.update_attr('Current')

            STEP = 5
            # local state, interlock
            def up(aname, phase=0, dflt=None):
                if self.update_cycle % STEP == phase:
                    return self.update_attr(aname)
                else:
                    return self.value(aname, dflt=dflt)


            # every fourth step one of the following is updated
            REM = up('RemoteMode',1, dflt=False)
            WMO = up('WaveGeneration',2, dflt=False)
            Iref = up('CurrentSetpoint',3, dflt=NAN)

            # needed to have responsive bend interface
            if self.cab.use_waveforms:
                up('WaveOffset', 4, dflt=NAN)

            # updates Voltage only in DC mode
            ps_on = self.is_power_on()
            # only needed for historic
            if WMO:
                moving = False
            else:
                U = up('Voltage', 4, dflt=NAN)
                Inominal = self.value('CurrentNominal')
                moving = ps_on and abs(Iref-Imeas) > self.RegulationPrecision

            # always query STC (errors) and STA (machine state) for ErrorCode
            MODULENUM = len(t.module)
            alarms = UniqList()
            for idx,mod in enumerate(t.module):
                port = self.Port+idx
                start_t = time()
                mod.error_code = cab.checked_command(port, 'STA/')
                state_start_t = time()
                mod.state_id = cab.checked_command(port, 'STC/')
                state_t = (state_start_t+time()) / 2.0
                code_t = (start_t+time()) / 2.0
                code = state.synthesize_error_code(mod.state_id, mod.error_code)
                mod.update_stat2()
                self.ERR_CODE[idx].update(v=code, d=code_t, q=AQ_VALID)
                if MODULENUM>1:
                    alarms += [ '['+mod.name+'] '+ m for m in mod.alarms ]
                else:
                    alarms += mod.alarms

            self.update_attr('ErrorCode')
            ps_state_id = self.update_attr('MachineState')
            alarms += self.cab.alarms
            te,tus = cab.stat[:2]

            # kludges around the fact that cabinet may indicate
            # cabinet water flow being faulty but correctors and LT quadrupoles
            # will not care
            if t.name=='corrector' and PM.CABINET_WATER in alarms:
                alarms.remove(PM.CABINET_WATER)

            if t.name=='corrector' and PM.DOOR in alarms:
                alarms.remove(PM.DOOR)

            # decides which state should be used
            # FAULT overrides everything else until Reset(Interlocks)
            if self.get_state()==DevState.FAULT:
                pass

            #Status message is managed in the cycling thread
            elif self.is_cycling():
                if self._cycleThread == None or self._cycleThread.isCyclingDone():
                    self._isCycling = False
                    STAT.ON()

            # current errors will cause ALARM
            elif alarms:
                STAT.ALARMS(alarms)

            # check for moving state
            elif moving:
                STAT.CURRENT_ADJUST()

            elif ps_on:
                STAT.ON()

            elif ps_state_id in t.states_switch_on:
                STAT.SWITCHING_ON()

            elif ps_state_id in t.states_switch_off:
                STAT.SWITCHING_OFF()

            elif cab.state_id is not None:
                te,tus = cab.stat[0:2]
                if te==Tg.DevState.STANDBY or te==Tg.DevState.ON:
                    STAT.OFF()

                elif te==Tg.DevState.OFF:
                    STAT.OFF(what='cabinet')

                else:
                    STAT.OFF(extra=tus)

            else:
                STAT.OFF()

        except socket.error, m:
            STAT.COMM_ERROR(m)
            self.socket_error_counter += 1

        except cabinet.CanBusTimeout, m:
            # STAT.COMM_FAULT(m)  # Not necessary to decay to fault,
            self._alarm(str(msg))  # only alert that it have happen.

        except PS.PS_Exception, exc:
            self._alarm(str(exc))

        finally:
            errors = UniqList()
            errors += self.faults
            errors += self.alarms
            self.__errors = errors
            self.update_cycle += 1
            self._up_end_t = time()
            self._up_diff_t = self._up_end_t-self._up_start_t

    @PS.CommandExc
    def CabinetOn(self):
        self.cab.power_on()

    @PS.CommandExc
    def CabinetOff(self):
        self.cab.power_off()

    @PS.CommandExc
    def Off(self):
        try:
            if self.cab.use_waveforms and self.value('WaveGeneration'):
                self._write('TriggerMask',0)
        except Exception:
                self.log.error('while switching wave cabinet off', exc_info=1)
        self._backup_setpoint = self.cache['CurrentSetpoint'].value
        qstat = self.STAT if self.wavl.busy else None
        self.cab.switch_power(self.Port, self.DCP_Bit, qstat=qstat)

    @PS.CommandExc
    def On(self):
        # uses 'local' trigger mode
        if not self.wavl.busy:
            self.cmd('WTS=0')
        qstat = self.STAT if self.wavl.busy else None
        self.cab.switch_power(self.Port, self.DCP_Bit+1, qstat=qstat)
        self.cache['TriggerMask'] = VDQ(0,q=AQ_VALID)
        #check if the PS have lost the setpoint during the Off time
        try:
            if not self._backup_setpoint == None and \
               self._read('CurrentSetpoint').value != self._backup_setpoint:
                self._write('CurrentSetpoint',self._backup_setpoint)
        except Exception,e:
            self.log.error('while trying to remember previous setpoint when switch on', exc_info=1)

    @PS.CommandExc
    def ResetInterlocks(self):
        PS.PowerSupply.ResetInterlocks(self)
        self.__errors = self.faults+self.alarms
        self.cab.reconnect(reap=True)
        self.wavl.reconnect_reap()
        self.cab.can_hang[self.Port] = False
        self.cab.reset_interlocks()
        self.cmd('RST=0')
        self.cache['WaveStatus'].value = wavl.READY

    @PS.CommandExc
    def UploadWaveform(self, maxlen=None):
        if not self.cab.use_waveforms: return
        self._check_use_waveforms()
        if self.is_ramping():
            desc = 'Not allow to upload waveform during ramping'
            self.log_debug(desc)
            raise PS.PS_Exception(desc)
            return
        self.log.debug('UploadWaveform!')
        if 'Waveform' in self.cache:
            del self.cache['Waveform']
        n = self.update_attr('WaveLength')
        local_wave_upload = self.wave_load = wavl.Upload(self.Port, n)
        self.push_vdq('WaveLength', q=AQ_CHANGING)
        self.push_vdq('WaveDuration', q=AQ_CHANGING)
        self.push_vdq('WaveStatus', local_wave_upload.BASE_MSG+' pending', q=AQ_CHANGING)

    @PS.CommandExc
    def Cycle(self):
        self._check_use_cycle('Cycle')
        if self._isCycling:
            #desc = "Magnet is already cycling."
            #raise PS.PS_Exception(desc)
            pass#no message if it's already cycling
            self._append_new_cycling()
            self._set_cycling_trace("Cycle() called but already cycling")
        elif not self.get_state() in [DevState.ON,DevState.MOVING]:
            desc = "Magnet can only be cycled in on state."
            self._append_new_cycling()
            self._set_cycling_trace("Cycle() called but %s"%desc)
            raise PS.PS_Exception(desc)
        else:#if not self._isCycling:
            self._append_new_cycling()
            self.log.debug('Cycling')
            multiattr=self.get_device_attr()
            setpointAttr = multiattr.get_attr_by_name('CurrentSetpoint')
            try:
                currentAttr = self.query('Current')
            except Tg.DevFailed, e:
                Except.re_throw_exception(e,
                                       "Cycle command failed",
                                       "Not possible to launch cycling procedure",
                                       "Cycle()")
                
            try:
                Imax = float(setpointAttr.get_max_value())
                Imin = float(setpointAttr.get_min_value())
            except Tg.DevFailed, e:
                Except.re_throw_exception(e,
                                       "Cycle command failed",
                                       "Non usable range defined for cycling",
                                       "Cycle()")
            #start the thread who will do the cycling
            #backup the current setpoint to go back at the end of the cycling procedure
            finalSetp = self.vdq('CurrentSetpoint').value#setpointAttr.get_write_value()
            #FIXME: change the state and message in the status
            self._cycleThread = cycle.CycleMagnet(Imax,Imin,finalSetp,
                                                  self._read,self._write,
                                                  self.STAT,self.log,
                                                  self._set_cycling_trace)
            self.log.debug('Cycle min=%6.3f max=%6.3f setpoint=%6.3f'%
                           (Imin,Imax,finalSetp))
            self._set_cycling_trace("Cycle() called with %6.3f current: "\
                                    "min=%6.3f and max=%6.3f"
                                    %(finalSetp,Imin,Imax))
            self._isCycling = True
            self._cycleThread.start()
    
    @PS.CommandExc
    def AbortCycle(self):
        self._append_new_cycling()
        self._set_cycling_trace("AbortCycle() called")
        self._check_use_cycle('AbortCycle')
        #TODO: if is cycling, send a message to the thread to abort it.
        if self._isCycling and not self._cycleThread == None:
            self._cycleThread.abort()
        else:
            Except.throw_exception("AbortCycle command failed",
                                   "No cycling to be aborted",
                                   "AbortCycle()")

    @PS.CommandExc
    def ObjInt(self, cobj):
        rs = self.cmd_seq('OBJ=%d' % cobj, 'VAL/','SCALED/')
        if rs[0]=='*':
            raise PS.PS_Exception('time out or undefined can bus object %x'%cobj)
        else:
            if rs[1]=='*':
                raise PS.PS_Exception('cobj %x not responsive' % cobj)
            else:
                return int(rs[1],16)

    def obj_vdq(self, cobj, idx=0, conv=float):
        if self.wavl.busy:
            msg = 'obj_vdq: unable to read %04x while %s' % (cobj, self.wavl.busy)
            raise PS.PS_Exception(msg)

        port = self.Port+idx
        rs = self.cab.command_seq(port, 'OBJ=%d' % cobj, 'VAL/')
        if rs[0]=='*':
            raise PS.PS_Exception('time out or undefined can bus object %x' % cobj)
        else:
            if rs[1]=='*':
                raise PS.PS_Exception('cobj %x not responsive' % cobj)
            else:
                return VDQ(conv(rs[1]), q=AQ_VALID)

    def obj_set(self, cobj, idx, value):
        port = self.Port+idx
        self.cab.command_seq(port, 'OBJ=%d' % cobj, 'VAL='+str(value))

    @PS.CommandExc
    def ObjFloat(self, cobj):
        rs = self.cmd_seq('OBJ=%d' % cobj, 'VAL/','SCALED/')
        if rs[0]=='*':
            raise PS.PS_Exception('time out or undefined can bus object %x' % cobj)
        else:
            if rs[1]=='*':
                raise PS.PS_Exception('cobj %x not responsive' % cobj)
            else:
                return float(rs[1])

    @PS.CommandExc
    def ReadMachineStates(self):
        t = self.pstype()
        codes = [ '????????'] * len(t.module)
        for idx,mod in enumerate(t.module):
            port = self.Port+idx
            c = codes[idx] = self.cab.command(port, 'STC/')
        return " ".join(codes)

    ## is_xyz functions ##
    def is_dc_mode(self):
        return not self.value('WaveGeneration', dflt=False)

    def is_power_on(self):
        t = self.pstype()
        return self.value('MachineState', dflt=state.STATE_NONE) in t.states_on

    def is_ramping(self):
        '''Returns True when the power supply is ramping,
           False if not ramping, None if not sure.
        '''
        try:
            if not self.is_power_on():
                return False
            # only reach this line if power on

            wg = self.vdq('WaveGeneration')
            if wg.quality==AQ_VALID:
                if wg.value is False:
                    return False
            else:
                return None
            # line only reached, when power on AND wave generation on

            if self.pstype().has_trigger_mask:
                tm = self.vdq('TriggerMask')
                if tm.quality==AQ_VALID:
                    return tm.value!=0
                else:
                    return None
            else:
                return True
        except Exception:
            self.log.exception('is_ramping')

    def is_cycling(self):
        '''Returns True when the power supply is doing a cycling procedure.
        '''
        return self._isCycling

    def is_WaveGeneration_allowed(self, write):
        return not write or not self.is_power_on()

    ## Attributes ##
    def query_RemoteMode(self):
        return self.cab.rem_vdq

    def query_Current(self):
        vdq = self.query_ec('ADC', float)
        try:
            if self.is_ramping() and vdq.quality==AQ_VALID:
                vdq.quality = AQ_CHANGING
        except Exception:
            self.log.error('query_Current', exc_info=1)
        return vdq

    @PS.AttrExc
    def read_Current(self, attr):
        self.vdq('Current').set_attr(attr)

    @PS.AttrExc
    def read_CurrentSetpoint(self, attr):
        vdq = self.vdq_set(attr, query='never', dflt=NAN)
        try:
            attr.set_write_value(vdq.value)
        except Tg.DevFailed, df:
            reason = df[0].reason
            # ignore when setpoint readback is outside limits
            if reason=='API_WAttrOutsideLimit':
                pass
            else:
                raise

    def query_CurrentDelta(self):
        I = self.vdq('Current')
        Iset = self.vdq('CurrentSetpoint')
        delta = Iset.value - I.value
        t = max(I.date, Iset.date)
        q = PS.combine_aq(I.quality, Iset.quality)
        return VDQ(delta,t,q)

    @PS.AttrExc
    def read_Version(self, attr):
        self.vdq_set(attr)

    def query_Voltage(self):
        if self.is_dc_mode():
            return self.pstype().query_Voltage(self)
        else:
            return VDQ(0.0, q=AQ_INVALID)

    def query_CurrentNominal(self):
        self.pstype()
        return self.cache['CurrentNominal']

    @PS.AttrExc
    def read_Waveform(self, attr):
      self.vdq_set(attr)

    def query_Waveform(self):
        if 'Waveform' in self.cache:
            vdq = self.cache['Waveform']
            if self.wave_load:
                vdq.quality = AQ_CHANGING
        else:
            vdq = self.cache['Waveform'] = VDQ([], q=AQ_INVALID)
            # if no upload was initiated so far, start one now
            # possibly this should be done by the init_device thing
            if self.wave_load is None:
                self.UploadWaveform()
        return vdq

    # WaveX related features
    def query_WaveX(self):
        N = len(self.value('Waveform'))
        return VDQ(self.get_regulation_x(n=N), q=AQ_VALID)

    @PS.AttrExc
    def read_WaveX(self,attr):
        ## glued to be depending on RegulationFrequency
        self.vdq_set(attr, query='force')
        # returns only what has been written
        ##          self.vdq_set(attr, query='none')


    @PS.AttrExc
    def read_WaveY(self,attr):
        self.vdq_set(attr, query='none', dflt=[])

    def _write_wave_z(self,attr):
        aname = attr.get_name()
        data = attr.get_write_value()
        vdq = self.cache[aname] = VDQ(data,q=AQ_VALID)
        vdq.set_attr(attr)
        # self.store_wave()

    def _check_use_waveforms(self, msg=None):
        '''Checks wether waveform features are available with this power supply.
           If not a WaveUnsupported exception is thrown
        '''
        use = self.cab.use_waveforms
        if use is False:
            if msg is None:
                fun = traceback.extract_stack(limit=1)[-1][3]
                msg = fun
            raise WaveDisabled(msg)

    def _check_use_cycle(self,msg=None):
        '''Checks if cycling feature is available with this power supply.
           If not supported CycleDisabled exception is thrown
        '''
        if self.is_ramping():
            desc = 'Not allow to cycle during ramping mode'
            self.log_debug(desc)
            raise CycleDisabled(msg)

    ### Wave Attributes ###
    @PS.AttrExc
    def write_WaveGeneration(self, attr):
        self.log.debug('writing wave generation')
        self._check_use_waveforms('WaveGeneration')
        if self.is_cycling():
            desc = 'Not allow to start waveform generation during cycling'
            self.log_debug(desc)
            raise PS.PS_Exception(desc)
            return
        val = attr.get_write_value()
        self.cmd('WMO=%s' % int(val), q=True)
        if not self.wavl.busy:
            self.update_attr('WaveGeneration')
            # pushing an INVALID voltage
            # in case wave generation is not immediately updated
        if val:
            self.push_vdq('Voltage', q=AQ_INVALID)
        else:
            vdq = self.pstype().query_Voltage(self)
            self.push_vdq('Voltage', *vdq.triple)


    @PS.AttrExc
    def write_WaveX(self,attr):
        self._check_use_waveforms('WaveX')
        self._write_wave_z(attr)

    @PS.AttrExc
    def write_WaveY(self,attr):
        self._check_use_waveforms('WaveY')
        self._write_wave_z(attr)

    @PS.AttrExc
    def read_WaveStatus(self, attr):
        self.vdq_set(attr)

    @PS.AttrExc
    def read_Iwave(self, attr):
        self._check_use_waveforms('Iwave')
        try:
            wln = self.value('WaveLength', query='force')
            wave = self.wavl.upload(self.Port, wln)
            attr.set_value(wave)
        except PS.PS_Exception:
                attr.set_quality(AQ_INVALID)
                raise

    @PS.AttrExc
    def write_Iwave(self, wattr):
        self._check_use_waveforms('Iwave')
        raw_wave = wattr.get_write_value()
        xmin = -wavl.PT_MAX
        xmax = wavl.PT_MAX
        if min(raw_wave) < xmin or max(raw_wave) > xmax:
            raise PS.PS_Exception('input data out of range')
        wave = self.wavl.download(self.Port, raw_wave)

    @PS.ExceptionHandler
    def push_wave_up(self, up, raw_data):
        '''Called after waveform has been uploaded.
        '''
        self.log.debug('push wave up for %s, %d pt', up, len(raw_data))

        ymin,ymax = self.get_nominal_y()
        xmax = float(wavl.PT_NOMINAL)

        iy = [ i / xmax * ymax for i in raw_data ]
        t0 = time()
        self.cache['Waveform'] = VDQ(iy, t0, AQ_VALID)
        try:
                ix = self.get_regulation_x(n=len(iy))
                self.cache['WaveX'] = VDQ(ix, t0, AQ_VALID)
        except Exception:
                self.log.error('push_wave_up', exc_info=1)


    def push_wave_down(self, waveform):
        '''Called after waveform has been downloaded.
        '''
        t0 = time()
        self.cache['Waveform'] = VDQ(waveform, t0, AQ_VALID)

    def get_regulation_x(self, n=None, t=None):
        '''Returns an array of points in time corresponding to the actual regulation
           frequency.
        '''
        ppp = 2**self.value('WaveInterpolation')
        dt = ppp/self.value('RegulationFrequency')/1000.0
        if n is None:
            n = t / dt + 1
        ix = tuple( dt*r for r in xrange(n) )
        return ix

    @PS.AttrExc
    def read_Errors(self, attr):
        attr.set_value(self.__errors)

    def query_Errors(self):
        return VDQ(self.__errors, q=AQ_VALID)

    def check_waveform_input(self, wave):
        # checks whether input wave could cause problems for the power supply
        # currently no check is defined
        return

    @PS.AttrExc
    def write_Waveform(self, wattr):
        self._check_use_waveforms('attribute Waveform')
        if not self.value('RemoteMode'):
            raise PS.PS_Exception('no waveform download when in local mode')

        waveform = wattr.get_write_value()
        self.check_waveform_input(waveform)
        YBOTTOM, I_nominal = self.get_nominal_y()
        raw_data = wavl.to_raw(I_nominal, waveform)
        local_wave_download = self.wave_load = wavl.Download(self.Port, waveform, raw_data)
        ix = self.get_regulation_x(n=len(waveform))

        self.cache['WaveX'] = VDQ(ix, q=AQ_CHANGING)
        self.cache['WaveY'] = self.push_vdq('Waveform', waveform, q=AQ_CHANGING)
        self.push_vdq('WaveLength', q=AQ_CHANGING)
        self.push_vdq('WaveDuration', q=AQ_CHANGING)
        self.push_vdq('WaveId', q=AQ_CHANGING)
# do not push writeable DevStrings until Tiago updated to 7.2
#        self.push_vdq('WaveName', v=self.value('WaveName',''), q=AQ_ALARM)
        self.push_vdq('WaveStatus', local_wave_download.BASE_MSG+' pending', q=AQ_CHANGING)

    def ramp_off(self):
        '''ensures that ps is not ramped after calling the function,
           or exception is thrown.
        '''
        if self.pstype().has_trigger_mask:
            #WST=value: for PS with wf value means who many triggers will be accepted
            self.cmd('WTR=0')#value 0, means stop listen triggers
            self._write('TriggerMask', 0)
        else:
            self.cmd('DCP=0')#in non dipole/quadrupole 0 means DC OFF

    def query_ErrorCode(self):
        return self.ERR_CODE[0]

    def query_MachineState(self):
        return VDQ(self.pstype().module[0].state_id, q=AQ_VALID)

    def query_ec(self, mnemonic, conv_fun):
        '''Executes EC query 'mnemonic' in order to obtain the value of an
           attribute, convert and returning it as VDQ.
        '''
        if self.wavl.busy:
            msg = 'unable to query %s while %s' % (mnemonic, self.wavl.busy)
            self._trace = msg+'\n'+traceback.format_exc()
            raise PS.PS_Exception(msg)

        payload = self.cmd(mnemonic+'/')
        value = conv_fun(payload)
        return VDQ(value, q=AQ_VALID)

    def write_ec_attribute(self, attr, mnemonic, conv_fun):
        '''Executes EC set command 'mnemonic' in order to obtain the value of an
           attribute and set it.
        '''
        aname = attr.get_name()
        value = attr.get_write_value()
        value_str = str(conv_fun(value))
        wr_cmdstr = mnemonic+'='+value_str
        self.cmd(wr_cmdstr,q=True)
        if aname in self.cache:
            self.cache[aname].quality = AQ_CHANGING
        return value

    @PS.AttrExc
    def write_CurrentSetpoint(self, attr):
        #if self._isCycling:
        #    Except.throw_exception("Not available during cycling",
        #                           "Write CurrentSetpoint not possible during cycling",
        #                           "write_CurrentSetpoint")
        # gets write value
        Iset = attr.get_write_value()
        t0 = time()
        vdq = self.cache['CurrentSetpoint'] = VDQ(Iset, t0, AQ_CHANGING)
        self.write_ec_attribute(attr, 'CUR', repr)
        vdq.set_attr(attr)
        if self.get_state() in (Tg.DevState.ON, Tg.DevState.MOVING):
            self.STAT.CURRENT_ADJUST()

    def query_RegulationFrequency(self):
        '''returns VDQ for RegulationFrequency.
           constant during PS.
        '''
        # uses device property configured value if defined,
        # otherwise query PS itself
        if not self.RegulationFrequency:
            # cobj is given as 1% of kHZ
            mrf = self.ObjInt(state.COBJ_MAIN_REGUL_FREQ) / 100 / 1000.0
            self.RegulationFrequency = mrf

        return VDQ(self.RegulationFrequency, q=AQ_VALID)

    def query_WaveDuration(self):
        mrf = self.value('RegulationFrequency')
        wln = self.value('WaveLength')
        if wln <= 0:
            duration = 0.0
        else:
            wst = self.value('WaveInterpolation')
            duration = wln * pow(2,wst) / mrf
        return VDQ(duration, q=AQ_VALID)

    def query_WaveGeneration(self):
        if self.cab.use_waveforms:
            return self.query_ec('WMO', lambda x: bool(int(x)))
        else:
            return VDQ(False, q=AQ_VALID)

    @PS.AttrExc
    def read_RemoteMode(self, attr):
        self.cab.rem_vdq.set_attr(attr)

    def query_Version(self):
        r = self.ObjInt(state.COBJ_VERSION)

        # given in bit 23-0, lowest 3 bytes
        typ = r & cabinet.TYPE_MASK
        # release or not?
        release = 1 if (r>>24) & 1 else 0
        # remaining 7 bits 31-25
        rev = r >> 25
        val = [typ, release, rev ]
        return VDQ(val,q=AQ_VALID)

    def get_nominal_y(self):
        '''Obtains nominal range (ymin and ymax) either from cache, from attribute
           configuration or otherwise by reading from control unit, then storing
           it into attribute configuration.
        '''
        C = self.cache
        if not 'ymin' in C or not 'ymax' in C:
            ydat = self.cmd_seq('OBJ=0x202a', 'YMIN/', 'YMAX/' )
            t = time()
            C['ymin'] = VDQ(float(ydat[1]), t, AQ_VALID)
            C['ymax'] = VDQ(float(ydat[2]), t, AQ_VALID)

        ymin = C['ymin'].value
        ymax = C['ymax'].value
        if ymin not in (0,-ymax):
            msg = 'current range [%s,%s] neither uni- nor bipolar' % (ymin,ymax)
            self._fault(msg)
        return ymin,ymax

    @PS.AttrExc
    def read_WaveLength(self, attr):
        self._check_use_waveforms('attribute '+attr.get_name())
        self.vdq_set(attr)

    @PS.AttrExc
    def read_WaveId(self, attr):
        self._check_use_waveforms('attribute '+attr.get_name())
        self.vdq_set(attr)

    @PS.AttrExc
    def write_TriggerMask(self, attr):
        self._check_use_waveforms('attribute '+attr.get_name())
        value = attr.get_write_value()
        self.write_ec_attribute(attr, 'WTR', str)
        self.cache['TriggerMask'] = VDQ(value, q=AQ_VALID)

    def query_TriggerMask(self):
        if 'TriggerMask' in self.cache:
            return self.cache['TriggerMask']
        else:
            return VDQ(-1)

    @PS.AttrExc
    def read_WaveName(self, attr):
        self.vdq_set(attr)

    @PS.AttrExc
    def write_WaveName(self, attr):
        wave_name = attr.get_write_value()
        self.cache['WaveName'] = VDQ(wave_name, q=AQ_VALID)
        prop = { 'WaveName' : wave_name }
        DATABASE.put_device_property(self.get_name(), prop)

    @PS.AttrExc
    def read_isCycling(self,attr):
        attr.set_value(self.is_cycling())

class BrukerEC_PS_Class(PS.PowerSupply_Class):

    class_property_list = PS.gen_property_list(XI=4)

    class_property_list['RegulationPrecision'] = [ DevDouble,
        'for which current delta the regulation process is considered finished',
        5e-3
    ]

    device_property_list = PS.gen_property_list(cpl=class_property_list)
    device_property_list['Wave'] = [ DevVarStringArray,
        'waveform interpolation, abscissa and form', []
    ]

    device_property_list['WaveName'] = [ DevString,
        'name of waveform last downloaded', ''
    ]

    device_property_list['Port'] = [ DevShort,
        'port used for cabinet control (querying and reading interlocks)',
        0
    ]

    device_property_list.update({
        'EnableRegulationTuning' : [ DevBoolean,
            'indicates whether regulation parameters related attributes are supported or not',
            False
        ],
    })
    device_property_list['RegulationFrequency'] = [ DevDouble,
        'regulation frequency as calibrated',
        0.0
    ]
    device_property_list['DCP_Bit'] = [ DevShort, 'which bit is used to switch PS OFF', 0]

    attr_opt=('Resistance','CyclingTrace','isCycling')
    attr_list = PS.gen_attr_list(opt=attr_opt,max_err=100)
    attr_list['WaveX'] = [[DevDouble, SPECTRUM, READ_WRITE, wavl.MAX_WAVE_LEN] ,{
        'description' : 'abscissa of wave as written by the user'
    }]
    # disabled
    attr_list['Iwave'] = [[DevDouble, SPECTRUM, READ, wavl.MAX_WAVE_LEN] ,{
	  'description' : '''direct read and write access to wave up- and download values.
  Read and writing this attribute can take longer than the usual 3 seconds timeout.
	  '''
      }]
#    attr_list['WaveY'] = [[DevDouble, SPECTRUM, READ_WRITE, wavl.MAX_WAVE_LEN] ,{
#        'description' : 'y-data '
#        }]

    attr_list['Waveform'] = [[DevDouble, SPECTRUM, READ_WRITE, wavl.MAX_WAVE_LEN] , {} ]
    attr_list['Version'] = [[DevShort, SPECTRUM, READ, 3] , {
        'description' : 'software version object: type, revision, release (0=Beta, 1=Release)'
        } ]
    cmd_list = PS.gen_cmd_list(opt=('Command',))

factory.start(BrukerEC_PS,BrukerEC_PS_Class)
# factory.add_mnemonic('On', 'DCP', tp=DevBoolean)
factory.add_commands('On', 'Off', 'CabinetOn', 'CabinetOff', 'Cycle', 'AbortCycle',)
factory.add_cmd('ReadMachineStates',
    outd=[DevString, 'returns internal status machine codes of all submodule']
)
factory.add_cmd('UploadWaveform',
    ind=[DevVoid, 'force re-reading of waveform from control unit']
)
factory.add_attribute('RemoteMode', tp=DevBoolean)
FMT = '%6.4f'
factory.add_ec_attr('CurrentSetpoint', 'CUR', rw=READ_WRITE,
    extra={'format' : FMT},
)

factory.add_ec_attr('Current', 'ADC', rw=READ,
    extra={'format' : FMT}
)
factory.add_ec_attr('CurrentRamp', 'RTC', rw=READ_WRITE,
    extra={'format' : FMT}
)
factory.add_ec_attr('Voltage', 'ADV',
    extra={'format' : FMT}
)

factory.add_ec_attr('TriggerMask', 'WTR', rw=READ_WRITE, tp=DevLong,
    extra = { 'min value' : 0, 'max value' : 0x3fffffff,
    'description' : '''WTR setting, number of triggers to consider:
0:no wave generated, 1:generate only wave, 0x3fffffff:continously'''
})

factory.add_ec_attr('CurrentRMS', 'ARM', tp=DevDouble,
    extra = { 'format' : FMT, 'description' : 'Current RMS value' }
)

# Waveform Handling
factory.add_ec_attr('WaveOffset', 'WOF', rw=READ_WRITE,
    extra={'format' : FMT, 'unit':'A' , 'label':'Offset'}
)

factory.add_ec_attr('WaveLength', 'WLN', tp=DevShort, 
                    extra={'unit':'points', 'label' : 'Length' },
                    query='force'
)
factory.add_ec_attr('WaveGeneration', 'WMO', rw=READ_WRITE, tp=DevBoolean,
     extra = { 'format' : '%1d' }
)



factory.add_ec_attr('WaveInterpolation', 'WST', rw=READ_WRITE, tp=DevShort,
    extra={'min value':0, 'max value' : 5,
           'description' : 'Indicates the number of waveform points to be '+
                           'interpolated per regulation period. Valid values '+
                           'range from 0 to 5 corresponding to the powers of '+
                           '2 between 0 and 32.',
            'label' : 'Interpolation',
            'format' : '%1d',
            'unit':'*'
        },
)
factory.add_attribute('WaveStatus', tp=DevString, extra=dict(label='Status'))
factory.add_attribute('WaveName', rw=READ_WRITE, tp=DevString, extra=dict(Memorized=True, label='Ramp'))

factory.add_cmd('ObjInt',
    [DevShort,'cobj id to read'],
    [DevLong, 'value of can bus object']
)
factory.add_cmd('ObjFloat',
    [DevShort,'cobj id to read'],
    [DevDouble, 'value of can bus object']
)

factory.add_attribute('RegulationFrequency', tp=DevDouble,extra={
    'unit':'kHz',
    'description' : 'regulation frequency as calibrated or as read from power supply',
    'format' : '%7.4f'
})

factory.add_attribute('WaveDuration', tp=DevDouble, extra={
    'unit' : 'ms',
    'description' : 'how much time it takes to pass the wave form',
    'label' : 'Duration'
})

factory.add_attribute('WaveId', tp=DevLong, extra={
    'description' : 'crc32 finger print of wave form'
})

# retrieved directly from cache
factory.add_attribute('MachineState', tp=DevLong, extra={
    'description' : 'code for the internal status machine of the power supply control unit',
    'format' : '%2x',
})

factory.add_ec_attr('ErrorCode', 'STA/', tp=DevLong,extra={
    'description' : 'numerical error'
})

factory.add_attribute('CurrentDelta', tp=DevDouble, extra={
    'unit' : 'A',
    'description' : 'difference between current setpoint and current readback relative to nominal current'
    # , 'max_alarm' : 0.1
})

factory.add_attribute('CurrentNominal', tp=DevDouble, extra={
    'unit' : 'A',
    'description' : 'nominal current'
    # , 'max_alarm' : 0.1
})



class BrukerEC_Cabinet(PS.PowerSupply):
    '''Tango interface to CabinetControl object.
    '''

    # needed to silence pychecker warnings
    Port = None
    IpAddress = None

    PUSHED_ATTR = ('ErrorCode', 'MachineState', 'State', 'Status')

    def __init__(self, cl, name):
        PS.PowerSupply.__init__(self, cl, name)
        self.init_device(cl, name)

    def init_device(self, cl=None, name=None):
        PS.PowerSupply.init_device(self, cl, name)

        if not self.IpAddress:
            msg = 'device property IpAddress must be configured'
            raise Exception(msg)

        self.cab = cabinet.instance()
        self.cab.log = self.log
        self.cab.connect(self.IpAddress)
        self.restart_bsw_tcp_last_attempt_t = 0
        self.cab.restart_bsw_tcp = self.restart_bsw_tcp
        try:
            self.cab.reconnect()
        except socket.error, err:
            self.log.warn(err)
        if not self.cab.is_connected:
            self.log.error('not being connected %s',self.cab.comm.hopo)

        self.wavl = wavl.instance()
        self.wavl.log = self.log
        self.wavl.connect(self.IpAddress)
        self.wavl.reconnect_reap = self.wavl_reconnect_reap

        # for the cabinet instead of cache["XY"] _XY is used.
        self._ErrorCode = PseudoAttr('ErrorCode', 0)
        self._RemoteMode = PseudoAttr('RemoteMode', False)
        self._MachineState = PseudoAttr('MachineState', 0)

        self.cab.updater.add(self.UpdateState)
        xi_msg = [ getattr(self,'Interlock%d' % d) for d in range(1,5) ]
        self.cab.customize_xi(xi_msg)
        self.cab.start()

    @PS.ExceptionHandler
    def delete_device(self):
        self.STAT.DELETE()
        self.cab.stop()
        self.cab.disconnect_exc()
        self.wavl.disconnect_exc()

    #### Attributes ####
    def query_RemoteMode(self):
        if self.Port == 'stb':
            REM = self.cab.command_seq(0, 'REM/')[0]
        else:
            REM = self.cab.command_seq(self.Port, 'REM/')[0]
        return VDQ(bool(int(REM)), time(), AQ_VALID)

    @PS.AttrExc
    def read_RemoteMode(self, attr):
      self.cab.rem_vdq.set_attr(attr)

    @PS.AttrExc
    def read_MachineState(self, attr):
        state_id = self.cab.state_id
        if state_id is None:
            attr.set_quality(AQ_INVALID)
        else:
            attr.set_value(state_id)

    @PS.AttrExc
    def read_ErrorCode(self, attr):
        self._ErrorCode.set_attr(attr)

    #### Commands ####
    @PS.CommandExc
    def ResetInterlocks(self):
        PS.PowerSupply.ResetInterlocks(self)
        self.wavl.reconnect_reap()
        self.cab.reconnect(reap=True)
        self.cab.command_seq(self.Port, 'RST=0')
        self.cab.update()

    def restart_bsw_tcp(self):
        '''restarts bsw tcp repeater.
        '''
        # assures that between two restart attempts elapses more than 10 seconds
        if self.restart_bsw_tcp_last_attempt_t+10.0 >= time():
            return
        else:
            self.restart_bsw_tcp_last_attempt_t = time()
        self.STAT.set_stat2(Tg.DevState.UNKNOWN, 'restarting bsw server...')
        try:
            self.cab.telnet((
                "kill `ps | grep bsw_tcp_repeater2  | cut -b-6`",
                "start_repeater.sh &"
              ))
            return True
        except Exception:
            self.log.error('while trying to restart bsw server', exc_info=1)


    def change(self, pseudo_attr, *args, **kwargs):
        pseudo_attr.update(*args, **kwargs)
        if not pseudo_attr.value is None:
            self.push_change_event(pseudo_attr.name, *pseudo_attr.triple)

    @PS.CommandExc
    def UpdateState(self):
        # for all practical concerns UpdateState should behave like any command
        # in order to avoid
        cab = self.cab
        try:
            if not cab.reconnect():
                if cab.comm._exc_msg:
                    self.STAT.COMM_ERROR(self.cab.comm._exc_msg)
                else:
                    self.STAT.COMM_ERROR('no connection to control unit %s' % cab.host)
                return
            cab.process_command_queue()
            cab.update()
            self.change(self._ErrorCode, cab.error_code, cab.update_t, q=AQ_VALID)
            self.change(self._MachineState, cab.state_id, cab.update_t, q=AQ_VALID)
            self.STAT.set_stat2(*cab.stat)

        except socket.timeout:
            self.STAT.OFFLINE('control unit', str(cab.comm.host)+ ' timeout')

        except socket.error, err:
            msg = '%s (%s)' % (err.strerror, err.errno)
            if not err.filename is None:
                msg = str(err.filename)+': '+msg
            self.STAT.COMM_ERROR(msg)

        except cabinet.CanBusTimeout:
            # self.STAT.COMM_FAULT('CAN bus hanging')
            # Not necessary to decay to fault, only alert that it have happen.
            self._alarm(str(msg))

        if cab.use_waveforms and cab.is_connected:
            try:
                self.wavl.reconnect()
            except socket.error, err:
                self.STAT.COMM_FAULT(str(err))
                return

    @PS.CommandExc
    def Command(self, command_list):
        '''Executes a list of EC commands, returing a list of responses for
           each command.
        '''
        if self.Port=='stb':
            port = 0
        else:
            port = int(self.Port)

        cmd_prt = 'PRT=%d' % port #< command for going back to current port
        cl = command_list[:]
        cl.append(cmd_prt)
        rls = self.cab.command_seq(port, *cl)[:-1]
        # executes a PRT that resets the port to the original one
        self.cab.command_seq(port)
        return rls

    cmd_seq = BrukerEC_PS.__dict__['cmd_seq']

    @PS.CommandExc
    def On(self):
        self.cab.power_on()

    @PS.CommandExc
    def Off(self):
        self.cab.power_off()

    @PS.CommandExc
    def Tel(self, commands):
        return self.cab.telnet(commands)

    @PS.CommandExc
    def RebootControlUnit(self):
        return self.cab.telnet( ('reboot',) )

    @PS.CommandExc
    def Sync(self):
        self.cmd_seq('SYNC')

    def wavl_reconnect_reap(self):
        '''Tries to reconnect to waveform loader, if this fails
           because of connection refused it will try to restart the
           bsw_tcp_repeater.
        '''
        if not self.cab.use_waveforms:
            return
        try:
            self.wavl.reconnect()
        except socket.error, err:
            if err.errno==ECONNREFUSED:
                self.restart_bsw_tcp()
            else:
                self.log.exception('wave reconnect reap')
            return False
        except Exception:
            self.log.exception('wave reconnect reap')


class BrukerEC_Cabinet_Class(PS.PowerSupply_Class):

    class_property_list = PS.gen_property_list(XI=4)

    device_property_list = PS.gen_property_list( ('IpAddress',),cpl=class_property_list)
    device_property_list.update({
        'Port' : [ DevString,
            'port used for cabinet controller, or empty to indicate that STB/ on port 0 should be used',
            'stb'
        ]
    })

    device_property_list['ConfigureEvents'] = [ DevShort,
'Wether events should be configured when DS is started. After configuration the property will be set to 0, so that it once.'
'Note that setting this property and restarting the DS will overwrite existing event configuration of ALL devices of this server instance.',
        0 ]
    cmd_list = PS.gen_cmd_list(opt=('UpdateState','Command'))
    cmd_list['Tel'] = [ [ Tg.DevVarStringArray, 'executes telnet commands'],
        [ Tg.DevString, 'response to telnet commands'],
        { 'display level' : Tg.DispLevel.EXPERT },
    ]
    cmd_list['RebootControlUnit'] = [ [ Tg.DevVoid, 'reboots control unit'],
        [ Tg.DevVoid, ''],
        { 'display level' : Tg.DispLevel.EXPERT },
    ]
    attr_list = PS.gen_attr_list(max_err=64)
    attr_list['ErrorCode'] = [[Tg.DevLong, Tg.SCALAR, Tg.READ],
                              {'display level':Tg.DispLevel.EXPERT}]
    attr_list['MachineState'] = [[Tg.DevLong,Tg.SCALAR,Tg.READ],
                                 {'display level':Tg.DispLevel.EXPERT}]


if __name__ == '__main__':
    classes = (BrukerEC_PS, BrukerEC_Cabinet)
    PS.tango_main( 'BrukerEC_PS', classes, prerun=cfg_events)

