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

__version__ = "$Revision: 68852 $"
# $Source$


class Release:
    author = "Lothar Krause <lkrause@cells.es> for CELLS / ALBA synchrotron"
    date = "2010-03-29"
    release = "sourceforge 1.8"
    hexversion = 0x010802

Release.__str__ = lambda self: "%06x" % self.hexversion

# python standard imports
import sys
import errno
import socket
import traceback
from types import StringType
from pprint import pformat,pprint
from copy import deepcopy
from time import time, sleep
import logging
from zlib import crc32

from PyTango import DevVoid, DevVarStringArray, DevString, DevDouble, \
  DevBoolean, DevShort, DevLong, DevUShort, DevULong, DevState, \
  SPECTRUM, SCALAR, READ_WRITE, READ

import PyTango as Tg
import ps_standard as PS
import ps_util as PU

from ps_standard import AQ_VALID, AQ_INVALID, AQ_CHANGING, AQ_ALARM
from ps_util import NAN

# relative imports
import wavl
import cabinet
import state
import factory
import tuning

class WaveDisabled(PS.PS_Exception):
    '''Raised when a wave form related feature that is requested
       and power supply does not support wave forms
    '''
    pass

class AttributeInvalid(PS.PS_Exception):
    '''Raised when the value an invalid attribute is needed.
    '''
    attribute_name = None
    def __init__(self, aname):
        PS.PS_Exception.__init__(self, repr(aname) + ' is invalid')
        self.attribute_name = aname

DB = PS.DATABASE

# minimum amount of time that a device will be in MOVING state before
# going to ON state.
MIN_MOVING_TIME = 0.1
VDQ = PS.VDQ

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
    DB.put_device_attribute_property(dev, props)

def cfg_events(serv):
    '''Configures the events of the devices of 'serv'.
    '''
    # unless surpressed by setting ConfigureEvents property of the cabinet
    # device to 0 change events will be configured
    want_cfg = 1

    # firsts the name of the cabinet device is searched
    cab_name = None
    dcl = DB.get_device_class_list(serv)
    dcl = zip(dcl[::2], dcl[1::2])
    for cab_name, cl in dcl:
        if cl=='BrukerEC_Cabinet':
            break

    if cab_name is None:
        print('warning: no cabinet device defined')
        return

    # event configuration wanted?
    props = DB.get_device_property(cab_name, ('ConfigureEvents', ) )
    p = props.get('ConfigureEvents', ['1'])
    want_cfg = bool(int(p[0])) if p else True

    # in any case the event configuration routine is only executed once,
    # when the server is started for the first time
    if not want_cfg: return
    DB.put_device_property(cab_name, {'ConfigureEvents':'0'})
    print('configuring change events...')
    for d,c in dcl:
        if c=='BrukerEC_PS':
            cfg_ev_ps(d)


def customize_interlock_msg(ls, dev_name, index_xi):
    '''Updates the interlock messages in 'ls' from the Interlock class and
       device properties for 'dev_name'.
    '''

    # list of interlock keys
    key_list = [ 'Interlock'+str(i+1) for i in range(len(index_xi)) ]
    # interlock properties 'props' are initialized from class properties
    props = DB.get_class_property('BrukerEC_PS', key_list)

    props_device = DB.get_device_property(dev_name, key_list)
    for p in props_device.keys():
        if not props_device[p]:
            del props_device[p]

    # interlock properties are updated from the device properties
    props.update(props_device)

    # finally the interlock messages in 'ls' are updated at the indices given
    # in index_xi
    for i,index in enumerate(index_xi):
        key = key_list[i]
        p = props[key]
        if len(p)>0:
            ls[index] = p[0]

class BrukerEC_PS(PS.PowerSupply):
    '''Representing individual power supplies.
    '''

    # needed for pychecker warnings regarding device properties
    Port = None
    tuner = None
    wave_load = None
    __errors = []

    PUSHED_ATTR = ('Current', 'CurrentSetpoint', 'Voltage',
      'MachineState', 'ErrorCode', 'State', 'ShortStatus', 'Status',
      'WaveStatus')

    def __init__(self, cl, name):
        PS.PowerSupply.__init__(self, cl, name)
        self.init_device(cl, name)

    def init_device(self, cl=None, name=None):
        PS.PowerSupply.init_device(self, cl, name)

        def intify(ls):
            """Converts strings configured in CabMcOx properties into integers.
            """
            return tuple(int(x) for x in ls)
        self.CabMcOn = intify(self.CabMcOn)
        self.CabMcOff = intify(self.CabMcOff)

        # sets up cache of VDQ objects for safely manipulating and accessing
        # read values from within the device itself
        self.cache = dict(
            WaveGeneration = VDQ(False),
            WaveLength = VDQ(0),
            WaveDuration = VDQ(0.0),
            WaveStatus = VDQ(wavl.READY, q=AQ_VALID),
            WaveId = VDQ(0, q=AQ_INVALID),
            WaveName = VDQ(self.WaveName, q=AQ_VALID)           
        )
        # when moving is expected to be finished
        self.move_fin = 0

        self.cab = cabinet.instance()
        self.wavl = wavl.instance()

        self._pstype = None
        self.update_cycle = 0
        self.cab.updater.add(self.UpdateState)
        self.STAT.INITIALIZED()
        self.STAT.set_stat2(Tg.DevState.UNKNOWN, 'not yet connected')

    use_waveforms = property(lambda self: self.cab.use_waveforms)

    # Basic Utilities
    def query(self, aname):
        '''Executes query to hardware for attribute 'aname' by calling the
           returning the result from calling the query_<aname> method
        '''
        qfun_name = 'query_'+aname
        try:
            return getattr(self, qfun_name)()
        except AttributeError:
            if not hasattr(self, qfun_name):
                raise
            else:
                msg = 'no method %r for querying %r'  % (qfun_name, repr(aname))
                raise Exception(msg)

    def vdq_set(self, attr, **kwargs):
        '''Shortcut for self.vdq(attr.get_name()).set_attr(attr)
        '''
        vdq = self.vdq(attr.get_name(), **kwargs)
        vdq.set_attr(attr)
        return vdq

    def push_vdq(self, aname, *args, **kwargs):
        self.cache[aname] = vdq = VDQ(*args, **kwargs)
        self.push_change_event(aname, *vdq.triple)

    def vdq(self, aname, query='auto', dflt=None, dflt_q=AQ_INVALID):
        '''Returns a VDQ object for aname, or throws an exception.
            @param aname attribute name for which vdq object is needed
            @param query whether the value should be queried from hardware can be
                        'auto', which will execute the query when necessary, or
                        'force' will always queried, or
                        'never' means that an exception will be generated if
                        unavailable.
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

    def value(self, aname, dflt=None, query='auto'):
        '''Returns the value of aname, or throws an Exception.
           @param aname name of the attribute desired
           @param query will be passed to @see vdq
           @param dflt passed on to @see vdq
        '''
        vdq = self.vdq(aname, query=query, dflt=dflt)
        if vdq.quality == AQ_INVALID:
            if dflt is None:
                traceback.print_stack()
                raise AttributeInvalid(aname)
            else:
                return dflt

        return vdq.value

    def update_attr(self, aname):
        '''Updates attribute 'aname' by first querying its vdq,
           than saving it to the corresponding Attribute object and
           generating change events.
           Note that attributes that are updated in this manner must not be polled.
           in order to avoid concurrency problems of TANGO polling thread and
           client and other threads.
            @param aname which attribute to query and update
        '''

        vdq = self.cache[aname] = self.query(aname)
        if vdq.value is None:
            msg = 'attempt to push change event for attribute %r w/o value' % aname
            self._alarm(msg)
        else:
            self.push_change_event(aname, *vdq.triple)
        return vdq.value

    def query_RemoteMode(self):
        return self.cab.rem_vdq

    @PS.AttrExc
    def read_Current(self, attr):
        self.vdq('Current').set_attr(attr)

    @PS.AttrExc
    def read_CurrentSetpoint(self, attr):
        self.vdq_set(attr)
        attr.set_write_value(self.value('CurrentSetpoint'))

    def query_CurrentDelta(self):
        I = self.vdq('Current')
        Iset = self.vdq('CurrentSetpoint')
        delta = Iset.value - I.value
        t = max(I.date, Iset.date)
        q = PS.combine_aq(I.quality, Iset.quality)
        return VDQ(delta,t,q)

    def pstype(self):
        """Returns PSType object for power supply.
        """
        # By executing the method it is also guranteed that certain
        # tasks (detecing CurrentNominal) have succeeded
        if self._pstype is None:
            type_code = self.query('Version').value[0]

            if type_code==state.PSTYPE_CODE_QUAD:
                brk_config2 = self.ObjInt(state.COBJ_BRK_CONFIG2)
                if brk_config2==state.BRKconfig2_MASTER:
                    pst = deepcopy(state.PSTYPE_BIG_QUAD)
                elif brk_config2 in state.BRKconfig2_STANDALONES:
                    pst = deepcopy(state.PSTYPE_SMALL_QUAD)
                else:
                    msg = 'bad Port %s configured, can not be slave module' % self.Port
                    raise PS.PS_Exception(msg)
            else:
                pst = deepcopy(state.REG2PSTYPE[type_code])
            if pst is None:
                msg = 'failed to detect suitable PS, port %d has type code %d' \
                    % (self.Port, type_code)
                raise PS.PS_Exception(msg)

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

            if pst.XI:
                err = pst.ports[0].errors
                name = self.get_name()
                customize_interlock_msg(err, name, pst.XI)
            self.get_nominal_y()
            # using self.use_waveform is valid here because the
            # cabinet type is detected when that cabinet device connects
            # the socket and the Version query above ensure that this has
            # happened
            if self.use_waveforms:
                self.tuner = tuning.Tuner(self, pst)

            # after this line the pstype is considered fully detected
            self._pstype = pst
            self.cab.init_counter += 1

            try:
                # sets sane write value for WaveOffset
                wof = float(self.cmd_seq('WOF/')[0])
                self._attr('WaveOffset').set_write_value(wof)

                # set sane write value for CurrentSetpoint
                cur = float(self.cmd_seq('CUR/')[0])
                self._attr('CurrentSetpoint').set_write_value(cur)

                self.UploadWaveform()

            except Exception, exc:
                self.log.exception(str(exc))

        return self._pstype

    @PS.ExceptionHandler
    def delete_device(self):
        PS.PowerSupply.delete_device(self)
        self.cab.updater.remove(self.UpdateState)

    ## Tango Commands ##
    @PS.ExceptionHandler
    def Command(self, command_list):
        '''Executes a list of EC commands, returing a list of responses for
           each command.
        '''
        return self.cmd_seq(*command_list)

    def cmd(self, *commands):
        return '\n'.join(c for c in self.cmd_seq(*commands) if c)

    def cmd_seq(self, *commands):
        '''Executes a list of EC commands, returing a list of responses for
           each command.
        '''
        try:
            return self.cab.command_seq(self.Port, *commands)

        except socket.timeout:
            msg = 'control unit %r not responding' % self.cab.host
            raise PS.PS_Exception(msg)

        except socket.error, exc:
            e = exc.args[0]
            if e==errno.EHOSTUNREACH:
                msg = 'control unit %r offline' % self.cab.host
            else:
                msg = 'control unit %r: %s (%d)' % (self.cab.host, exc.args[1], e)
            raise PS.PS_Exception(msg)

    def load_waveform(self):
        '''Executes up - and downloads of waveforms.
        '''
        if not self.cab.use_waveforms: return
        load = self.wave_load
        if load is None: return
        try:
            # invalidates previous Waveform read value
            self.push_changing('Waveform' , [])
            self.push_vdq('WaveStatus', load.BASE_MSG+'ing...', q=AQ_CHANGING)
            start_t = time()
            load.run(self, self.wavl)
            diff_t = time()-start_t
            self.wave_load = None
            msg = load.BASE_MSG+' finished %.2fs' % diff_t
            self.push_vdq('WaveStatus', msg, q=AQ_VALID)            

        except PS.PS_Exception, exc:
            msg = load.BASE_MSG+' error: '+str(exc)
            self.cache['Waveform'] = VDQ([], q=AQ_INVALID)
            self.push_vdq('WaveStatus', msg, q=AQ_ALARM)
            traceback.print_exc()

        except socket.error, err:       
            err_msg = str(err)
            self._alarm('waveforms %s' % err_msg)
            ws = load.BASE_MSG+' failed: '+err_msg
            self.push_vdq('WaveStatus', ws, q=AQ_ALARM)

        # faults during waveform upload / download will not prevent
        # general status update of GUI, even though maybe it should.
        except Exception, exc:
            msg = str(exc)
            self._fault(msg, exc_info=1)
            msg = load.BASE_MSG+' failed: '+msg
            self.cache['Waveform'] = VDQ([], q=AQ_INVALID)
            self.push_vdq('WaveStatus', msg, q=AQ_ALARM)

    @PS.CommandExc
    def UpdateState(self):
        '''Updates the state of device.
        '''
        if not self.cab.is_connected:
            if self.cab.comm._exc_msg:
                self.STAT.ALARM(self.cab.comm._exc_msg)
            else:
                self.STAT.ALARM('no connection to control unit') 
            return

        try:
            self._up_start_t = time()
            self.pstype()
            if not self.cab.all_initialized():
                return


            # resets alarms
            self.alarms.clear()
            STEP = 6
            # local state, interlock
            def up(aname, phase=0, dflt=None):
                if self.update_cycle % STEP == phase:
                  return self.update_attr(aname)
                else:
                  return self.value(aname, dflt=dflt)

            # checks CAN bus communication
            STA = up('ErrorCode',0)
            if STA & 0x00800000:
                raise cabinet.CanBusTimeout('CAN bus communication timing out')

            # detects power supply specific settings
            t = self.pstype()

            # performs wave up/down-loads first, if any
            self.load_waveform()

            # continues with regular status update
            REM = up('RemoteMode',1, dflt=False)
            STC = up('MachineState',2)
            WMO = up('WaveGeneration',3, dflt=False)
            Iref = up('CurrentSetpoint',4)
            Imeas = self.update_attr('Current')

            # updates Voltage only in DC mode
            ps_on = self.is_power_on()
            if WMO:
                ramp_mode = 'ramp mode'
                moving = False
            else:
                U = up('Voltage',5, dflt=0.0)
                ramp_mode = ''
                Inominal = self.value('CurrentNominal')
                rel_err = PU.relative_error(Iref, Imeas, Inominal)
                moving = ps_on and (rel_err > self.RegulationPrecision)

            alarms = PU.bit_filter_msg(STA, t.ports[0].errors)
            self.alarms += alarms
            self.alarms += self.cab.get_alarms(t.mask_cab)

            CAB_STAT = self.cab.stat
            CAB_MC = self.cab.state_id

            interlocks = self.value('Errors', query='force')

            # decides which state should be used
            # faults are sticky and must be fixed by ResetInterlocks
            if self.get_state()==DevState.FAULT:
                pass

            elif not self.wavl.is_connected:
                msg = self.wavl.sok._exc_msg
                if not msg:
                    msg = 'waveform loading not connected'
                self.STAT.ALARM(msg)

            elif STC in state.FAULT_STATES:
                msg = 'state %2x' % self.value('MachineState')
                self.STAT.FAULT(msg)

            elif STC in state.INTERLOCK_STATES:
                self.STAT.INTERLOCKS(interlocks)

            # any error will result in alarm state
            elif interlocks:
                self.STAT.ALARMS(interlocks)

            elif ps_on:
                self.STAT.ON(ramp_mode)

            # check for moving state
            elif moving or self.move_fin > time():
                self.STAT.CURRENT_ADJUST()

            elif CAB_MC in self.CabMcOn:
                step = CAB_MC-self.CabMcOn[0]
                self.STAT.SWITCHING_ON("%d" % step, ramp_mode)

            elif CAB_MC in self.CabMcOff:
                step = CAB_MC-self.CabMcOff[0]
                self.STAT.SWITCHING_OFF("%d" % step, ramp_mode)

            elif CAB_STAT:
                c = CAB_STAT[0:2]
                if ramp_mode:
                    c.append(c[1]+', '+ramp_mode)
                self.STAT.set_stat2( *c  )

            else:
                self.STAT.OFF(extra=ramp_mode)

        except PS.PS_Exception, exc:
            self._alarm(str(exc))

        finally:
            self.__errors = self.faults+self.alarms
            self.update_cycle += 1
            self._up_end_t = time()
            self._up_diff_t = self._up_end_t-self._up_start_t

    @PS.CommandExc
    def Off(self):
        self.cab.switch_power(self.Port, self.DCP_Bit)

    @PS.CommandExc
    def On(self):
        # use 'broadcast' mode
        self.cmd('WTS=0')
        self.cab.switch_power(self.Port, self.DCP_Bit+1)
        self.cache['TriggerMask'] = VDQ(0,q=AQ_VALID)

    @PS.CommandExc
    def ResetInterlocks(self):
        PS.PowerSupply.ResetInterlocks(self)
        self.cmd('RST=0')
        self.cache['WaveStatus'].value = wavl.READY

    @PS.CommandExc
    def UploadWaveform(self, maxlen=None):
        self._check_use_waveforms()
        self.log.debug('UploadWaveform!')
        if 'Waveform' in self.cache:
            del self.cache['Waveform']
        n = self.update_attr('WaveLength')
        ul = self.wave_load = wavl.Upload(self.Port, n)
        self.push_changing('WaveLength')
        self.push_changing('WaveDuration')
        self.push_vdq('WaveStatus', ul.BASE_MSG+' pending', q=AQ_CHANGING)

    @PS.CommandExc
    def ObjInt(self, cobj):
        rs = self.cmd_seq('OBJ=%d' % cobj, 'VAL/','SCALED/')
        if rs[0]=='*':
            raise PS.PS_Exception('time out or undefined can bus object %x' % cobj)
        else:
            if rs[1]=='*':
                raise PS.PS_Exception('cobj %x not responsive' % cobj)
            else:
                return int(rs[1],16)

    def obj_vdq(self, cobj, idx=0, conv=float):
        port = self.Port+idx
        self.log.debug('obj_vdq %4x port %d, %d', cobj, self.Port, port)
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


    #### Attributes ####
    def is_dc_mode(self):
        return not self.value('WaveGeneration', dflt=False)

    def is_power_on(self):
        return self.value('MachineState') in self.pstype().states_on

    def is_power_off(self):
        return self.value('MachineState') in self.pstype().states_off

    def is_ramping(self):
        return self.value('WaveGeneration') and self.is_power_on() and self.value('TriggerMask')!=0

    def is_WaveGeneration_allowed(self, write):
        return not write or self.is_power_off()

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

    @PS.AttrExc 
    def write_WaveGeneration(self, attr):
        self.log.debug('writing wave generation')
        self._check_use_waveforms('WaveGeneration')
        val = attr.get_write_value()
        self.cmd('WMO=%s' % int(val))
        self.update_attr('WaveGeneration')            
        self.update_attr('Voltage')            

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
        ix = self.get_regulation_x(n=len(iy))

        t0 = time()
        self.cache['WaveX'] = VDQ(ix, t0, AQ_VALID)
        self.cache['Waveform'] = VDQ(iy, t0, AQ_VALID)
        self.update_attr('WaveLength')
        self.update_attr('WaveDuration')
        wavehash = wavl.calc_hash(ymax, iy)
        self.push_vdq('WaveId', wavehash, d=t0, q=AQ_VALID)
        return

        # interpolation features are simply ignored
        ax = self.cache['WaveAbscissa'][0]
        self.cache['Waveform'] = numpy.interp(ax, ix, iy)

    def push_wave_down(self, waveform):
        '''Called after waveform has been downloaded.
        '''
        t0 = time()
        self.cache['Waveform'] = VDQ(waveform, t0, AQ_VALID)
        self.update_attr('WaveLength')
        self.update_attr('WaveDuration')
        Inominal = self.get_nominal_y()[1]
        wavehash = wavl.calc_hash(Inominal, waveform)
        self.push_vdq('WaveId', wavehash, d=t0, q=AQ_VALID)

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

    def ramp_off(self):
        if self.pstype().has_trigger_mask:
            if 'TriggerMask' in self.cache:
                vdq = self.vdq('TriggerMask')
                if vdq.quality==AQ_INVALID:
                    return DevState.UNKNOWN
                elif vdq.value in (0,1):
                    return 0
                else:
                    return vdq.value
            else:
                return DevState.UNKNOWN
        else:
            self.cmd('DCP=0')
            return DevState.UNKNOWN


    def ramp_restore(self, old_state):
        if old_state is DevState.UNKNOWN:
            return
        else:
            attr = PS.PseudoAttr('TriggerMask')
            attr.write_value = old_state
            self.write_TriggerMask(attr)

    def check_waveform_input(self, x):
        # does nothing currently
        return

    def push_changing(self, aname, new_value=None):
        '''Pushes a new change event for aname with quality == CHANGING
        '''
        if aname in self.cache:
            vdq = self.vdq(aname)
        elif new_value is None:
            try:
                vdq = self.vdq(aname)
            except PS.PS_Exception, exc:
                self.log.warn(exc, exc_info=1)
                return
        else:
            vdq = VDQ(new_value)

        if not new_value is None:
            vdq.value = new_value
        vdq.quality = AQ_CHANGING
        self.push_change_event(aname, *vdq.triple)
        return vdq

    @PS.AttrExc
    def read_Errors(self, attr):
        attr.set_value(self.__errors)

    def query_Errors(self):
        return VDQ(self.__errors, q=AQ_VALID)

    @PS.AttrExc
    def write_Waveform(self, wattr):
        self._check_use_waveforms('attribute Waveform')
        if not self.value('RemoteMode'):
            raise PS.PS_Exception('no waveform download when in local mode')

        old_ramp_state = self.ramp_off()
        try:
            waveform = wattr.get_write_value()
            self.check_waveform_input(waveform)
            YBOTTOM, I_nominal = self.get_nominal_y()
            raw_data = wavl.to_raw(I_nominal, waveform)
            dl = self.wave_load = wavl.Download(self.Port, waveform, raw_data)
            ix = self.get_regulation_x(n=len(waveform))

            self.cache['WaveX'] = VDQ(ix, q=AQ_CHANGING)
            self.cache['WaveY'] = self.push_changing('Waveform', waveform)
            self.push_changing('WaveLength')
            self.push_changing('WaveDuration')
            self.push_changing('WaveStatus', dl.BASE_MSG+' pending')
            self.push_changing('WaveId')

            self.push_vdq('WaveName', v=self.value('WaveName',''), q=AQ_ALARM)

        finally:
            self.ramp_restore(old_ramp_state)

    @PS.CommandExc
    def CabinetOn(self):
        self.cab.power_on()

    @PS.CommandExc
    def CabinetOff(self):
        self.cab.power_off()

    def query_ErrorCode(self):
        def conv(x):
            return cabinet.check(self.Port, x)
        vdq = self.query_ec('STA', conv)
        return vdq

    def query_MachineState(self):
        def conv(x):
            return cabinet.check(self.Port, x)
        return self.query_ec('STC', conv)

    def query_ec(self, mnemonic, conv_fun):
        '''Executes EC query 'mnemonic' in order to obtain the value of an
           attribute, convert and returning it as VDQ.
        '''
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
        self.cmd(wr_cmdstr)
        if aname in self.cache:
            del self.cache[aname]
        return value

    @PS.AttrExc
    def write_CurrentSetpoint(self, attr):
        # gets write value
        Iset = attr.get_write_value()
        slope = self.value('CurrentRamp')
        Iread = self.value('Current', query='force')
        t0 = time()
        vdq = self.cache['CurrentSetpoint'] = VDQ(Iset, t0, AQ_CHANGING)
        dt = max( abs(Iread-Iset) / slope, MIN_MOVING_TIME)
        self.write_ec_attribute(attr, 'CUR', repr)
        vdq.set_attr(attr)
        if self.get_state() in (Tg.DevState.ON, Tg.DevState.MOVING):
            self.move_fin = t0 + dt
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
        if self.use_waveforms:
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
	DB.put_device_property(self.get_name(), { 'WaveName' : wave_name })

    def write_attributes(self, *args, **kwargs):
        self._trace = str(args)

class BrukerEC_PS_Class(PS.PowerSupply_Class):

    class_property_list = PS.gen_property_list()

    class_property_list['RegulationPrecision'] = [ DevDouble,
        'for which current delta we consider the regulation process to be finished',
        5e-4 # 5 ppm
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
        'CabMcOn' : [ DevVarStringArray,
            'state machine codes of relay board when switching power converter ON',
            []
        ],
        'CabMcOff' : [ DevVarStringArray,
            'state machine codes of relay board when switching power converter OFF',
            []
        ]
    })
    device_property_list['RegulationFrequency'] = [ DevDouble,
        'regulation frequency as calibrated',
        0.0
    ]
    device_property_list['DCP_Bit'] = [ DevShort, 'which bit is used to switch PS OFF', 0]


    attr_list = PS.gen_attr_list(max_err=100, opt=('Resistance',))
    attr_list['WaveX'] = [[DevDouble, SPECTRUM, READ_WRITE, wavl.MAX_WAVE_LEN] ,{
        'description' : 'abscissa of wave as written by the user'
    }]
    # disabled
    if 0:
      attr_list['Iwave'] = [[DevDouble, SPECTRUM, READ_WRITE, wavl.MAX_WAVE_LEN] ,{
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
    cmd_list = PS.gen_cmd_list(opt=('UpdateState','Command'))

factory.start(BrukerEC_PS,BrukerEC_PS_Class)
# factory.add_mnemonic('On', 'DCP', tp=DevBoolean)
factory.add_cmd('On')
factory.add_cmd('Off')
factory.add_cmd('CabinetOn')
factory.add_cmd('CabinetOff')
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
    extra={'format' : FMT, 'unit':'A' , 'label':'Offset'}, query='force'
)

factory.add_ec_attr('WaveLength', 'WLN', tp=DevShort, extra={'unit':'points', 'label' : 'Length' },
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
factory.add_ec_attr('MachineState', tp=DevLong, extra={
    'description' : 'code for the internal status machine of the power supply control unit',
    'format' : '%2d',
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

    PUSHED_ATTR = ('ErrorCode', 'State', 'Status')

    def __init__(self, cl, name):
        PS.PowerSupply.__init__(self, cl, name)
        self.init_device(cl, name)

    def init_device(self, cl=None, name=None):
        PS.PowerSupply.init_device(self, cl, name)

        if not self.IpAddress:
            msg = 'device property IpAddress must be configured'
            raise Exception(msg)

        self.cab = cabinet.instance()
        self.cab.connect(self.IpAddress)
        if not self.cab.is_connected:
            self.log.error('not being connected %s',self.cab.comm.hopo)

        self.wavl = wavl.instance()
        self.wavl.log = logging.getLogger(self.log.name+'.wavl')
        self.wavl.connect(self.IpAddress)

        # for the cabinet instead of cache["XY"] _XY is used.
        self._ErrorCode = VDQ(0, q=AQ_INVALID)
        self._RemoteMode = VDQ(False, time(), AQ_INVALID)

        self.cab.updater.add(self.UpdateState)
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
        self.cab.command_seq(self.Port, 'RST=0') 

    def restart_bsw_tcp(self, *args):
        self.STAT.set_stat2(Tg.DevState.UNKNOWN, 'restarting bsw server...')
        try:
          self.cab.telnet((
              "kill `ps ax | grep bsw_tcp  | cut -b-6`",
              "start_repeater.sh &"
              ))
        except Exception:
              self.log.error('while trying to restart bsw server', exc_info=1)


    @PS.CommandExc
    def UpdateState(self):
        try:
            self.cab.reconnect()
            self.alarms.clear()
            code = st = self.cab.update_state()
            self._ErrorCode = VDQ(code, q=AQ_VALID)
            self.alarms += self.cab.get_alarms()
            if self.alarms:
                self.STAT.ALARM(self.alarms[0])
            else:
                self.STAT.ON('cabinet %s okay' % self.IpAddress)
            self.wavl.reconnect() 

        except socket.timeout:
            self.STAT.ALARM('communication timeout')
            raise

        except socket.error, err:
            msg = '%s (%s)' % (err.strerror, err.errno)
            if not err.filename is None:
                msg = str(err.filename)+': '+msg
            self._alarm(msg)
            if err.errno==111:
                self.restart_bsw_tcp()
            elif err.errno==-2:
                pass
            else:
                raise

        except cabinet.CanBusTimeout:
            self.STAT.FAULT('CAN bus hanging')

        if self.cab.use_waveforms:
            try:
                self.wavl.reconnect()                  
            except socket.error, err:
                self.STAT.FAULT(Str(err))
                return

    @PS.CommandExc
    def Command(self, command_list):
        '''Executes a list of EC commands, returing a list of responses for
           each command.
        '''
        if self.Port=='stb':
            port = 0
        else:
            port = self.Port

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
    def Uptime(self):
        tel = telnetlib.Telnet(self.IpAddress)
        tel.write('uptime\n')
        up = tel.read_line()
        return up

    @PS.CommandExc
    def Tel(self, commands):
        return self.cab.telnet(commands)

    @PS.CommandExc
    def RebootControlUnit(self):
        return self.cab.telnet( ('reboot',) )

    @PS.CommandExc
    def Sync(self):
        self.cmd_seq('SYNC')


class BrukerEC_Cabinet_Class(PS.PowerSupply_Class):

    class_property_list = {}

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
    attr_list['ErrorCode'] = [ [ Tg.DevULong, Tg.SCALAR, Tg.READ ], { 'display level' : Tg.DispLevel.EXPERT } ]
    attr_list['MachineState'] = [ [ Tg.DevULong, Tg.SCALAR, Tg.READ ], { 'display level' : Tg.DispLevel.EXPERT }  ]


if __name__ == '__main__':
    classes = (BrukerEC_PS, BrukerEC_Cabinet)
    PS.tango_main( 'BrukerEC_PS', sys.argv, classes, prerun=cfg_events)

