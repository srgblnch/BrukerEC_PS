#!/usr/bin/env python2.5
# -*- coding: utf-8 -*-

import sys
import socket
from threading import Thread, RLock, Event, Semaphore
from time import time, sleep
import telnetlib
import logging
from PyTango import Except, DevFailed, DevState

from ps_util import FriendlySocket, UniqList, bit_filter_msg, nop
import ps_standard as PS
from copy import copy

import factory
import state

TYPE_MASK = 0xFFFFFF

ERROR_RESPONSE = {
    "E01" : "Command %(cmd)r not supported",
    "E02" : "Invalid Argument",
    "E03" : "Port %(port)s not available",
    "E04" : "Local mode",
    "E05" : "Argument out of allowed range",
    "E06" : "Command or Mode Error",
    'E07' : 'Command Buffer Full',
    'E08' : 'Current Error',
    'E09' : 'Code Error',
    'E10' : 'Password Error'
}

PAYLOAD_OFFSET = 4

DEFAULT_EC_PORT = 3701
RELAY_PORT = 19

_CAB = None

CAB_READY = 'cabinet ready'

def instance():
    global _CAB
    if not _CAB:
        _CAB = CabinetControl()
    return _CAB

class UpdateThread(Thread):

    def __init__(self):
        Thread.__init__(self, target=self.run)
        self.daemon = True
        self.register = []
        self.busy = Event()
        self.running = True
        self.throttle = 0.2
        self.cycle = 0
        self.last_exc = None
        self.update_t = None
        # when this semaphore is equal to the number
        # of registered callbacks, the DS initialization phase is finished
        self.log = logging.getLogger('cab.up')

    def run(self):
        while self.running:
            self.cycle += 1
            self.busy.wait()
            if not self.running: return 1
            self.log.debug('starting update cycle %d',self.cycle)
            start_t = time()
            for fun in self.register:
                    if not self.running: return 2
                    try:
                        fun()
                    except Exception, exc:
                        self.last_exc = exc
            if not self.running: return 3
            end_t = time()
            diff_t = end_t - start_t
            if diff_t < self.throttle:
                sleep(self.throttle-diff_t)
            self.update_t = (start_t, end_t, diff_t)

    def add_list(self, fun_list):
      for fun in fun_list:
          self.add(fun)

    def add(self, fun):
        """Adds 'fun' to the list of registered callbacks. Each callable can
           only be called once.
        """
        if not fun in self.register:
            self.register = copy(self.register) + [fun]
            self.busy.set()
            return True
        else:
            return False

    def remove(self, fun):
        # creates an explicit copy instead of modifying existing list
        new_reg = copy(self.register)
        if fun in new_reg:
            new_reg.remove(fun)
        self.register = new_reg
        if not self.register and self.running:
            self.busy.clear()


    def __del__(self):
        self.running = False
        self.register = []
        self.busy.set()


class CabinetControl(object):
    '''Cabinet wide-status and control link.
    '''
    host = property(lambda inst: inst.comm.host)
    port = None #< port of the cabinet controller / relay board (used for resetting interlocks)
    type_hint = 0
    use_waveforms = None #< using None to indicate UNKNOWN

    #< indicates which 'port' of the PS should be used to reading cabinet-wide
    # status and which relay board to use for switching on dipole & quad
    # the default (None) indicates that there is no relay board in the system
    # and the functionality is provided by the relay board
    cab_port = None
    errors = None
    state_id = None
    # when CAN bus hangs this flag will be true
    # reset it by explicit assignment or by calling update_state
    can_hang = False

    def __init__(self):
        self.log = logging.getLogger(self.__class__.__name__)
        self.active_port = None
        self.comm = FriendlySocket()
        self.comm.read_timeout = 0.5
        # locks link to Ethernet bridge
        self.lck = RLock()
        self.log = logging.getLogger('cab')
        self.rem_vdq = factory.VDQ(False)
        self.updater = UpdateThread()

        self.error_code = 0
        # only true when the cabinet is ON

        self.desc = 'generic'
        self.stat = None
        self.init_counter = 0
  
    is_connected = property(lambda self: self.comm.is_connected)


    def all_initialized(self):
        return self.init_counter==len(self.updater.register)

    def start(self):
        self.updater.start()

    def stop(self):
        self.updater.running = False

    def restart(self):
      self.stop()
      old_register = copy(self.updater.register)
      self.updater = UpdateThread()
      self.updater.add_list(old_register)
      self.start()

    def connect(self, host, type_hint=0):
        '''Tells where to connect to. The actual connecting is done by reconnect.
        '''
        if not host:
          raise PS.PS_Exception('no IpAddress configured')
        self.log.info('connecting to '+host)
        self.comm.connect(host, DEFAULT_EC_PORT, connect=False)
        self.type_hint = type_hint


    def reconnect(self):
        '''Tries to etablish connection with 'host' (if configured).
           Returns True if the connection status went from down to up
        '''
        self.comm.reconnect()

        try:
            # Determines the type of the cabinet used.
            if self.__class__ is not CabinetControl: return self.is_connected
            if self.type_hint==0:
                # queries first port for the power supply it controls
                ver = self.command_seq(0,'OBJ=%d'%state.COBJ_VERSION,'VAL/')[1]
                pst = int(ver,16) & TYPE_MASK
            else:
                pst = self.type_hint

            self.init_counter += 1
            # decides type of cabinet based on first PS it control
            self.__class__, self.desc = CAB_CT[pst]

        except Exception:
            raise

        return self.is_connected

    def get_alarms(self, mask=0):
        word = (self.error_code & ~mask)
        if self.errors is None:
            return [ 'type of cabinet not yet detected']
        return bit_filter_msg(word, self.errors)

    def reset_interlocks(self):
        raise Exception('not implemented')

    def reset_interlocks_p(self, port):
        if self.is_connected:
            self.command_seq(port, 'RST=0') 

    def command_seq(self, port, *cmd_list, **kwargs):
        """Executes a series of commands for a specific channel,
           without allowing other commands to interfere.
           This method is thread safe (unlike most others).
        """
        with self.lck:
            self.switch_port(port)
            r = []
            for cmd in cmd_list:
                r.append(self._command(cmd))
            return r

    def command(self, port, cmd):
        """Executes a command for a specific channel.
           This method is thread safe (unlike most others).
        """
        with self.lck:
            self.switch_port(port)
            r = self._command(cmd)
            return r

    def _command(self, cmd):
        if not self.comm.is_connected:
            raise PS.PS_Exception('control %s not connected' % self.comm.hopo)
        if self.can_hang:
            raise CanBusTimeout('CAN bus is hung')
        cmd = cmd.upper().strip()
        COM = self.comm
        try:
            txt = cmd+"\r"
            COM.write(txt)
            response = COM.readline()
            if not response:
                self.log.debug("timed out reading")
                msg = "timed out waiting %s for responding to %s" % (self.comm.hopo, cmd)
                raise PS.PS_Exception(msg)

            elif response[0]=='*':
                tup =  (self.comm.host, self.active_port, cmd)
                msg = "CAN bus %s, port %s timed out, command %s" % tup
                self.can_hang = True
                raise CanBusTimeout(msg)            

        except socket.error, err:
            raise

        payload = response.strip()
        if payload.startswith("E"):
            if payload in ERROR_RESPONSE:
                msg = ERROR_RESPONSE[payload] % dict(cmd=cmd,port=self.active_port) + " ("+payload+")"
            else:
                msg = payload
            raise PS.PS_Exception(msg)
        return payload

    def switch_port(self, port):

        port = str(port)
        if port!=self.active_port:
            self._command("PRT="+port)
            assert str(port) == self._command("PRT/")
            self.active_port = port

    def __del__(self):
        self.disconnect_exc()

    def disconnect_exc(self):
        self.comm.disconnect_exc()

    def switch_power(self, ps_port, bit):
        '''Switch power of PS on or off.
        '''
        if self.port is None:
            relay = ps_port
        else:
            relay = self.port
        self.command(relay, 'DCP=%d' % bit)

    def power_on(self):
        """Switches cabinet on.
        """
        pass

    def power_off(self):
        """Switches cabinet off.
        """
        self.log.error('can not turn off cabinet without relay board')
        raise PS.PS_Exception('cabinet without relay board are always on.')

    def update_state(self):        
        self.can_hang = False
        STB = self.command(0, 'STB/')
        self.error_code = check(0, STB)
        rem = bool(int(self.command(0, 'REM/')))
        self.rem_vdq = factory.VDQ(rem, q=PS.AQ_VALID)
        return self.error_code

    def telnet(self, commands):
        tel = telnetlib.Telnet(self.comm.host, timeout=2.5)
        PS1 = '# '
        tel.read_until(PS1)
        ret = ''
        for c in commands:
            tel.write(c+'\n')
            ret += tel.read_until(PS1)
        tel.write('exit\n')
        ret += tel.read_very_eager()
        tel.close()
        ret += tel.read_all()
        # sometimes the exit command is contained in the output, sometimes not.
        self.log.debug('telnet> %r return %r', commands, ret)
        return ret


class STB_Cabinet(CabinetControl):
    errors = [
    # LSB
        True,  #< documented as bit 16
        True,  #< documented as bit 17
        True,  #< documented as bit 18
        'water interlock', #< documented as water interlock, not used for quad
# and correctors de LT
        'door interlock', #< 'door open'?
        False, #< documented as phase interlock; unused and always 1
        'emergency / PSS',
        True,
    ] + [True]*23 + ['Timeout']
#    MSB and 2 extra bytes used only for Timeout

    def reset_interlocks(self):
        self.reset_interlocks_p(0)

class DC_Cabinet(STB_Cabinet):
    use_waveforms = False

class Wave_Cabinet(STB_Cabinet):
    use_waveforms = True
    def reset_interlocks(self):
        self.reset_interlocks_p(RELAY_PORT)

class Big_Cabinet(Wave_Cabinet):

    POWER_ON_STATE = 0x06,
    POWER_OFF_STATE = 0x01,0x03

    DS = DevState
    MACHINE_STAT = (
        (DS.OFF, 'CONFIG'),
        (DS.OFF, 'cabinet off'),
        (DS.INIT, 'starting cabinet'),
        (DS.INIT, 'charging cabinet capacitors...'),
        (DS.INIT, 'cabinet getting ready'),
        (DS.STANDBY, 'STANDBY'),
        (DS.STANDBY, CAB_READY),
        (DS.MOVING, 'STOPPING'),
        (DS.ALARM, 'fault / interlock'),
        (DS.INIT, 'switching power supply (buck) on'),
        (DS.INIT, 'switching power supply (4Q) on...'),
        (DS.MOVING, 'switching power supply off...'),
        (DS.MOVING, 'ACK_BS'),
        (DS.MOVING, 'ACK_DM'),
        (DS.MOVING, 'ACK_QS'),
        (DS.MOVING, 'ACK_QM'),
        (DS.MOVING, 'discharging cabinet capacitors...'),
        (DS.MOVING, '(unused)'),
 #       (DS.MOVING, 'ack. buck
    )


# 0x0B : ACK_QS180_B1 (acknowledge Buck QS180-1)
# 0x0C : ACK_QS180_Q1 (acknowledge 4Q QS180-1)
# 0x0D : ACK_QS180_B2 (acknowledge Buck QS180-2)
# 0x0E : ACK_QS180_Q2 (acknowledge 4Q QS180-2)
# 0x0F : ACK_QS340_B (acknowledge Buck QS340)
# 0x10 : ACK_QS340_Q (acknowledge 4Q QS340)
# 0x11 : ACK_QC340_BS (acknowledge Buck QC340 Slave)
# 0x12 : ACK_QC340_BM (acknowledge Buck QC340 Master)
# 0x13 : ACK_QC340_QS (acknowledge 4Q QC340 Slave)
# 0x14 : ACK_QC340_QM (acknowledge 4Q QC340 Master)

    errors = [
    'cabinet CAN communication',
    'cabinet EPROM',
    'current RMS limit',
    'cabinet watchdog',

    'personal safety (PSS)',
    'phase',
    'door open',
    'cabinet water',
# MSB
    'cabinet on',
    'cabinet fuse',
    'emergency / PSS',
    'cabinet temperature',
    'magnet temperature',
    'magnet water flow',
    'extern 3',
    'extern 4'
    ]


    def update_state(self):
        STB = self.command(RELAY_PORT, 'STB/')
        self.error_code = check(RELAY_PORT, STB)
        STC = self.command(RELAY_PORT, 'STC/')
        self.state_id = check(RELAY_PORT, STC)
        self.stat = self.get_stat()
        rem = bool(int(self.command(RELAY_PORT, 'REM/')))
        self.rem_vdq = factory.VDQ(rem, q=PS.AQ_VALID)
        return self.error_code

    def get_stat(self):
        if self.state_id is None: return None
        if self.state_id > len(self.MACHINE_STAT):
            s = [ DS.STANDBY, CAB_READY+' [%02d]' % self.state_id ]
        else:
            s = list(self.MACHINE_STAT[self.state_id])
            s[1] = s[1].lower()
        return s

    def power_on(self):
        self.command(RELAY_PORT, 'DCP=1')

    def power_off(self):
        self.command(RELAY_PORT, 'DCP=0')

    def reset_interlocks(self):
        self.reset_interlocks_p(RELAY_PORT)

CAN_BUS_TIMEOUT_CODE = 0x800000
class CanBusTimeout(PS.PS_Exception):
    '''Exception thrown when CAN bus timesout
    '''
    pass


def check(port, response, hint=''):
    '''Checks wether timeout occured and that channel in reply matches with
       'my_ch'.
    '''
    port = int(port)
    # extracts channel and communication status from reponse
    try:
        port_recv = int(response[0:2], 16)
        state_code = int(response[2:8],16)
    except TypeError, exc:
        exc.message += repr(response)
        raise exc

    # checks wether communication timed out
    if state_code & CAN_BUS_TIMEOUT_CODE:
        msg = 'CAN bus communication with port %d timed out' % port
        if hint:
            msg+=' (%s)' % hint
        raise CanBusTimeout(msg)

    # checks wether response came for right channel
    if port!=port_recv:
        msg = 'active port should be %r but was %r' % (port, port_recv)
        raise Exception(msg)

    return state_code


# maps software types to subclasses of CabinetControl
CAB_CT = {
    1 : ( Wave_Cabinet, 'corrector' ),
    2 : ( DC_Cabinet, 'transferline'),
    3 : ( Big_Cabinet, 'quadrupole' ),
    7 : ( Wave_Cabinet, 'sextupole' ),
    8 : ( Big_Cabinet, 'dipole' )
}

