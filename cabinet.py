#!/usr/bin/env python2.5
# -*- coding: utf-8 -*-

# Python standard library imports
import sys
import socket
from threading import Thread, RLock, Event, Semaphore
from time import time, sleep
import telnetlib
import logging
from copy import copy
import traceback
from collections import deque

# TANGO imports
from PyTango import Except, DevFailed, DevState
from PowerSupply.util import FriendlySocket, UniqList, bit_filter_msg, nop
import PowerSupply.standard as PS

# local imports
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

# how often commands shall be attempted
COMMAND_RETRY = 3
# this tries to gurantee that mass downloads AND setting succeed anyway
# COMMAND_RETRY = 20


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
    state_id = 0
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
    # when CAN bus hangs this flag will be true
    # reset it by explicit assignment or by calling update_state

    def __init__(self):
        self.log = logging.getLogger(self.__class__.__name__)
        self.can_hang = {}
        self.active_port = None
        self.comm = FriendlySocket()
        self.comm.read_timeout = 3.0 # doubled from 1.5
        # locks link to Ethernet bridge
        self.lck = RLock()
        self.log = logging.getLogger('cab')
        self.rem_vdq = factory.VDQ(False)
        self.updater = UpdateThread()

        self.error_code = 0
        # only true when the cabinet is ON

        self.desc = 'generic'
        self.stat = DevState.UNKNOWN, "unknown"
        self.init_counter = 0
        self.command_canbus_timeout = 0
        self.command_timeout = 0
        self.command_queue = deque()

    is_connected = property(lambda self: self.comm.is_connected)

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

    def restart_bsw_tcp(self):
        self.log.info('restart_bsw_tcp stub does nothing')

    def reconnect(self, reap=False):
        '''Ensures that a connection with 'host' (if configured) is etablished
           Raising an exception if fails.
           If connection is refused it is attempted to restart_bsw_tcp.
        '''
        try:
            return self.reconnect_core()

        except socket.error, err:
            if reap and err.errno==111:
                self.restart_bsw_tcp()
                return False
            else:
                raise


    def reconnect_core(self):
        '''Ensures that a connection with 'host' (if configured) is etablished
           Raising an exception if fails
        '''
        was_connected = self.is_connected
        self.comm.reconnect()

        # Determines the type of the cabinet used.
        # NOTE: from typical OO style self-modifying code such as changing
        #       the type of an object at run-time, could lead to behavior
        #       that is difficult to predict. Since the new behavior
        #       is given by a sub-class and is done only once, I hope the
        #       mess is limited. This could be moved into BrukerEC_Cabinet.
        #       For now it works fine and I will leave it as is.

        # Leaves reconnect() when type of cabinet has already been determined successfully.
        if self.__class__ is not CabinetControl: return self.is_connected
        # guranted that the following lines are executed until being successful once,
        # in the sense that no exception is thrown
        if self.type_hint==0:
            # queries first port for the power supply it controls
            ver = self.command_seq(0,'OBJ=%d'%state.COBJ_VERSION,'VAL/')[1]
            pst = int(ver,16) & TYPE_MASK
        else:
            pst = self.type_hint

        self.init_counter += 1
        # re-assigns __class__ only if execution was successful
        self.__class__, self.desc = CAB_CT[pst]
        return self.is_connected

    def get_alarms(self, mask=0):
        word = (self.error_code & ~mask)
        if self.errors is None:
            return [ 'type of cabinet not yet detected']
        return bit_filter_msg(word, self.errors)

    def reset_interlocks(self):
        raise Exception('not implemented')

    def reset_interlocks_p(self, port):
        self.can_hang[port] = False
        if self.is_connected:
            self.command_seq(port, 'RST=0')

    def check_hung(self, port, force):
        if self.can_hang.get(port, False) and not force:
            raise CanBusTimeout('CAN bus is hung, or busy')

    def command_seq(self, port, *cmd_list, **kwargs):
        """Executes a series of commands for a specific channel,
           without allowing other commands to interfere.
           This method is thread safe (unlike most others).
        """
        force = kwargs.get('force', False)
        self.check_hung(port, force)
        with self.lck:
            self.switch_port(port)
            r = []
            try:
                for cmd in cmd_list:
                    r.append(self._command(cmd, force=force))
            except CanBusTimeout:
                self.can_hang[port] = True
                raise
            return r

    def command_q(self, STAT, port, *cmd_list, **kwargs):
        print 'queuing command', port, cmd_list, kwargs
        self.command_queue.append( (STAT, port, cmd_list, kwargs) )

    def checked_command(self, port, cmd, **kwargs):
        timeout = None
        with self.lck:
            for x in xrange(COMMAND_RETRY):
                try:
                    self.switch_port(port)
                    response = self._command(cmd, **kwargs)
                    return check(port, response)
                except CanBusTimeout, cb:
                    timeout = cb
                    self.log.warn('checked_command', exc_info=1)
        self.can_hang[port] = True
        # only useful for debugging
        self.log.debug('CAN bus timeout in checked_command\n'+traceback.format_stack())
        raise timeout

    def command(self, port, cmd, **kwargs):
        """Executes a command for a specific channel.
           This method is thread safe (unlike most others).
        """
        force = kwargs.get('force', False)
        self.check_hung(port, force)
        with self.lck:
            self.switch_port(port)
            try:
                r = self._command(cmd, force=force)
                self.can_hang[port] = False
            except CanBusTimeout:
                self.can_hang[port] = True
                raise
            return r

    def _command(self, cmd, force=False):
        if not self.comm.is_connected:
            traceback.print_exc()
            raise PS.PS_Exception('control %s not connected' % self.comm.hopo)
        cmd = cmd.upper().strip()
        COM = self.comm
        txt = cmd+"\r"
        response = None

        # retries CAN bus commands COMMAND_RETRY times
        for r in range(COMMAND_RETRY):
            COM.write(txt)
            response = COM.readline()
            if not response:
                self.command_timeout += 1
                self.log.debug("timed out reading")
                msg = "timed out waiting %s for responding to %s" % \
                    (self.comm.hopo, cmd)
                raise PS.PS_Exception(msg)

            if response[0]!='*':
                break

        if response[0]=='*':
            self.command_canbus_timeout += 1
            tup =  (self.comm.host, self.active_port, cmd)
            msg = "CAN bus timed out (%s.%s %s)" % tup
            traceback.print_stack()
            raise CanBusTimeout(msg)

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
            sp = str(port)
            reply = self._command("PRT/")
            assert str(port)==reply, '%s==%s' % (sp,reply)
            self.active_port = port

    def __del__(self):
        self.disconnect_exc()

    def disconnect_exc(self):
        self.comm.disconnect_exc()
        self.active_port = None
        self.can_hang.clear()

    def switch_power(self, ps_port, bit, qstat=None):
        '''Switch power of PS on or off.
        '''
        if self.port is None:
            relay = ps_port
        else:
            relay = self.port
        cmd = 'DCP=%d' % bit
        if qstat is None:
            self.command(relay, cmd)
        else:
            self.command_q(qstat, relay, cmd)

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
        self.error_code = self.checked_command(0, 'STB/')
        rem = bool(int(self.command(0, 'REM/')))
        self.rem_vdq = factory.VDQ(rem, q=PS.AQ_VALID)
        self.stat = DevState.ON, 'cabinet ready'
        return self.error_code

    def process_command_queue(self):
        '''Executes all commands in command queue.
        '''
        while len(self.command_queue) > 0:
            print 'processing command queue', len(self.command_queue)
            try:
                STAT,port,cmd_list,kwargs = self.command_queue.popleft()
                print 'executing queued command %d %s %s' % (port,cmd_list,kwargs)
                self.command_seq(port, *cmd_list, **kwargs)
            except Exception, exc:
                msg = 'processing queued command'
                self.log.exception(msg)
                alarm_msg = str(exc)
                STAT.ALARM(alarm_msg, sticky=True)

    def telnet(self, commands, timeout=2.5):
        t0 = time()
        def to():
            y = max(t0+timeout-time(), 0.0)
            return y
        tel = telnetlib.Telnet(self.comm.host, timeout=to())
        PS1 = '# '
        tel.read_until(PS1, timeout=to())
        ret = ''
        for c in commands:
            tel.write(c+'\n')
            ret += tel.read_until(PS1, timeout=to())
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

    MACHINE_STAT = None # abstract

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
        self.error_code = self.checked_command(RELAY_PORT, 'STB/')
        self.state_id = self.checked_command(RELAY_PORT, 'STC/')
        self.stat = self.get_stat()
        rem = bool(int(self.command(RELAY_PORT, 'REM/')))
        self.rem_vdq = factory.VDQ(rem, q=PS.AQ_VALID)
        return self.error_code

    def get_stat(self):
        if self.state_id is None: return None
        if self.state_id > len(self.MACHINE_STAT):
            s = [ DevState.STANDBY, CAB_READY+' [%02d]' % self.state_id ]
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

class Bend_Cabinet(Big_Cabinet):

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
        (DS.ALARM, 'cabinet interlock'),
        (DS.INIT, 'switching power supply (buck) on'),
        (DS.INIT, 'switching power supply (4q) on...'),
        (DS.MOVING, 'switching power supply off...'),
        (DS.MOVING, 'ack. buck slave'),
        (DS.MOVING, 'ack. buck master'),
        (DS.MOVING, 'ack. quad slave'),
        (DS.MOVING, 'ack. quad master'),
        (DS.MOVING, 'resetting cabinet...'),
    )


class Quad_Cabinet(Big_Cabinet):

    DS = DevState
    MACHINE_STAT = list(Bend_Cabinet.MACHINE_STAT[0:9]) + [
        (DS.MOVING, 'resetting cabinet...'),
        (DS.UNKNOWN, '(unused)'),
        (DS.INIT, 'ack. QV01 buck'),
        (DS.INIT, 'ack. QV01 4q'),
        (DS.INIT, 'ack. QV02 buck'),
        (DS.INIT, 'ack. QV02 4q'),
        (DS.INIT, 'ack. QH01 buck'),
        (DS.INIT, 'ack. QH01 4q'),
        (DS.INIT, 'ack. QH02 buck slave'),
        (DS.INIT, 'ack. QH02 buck master'),
        (DS.INIT, 'ack. QH02 4Q slave'),
        (DS.INIT, 'ack. QH02 4Q master'),
        (DS.INIT, 'starting QV01 buck'),
        (DS.INIT, 'starting QV01 4q'),
        (DS.MOVING, 'stopping QV01'),
        (DS.INIT, 'starting QV02 buck'),
        (DS.INIT, 'starting QV02 4q'),
        (DS.MOVING, 'stopping QV02'),
        (DS.INIT, 'starting QH01 buck'),
        (DS.INIT, 'starting QH01 4q'),
        (DS.MOVING, 'stopping QH01'),
        (DS.INIT, 'startin QH02 buck master'),
        (DS.INIT, 'starting QH02 buck slave'),
        (DS.INIT, 'starting QH02 4q'),
        (DS.MOVING, 'stopping QH02'),
    ]


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
        msg = 'CAN bus timeout port %d' % port
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
    2 : ( DC_Cabinet, 'linac transferline'),
    3 : ( Quad_Cabinet, 'quadrupole' ),
    6 : ( DC_Cabinet, 'booster transferline'),
    7 : ( Wave_Cabinet, 'sextupole' ),
    8 : ( Bend_Cabinet, 'dipole' )
}

