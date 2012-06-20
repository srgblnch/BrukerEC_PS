#!/usr/bin/python
# -*- coding: utf-8 -*-

# Python standard library imports
from copy import copy
from collections import deque
from errno import ECONNREFUSED
import logging
from threading import Thread, RLock, Event, Semaphore
from time import time, sleep
import telnetlib
import traceback
import socket
import sys

# TANGO imports
from PyTango import Except, DevFailed, DevState
from PowerSupply.util import FriendlySocket, UniqList, bit_filter_msg, nop
import PowerSupply.standard as PS

# local imports
import factory
import state

PM = PS.MSG

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

def command_pause_retry(n):
    '''sleep for nth command attempt'''
    sleep(n*0.05)

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
        self.log = None

    def run(self):
        while self.running:
            self.cycle += 1
            self.busy.wait()
            if not self.running: return 1
            # self.log.debug('starting update cycle %d',self.cycle)
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


class CabinetControl(state.Module):
    '''Cabinet wide-status and control link.
    '''
    name = 'cabinet'
    state_id = 0
    host = property(lambda inst: inst.comm.host)
    port = None #< port of the cabinet controller / relay board (used for resetting interlocks)
    type_hint = 0
    use_waveforms = None #< using None to indicate UNKNOWN
    stat = DevState.UNKNOWN, 'unknown'

    # which alarm bits shall be used

    #< indicates which 'port' of the PS should be used to reading cabinet-wide
    # status and which relay board to use for switching on dipole & quad
    # the default (None) indicates that there is no relay board in the system
    # and the functionality is provided by the relay board
    # when CAN bus hangs this flag will be true
    # reset it by explicit assignment or by calling update_state


    def __init__(self):
        state.Module.__init__(self, None)
        self.can_hang = {}
        self.active_port = None
        self.comm = FriendlySocket()
        self.comm.read_timeout = 2.5
        # locks link to Ethernet bridge
        self.lck = RLock()
        self.rem_vdq = factory.VDQ(False)

        self.error_code = 0
        self.state_id = None
        self.update_t = None
        # only true when the cabinet is ON
        self.updater = UpdateThread()

        self.desc = 'generic'
        self.stat = (DevState.UNKNOWN, 'unknown')
        self.command_queue = deque()

        # counters
        self.init_counter = 0
        self.command_counter = 0
        # counting can bus level timeouts
        self.command_canbus_timeout = 0
        # counting socket level timeouts
        self.command_timeout = 0
        self.__log = None
        self.telnet_connection = None

    def set_logger(self, l):
        self.__log = l
        self.updater.log = l

    log = property( lambda self: self.__log, set_logger )

    is_connected = property(lambda self: self.comm.is_connected)

    def start(self):
        if self.update is None:
            self.updater = UpdateThread()
        self.updater.start()

    def stop(self):
        if self.updater is None:
            return
        else:
            self.updater.running = False

    def restart(self):
      self.stop()
      old_register = copy(self.updater.register)
      self.updater = UpdateThread()
      self.updater.add_list(old_register)
      self.start(re=True)

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
            if reap and err.errno==ECONNREFUSED:
                self.restart_bsw_tcp()
                return False
            else:
                raise
        except Exception:
            traceback.print_exc()
            #raise


    def reconnect_core(self):
        '''Ensures that a connection with 'host' (if configured) is etablished
           Raising an exception if fails
        '''
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

    def customize_xi(self, interlocks):
        # standard cabinet has not external interlocks
        pass

    def reset_interlocks(self):
        self.log.debug('reset_interlocks stub')

    def reset_interlocks_p(self, port):
        self.can_hang[port] = False
        if self.is_connected:
            self.command_seq(port, 'RST=0')

    def check_hung(self, port, force):
        if port is None:
            return False
        elif self.can_hang.get(port, False) and not force:
            raise CanBusTimeout('CAN bus is hung, or busy')

    def capture_can_bus_timeout(self, cmd):
        msg = 'CAN bus timeout on port {0} {1}\n'.format(self.active_port, cmd)
        self.traceback = ( msg + '\n'.join(traceback.format_stack()) )
        self.log.warn(msg, exc_info=1)
        self.command_canbus_timeout += 1

    def raise_can_bus_timeout(self, cmd):
        self.can_hang[self.active_port] = True
        msg = 'CAN bus timeout on port {0} {1}\n'.format(self.active_port, cmd)
        raise CanBusTimeout(msg)

    def _command(self, cmd):
        '''Executes a single command.
           this command is not thread-safe!
        '''
        if not self.comm.is_connected:
            traceback.print_exc()
            raise PS.PS_Exception('control %s not connected' % self.comm.hopo)
        cmd = cmd.upper().strip()
        COM = self.comm
        txt = cmd+"\r"
        response = None
        self.command_counter += 1

        # retries CAN bus commands COMMAND_RETRY times
        for r in range(COMMAND_RETRY):
            # communication pause before retry
            command_pause_retry(r)
            COM.write(txt)
            response = COM.readline()
            # indicates TCP socket timeout
            if not response:
                self.command_timeout += 1
                self.log.debug("timed out reading")
                msg = "timed out waiting %s for responding to %s" % \
                    (self.comm.hopo, cmd)
                raise PS.PS_Exception(msg)

            if response[0]=='*':
                self.capture_can_bus_timeout(cmd)
            else:
                break

        if response[0]=='*':
            self.raise_can_bus_timeout(cmd)

        payload = response.strip()
        if payload.startswith("E"):
            if payload in ERROR_RESPONSE:
                msg = ERROR_RESPONSE[payload] % dict(cmd=cmd, port=self.active_port) + " ("+payload+")"
            else:
                msg = payload
            raise PS.PS_Exception(msg)
        return payload

    def checked_command(self, port, cmd, **kwargs):
        code = None
        with self.lck:
            self.switch_port(port)
            for r in range(COMMAND_RETRY):
                command_pause_retry(r)
                response = self._command(cmd, **kwargs)
                code = check(port, response)
                if code & CAN_BUS_TIMEOUT_CODE:
                    self.capture_can_bus_timeout(cmd)
                else:
                    return code

        if code & CAN_BUS_TIMEOUT_CODE:
            self.raise_can_bus_timeout(cmd)
        else:
            return code

    def switch_port(self, port):
        if port is None: return
        port = str(port)
        if port!=self.active_port:
            self._command("PRT="+port)
            sp = str(port)
            reply = self._command("PRT/")
            assert str(port)==reply, '%s==%s' % (sp,reply)
            self.active_port = port

    # command execution
    def command_seq(self, port, *cmd_list, **kwargs):
        """Executes a series of commands for a specific channel,
           without allowing other commands to interfere.
        """
        force = kwargs.get('force', False)
        self.check_hung(port, force)
        with self.lck:
            self.switch_port(port)
            r = []
            for cmd in cmd_list:
                r.append(self._command(cmd))
            return r

    def command_q(self, STAT, port, *cmd_list, **kwargs):
        self.command_queue.append( (STAT, port, cmd_list, kwargs) )

    def command(self, port, cmd, **kwargs):
        """Executes a command for a specific channel.
        """
        return self.command_seq(port, cmd, **kwargs)[0]

    def __del__(self):
        self.disconnect_exc()

    def disconnect_exc(self):
        self.comm.disconnect_exc()
        self.active_port = None

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
        raise PS.PS_Exception('cabinet without relay boards are always on.')

    def update_fail(self, exc):
        self.stat = DevState.ALARM, str(exc)

    def update(self):
        try:
            # querying state info
            start_t = time()
            self.state_id = None
            self.error_code = self.checked_command(0, 'STB/')
            self.update_t = (start_t+time()) / 2.0
            rem = bool(int(self.command(0, 'REM/')))
            self.rem_vdq = factory.VDQ(rem, q=PS.AQ_VALID)
            self.update_stat2()
        except Exception, exc:
            self.update_fail(exc)
            raise

    def process_command_queue(self):
        '''Executes all commands in command queue.
        '''
        while len(self.command_queue) > 0:
            try:
                STAT,port,cmd_list,kwargs = self.command_queue.popleft()
                msg = 'executing queued command {0} {1} {2}'.format(port,cmd_list,kwargs)
                self.log.info(msg)
                self.command_seq(port, *cmd_list, **kwargs)
            except Exception, exc:
                msg = 'processing queued command'
                self.log.exception(msg)
                alarm_msg = str(exc)
                STAT.ALARM(alarm_msg, sticky=True)

    def telnet(self, commands):
        try:
            self.TelnetTimeout = 29.0
            tfin = time()+self.TelnetTimeout
            def to():
                return max(tfin-time(), 0.01)
            PS1 = '# '
            if self.telnet_connection is None:
                self.telnet_connection = telnetlib.Telnet(self.comm.host, timeout=to())
                self.telnet_connection.read_until(PS1, timeout=to())
            tel = self.telnet_connection
            ret = ''
            for c in commands:
                tel.write(c+'\n')
                ret += tel.read_until(PS1, timeout=to())
            sleep(0.05)
            ret += tel.read_very_eager()
            # sometimes the exit command is contained in the output, sometimes not.
            self.log.debug('telnet> %r return %r', commands, ret)
            return ret
        except Exception:
            self.log.exception('telnet({0})'.format(commands))
            try:
                tel.close()
            except Exception:
                self.log.exception('closing telnet')
            self.telnet_connection = None
            raise


class STB_Cabinet(CabinetControl):
    name = 'STB cabinet (abstract)'
    errors = [
    # LSB
        True,  #< documented as bit 16
        True,  #< documented as bit 17
        True,  #< documented as bit 18
        PM.CABINET_WATER, #< cabinet water interlock (only for bending)
        PM.DOOR, #< 'door open'?
        False, #< documented as phase interlock; unused and always 1
        PM.EMERGENCY_STOP,
        True,
    ] + [True]*23 + ['Timeout']
#    MSB and 2 extra bytes used only for Timeout

    def reset_interlocks(self):
        self.reset_interlocks_p(0)


class DC_Cabinet(STB_Cabinet):
    name = 'cabinet transferline'
    alias = ('cabt', 'cablt', 'cabbt')
    use_waveforms = False

class Wave_Cabinet(STB_Cabinet):
    name = 'cabinet correctors'
    alias = ('cab1', 'cabcor', 'cabc')
    use_waveforms = True
    def reset_interlocks(self):
        self.reset_interlocks_p(RELAY_PORT)

class Sex_Cabinet(Wave_Cabinet):
    name = 'cabinet sextupoles'
    alias = ('cab6', 'cabsex', 'cabs')
    errors = copy(Wave_Cabinet.errors)
    errors[3] = False

class Big_Cabinet(Wave_Cabinet):
    name = 'cabinet big (abstract)'
    mask = 0
    machine_stat = None # abstract

    errors = [
    # LSB
        'cabinet CAN bus communication',
        'cabinet EPROM',
        'current RMS limit',
        'cabinet watchdog',

        'personal safety (PSS)',
        'phase',
        PM.DOOR,
        PM.CABINET_WATER,
    # MSB
        'cabinet on',
        'cabinet fuse',
        'emergency / PSS',
        'cabinet temperature',
        PS.MSG.EXTERNAL_INTERLOCK.format(1),
        PS.MSG.EXTERNAL_INTERLOCK.format(2),
        PS.MSG.EXTERNAL_INTERLOCK.format(3),
        PS.MSG.EXTERNAL_INTERLOCK.format(4),
    ]

    def customize_xi(self, interlocks):
        self.errors[12:12+len(interlocks)] = interlocks

    def update(self):
        try:
            # querying state info
            start_t = time()
            self.error_code = self.checked_command(RELAY_PORT, 'STB/')
            self.state_id = self.checked_command(RELAY_PORT, 'STC/')
            self.update_t = (start_t + time() ) / 2.0
            rem = bool(int(self.command(RELAY_PORT, 'REM/')))
            self.rem_vdq = factory.VDQ(rem, q=PS.AQ_VALID)
            self.update_stat2()
        except Exception, exc:
            self.update_fail(exc)
            raise

    def power_on(self):
        self.command(RELAY_PORT, 'DCP=1')

    def power_off(self):
        self.command(RELAY_PORT, 'DCP=0')

    def reset_interlocks(self):
        self.reset_interlocks_p(RELAY_PORT)

class Bend_Cabinet(Big_Cabinet):

    name = 'cabinet bending'
    alias = ('cabb', 'cab2')
    DS = DevState
    machine_stat = (
        (DS.OFF, 'CONFIG'),
        (DS.OFF, 'cabinet off'),
        (DS.INIT, 'starting cabinet'),
        (DS.INIT, 'charging cabinet capacitors...'),
        (DS.INIT, 'cabinet getting ready'),
        (DS.STANDBY, 'STANDBY'),
        (DS.STANDBY, CAB_READY),
        (DS.MOVING, 'STOPPING'),
        (DS.ALARM, 'cabinet fault'),
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

    name = 'cabinet quadrupoles'
    alias = ('cabq', 'cab4')
    DS = DevState
    machine_stat = list(Bend_Cabinet.machine_stat[0:9]) + [
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
        (DS.INIT, 'starting QH02 buck master'),
        (DS.INIT, 'starting QH02 buck slave'),
        (DS.INIT, 'starting QH02 4q'),
        (DS.MOVING, 'stopping QH02'),
    ]

CAN_BUS_TIMEOUT_CODE = 0x800000

class CabinetException(PS.PS_Exception):
    code = None

class CanBusTimeout(PS.PS_Exception):
    '''Exception thrown when CAN bus timesout
    '''
    code = 0x10000

def check(port, response):
    '''Converts text response in integer and checks whether the specified
       channel matches the response'.
    '''
    port = int(port)
    # extracts channel and communication status from reponse
    try:
        port_recv = int(response[0:2], 16)
        state_code = int(response[2:8],16)
    except TypeError, exc:
        exc.message += repr(response)
        raise exc

#    # checks wether communication timed out
#    if state_code & CAN_BUS_TIMEOUT_CODE:
#        msg = 'CAN bus timeout port %d' % port
#        if hint:
#            msg+=' (%s)' % hint
#        raise CanBusTimeout(msg)

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
    7 : ( Sex_Cabinet, 'sextupole' ),
    8 : ( Bend_Cabinet, 'dipole' )
}

state.MODULE_REGISTRY.register(Wave_Cabinet(), DC_Cabinet(), Quad_Cabinet(), Sex_Cabinet(),  Bend_Cabinet())

BOARD_LIST = state.BOARD_LIST + [
    state.MODULE_REGISTRY.get('cab6') ,
    state.MODULE_REGISTRY.get('cab4') ,
    state.MODULE_REGISTRY.get('cab2') ,
]

def main():
    import sys, optparse
    OP = optparse.OptionParser()
    opt, args = OP.parse_args(sys.argv)
    if len(args) < 2:
        print 'usage: {0} BOARD CODE'.format(args[0])
        print 'known regulation boards\n   ',
        print '\n    '.join(state.MODULE_REGISTRY.ls())

    else:
        mod = state.MODULE_REGISTRY.get(args[1])
        code = eval(args[2], {}, {})
        mod.state_id, mod.error_code = state.analyze_error_code(code)
        mod.update_stat2()
        print state.format_module_stat(mod)

if __name__ == "__main__":
    main()
