#!/usr/bin/python
# -*- coding: utf-8 -*-

# wavl.py
# TANGO Device Server (http://sourceforge.net/projects/tango-ds/)
#
# Copyright (c) 2009 by None
#
# GNU General Public Licence (GPL)
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc., 59 Temple
# Place, Suite 330, Boston, MA  02111-1307  USA

'''This module contains functionality related uploading and downloading
waveforms for the Bruker power supplies from ALBA.
In includes mainly WaveformLoader class and two Thread classes managing
the upload respective download of a waveform.
'''

from __future__ import with_statement

# imports from Standard Library
import os, os.path
from time import sleep, time
from threading import Lock, Thread
from collections import deque
from types import NoneType
import numpy as numpy
from zlib import crc32
from functools import partial

# extra modules
import PowerSupply.util as PU
import PowerSupply.standard as PS

# local imports
from wavl_hash import *

WAVE_PORT = 3702
TERM = '\r\n'
MAX_WAVE_LEN = 16384
AVG_TIME_PER_SAMPLE = 0.0013*3
# minimal time required for download plus safety margin
TIME_BASE = 0.45+1.0
# waveform points can have signed integer values from 0x80000 up to 0x7ffff
# value corresponding to nominal setting
PT_MAX = 0x7ffff
PT_MIN = -PT_MAX

READY = 'ready'

_WAVL = None #< recent most created wave form loader

def instance():
    global _WAVL
    if not _WAVL:
        _WAVL = WaveformLoader()
    return _WAVL

def dump_to_file(self, wup, wdown, attempt):
        '''never raises an Exception
        '''
        try:
            dirname = os.path.join(self.dumpdir, str(os.getpid()))
            try:
                os.makedirs(dirname)
            except OSError, err:
                self.log.debug('making new dump dir failed: ' + str(err))
            fname_up = os.path.join(dirname, str(attempt)+'-up.txt')
            fname_down = os.path.join(dirname, str(attempt)+'-down.txt')
            open(fname_up, 'w').write('\n'.join(map(repr,wup)))
            open(fname_down, 'w').write('\n'.join(map(repr,wdown)))
            self.log.info('dumped waveforms into {0}'.format(dirname))
        except Exception, exc:
            self.log.warn(
                'verification failed AND ' +
                'exception while dumping waveforms into {0}'.format(dirname),
                exc_info=1)
            return exc

class Load(object):
    def __init__(self, port):
        self.port = port

    __repr__ = lambda self: self.__str__


class Download(Load):

    BASE_MSG = 'download'

    def __init__(self, port, wave, data):
        Load.__init__(self, port)
        self.wave = wave
        self.data = data

    def __str__(self):
      return 'Download(port %d, %d pt)' % (self.port, len(self.data))

    def run(self, impl, wavl):
        impl.ramp_off()
        if impl.is_ramping():#it would have to have change to 'off' with ramp_off() call
            raise PS.PS_Exception('Trying to download a waveform when ramping')
        if impl.is_power_on():
            raise PS.PS_Exception('Trying to download a waveform when power on')
        rval = wavl.download(self.port, self.data)
        impl.push_wave_down(self.wave)
        return rval

class Upload(Load):

    BASE_MSG = 'upload'

    def __init__(self, port, maxlen=None):
        Load.__init__(self, port=port)
        self.maxlen = maxlen

    def __str__(self):
      return 'Upload(port %d, %s pt)' % (self.port, self.maxlen)

    def run(self, impl, wavl):
      rval = wavl.upload(self.port, maxlen=self.maxlen)
      impl.push_wave_up(self, rval)
      return rval

class WaveformException(PS.PS_Exception):

    def __init__(self, response):
        self.response = response
        self.code = int(response[2:4])
        text = response[5:]
        PS.PS_Exception.__init__(self, text)

class LoadException(PS.PS_Exception):
    pass

def bebusy(msg):

    def wrapper_maker(f):
        def wrap_busy(self, *args, **kwargs):

            self.busy = msg
            try:
                return f(self, *args,**kwargs)
            finally:
                self.busy = None
        return wrap_busy

    return wrapper_maker

class WaveformLoader(object):
    '''Managing the thread for up- and downloading waveforms.
    '''

    active_load = None
    busy = None

    # how many times verification shall be attempted before generating an error
    verify = 3

    def __init__(self):
        global _WAVL
        _WAVL = self
        self.log = None
        self.sok = PU.FriendlySocket()
        self.sok.reconnect_delay = 20
        self.sok.connect_timeout = 2.0
        # up- and downloads set their own timeouts
        self.sok.read_timeout = TIME_BASE
        self.dumpdir = '/tmp/BrukerEC-wavl/'

    is_connected = property(lambda self: self.sok.is_connected)

    def connect(self, host):
        self.sok.connect(host, WAVE_PORT)

    def disconnect_exc(self):
        self.sok.disconnect_exc()

    def reconnect(self):
        self.sok.reconnect()

    def __del__(self):
        self.disconnect_exc()

    @bebusy('uploading ramp')
    def upload(self, ch, maxlen=None):
        '''reads waveform from hardware, discarding the last point.
           since the server appends a copy of the first point to then end.
        '''
        sok = self.sok
        if not sok.is_connected:
            raise PS.PS_Exception('not connected')
        cmd_start_ul = 'u'+str(ch)+TERM
        wave = deque()
        tmout = AVG_TIME_PER_SAMPLE*maxlen + TIME_BASE
        self.log.debug('starting upload %s, timeout %s', ch, tmout)
        start_t = time()
        self.sok.write(cmd_start_ul)
        if maxlen is None: maxlen = MAX_WAVE_LEN
        self.sok.timeout = tmout
        try_again = True
        while True:
            payload = self.sok.readline().strip()
            if payload.startswith('ok'):
                break

            elif payload == 'er08' and try_again:
                self.log.debug('was busy, trying again')
                try_again = False
                self.sok.write(cmd_start_ul)
                continue

            elif payload.startswith('e'):
                raise WaveformException(payload)

            elif payload == '':
                self.sok.disconnect_exc()
                raise PS.PS_Exception('disconnected from wave loader')
            else:
                try:
                    wave.append(int(payload))
                except Exception:
                    raise PS.PS_Exception('strange payload %r in upload, read %d points so far' % (payload, len(wave)))

        self._duration = time()-start_t
        wave = tuple(wave)
        if maxlen and maxlen<len(wave):
            msg = 'upload yielded %d points, expected maximal %d' % (len(wave), maxlen)
            raise PS.PS_Exception(msg)

        self.log.debug('upload time %s s for %s pt', self._duration, len(wave))
        return wave


    def check_waveform(self, dat):
        """Generates exception when trying to transmit values that are out of
           valid range, which is given by PT_MAX (two times the nominal value).
        """
        max_dat = max(dat)
        if max_dat > PT_MAX:
            idx = dat.index(max_dat)
            msg = 'waveform point %d too big: %d > %d' % (idx, max_dat, PT_MAX)
            raise PS.PS_Exception(msg)

        min_dat = min(dat)
        if min_dat < PT_MIN:
            idx = dat.index(min_dat)
            msg = 'waveform point %d too small: %d < %d' % (idx, min_dat, PT_MIN)
            raise PS.PS_Exception(msg)

    @bebusy('downloading ramp')
    def download(self, ch, wave):
        '''Transmits waveform to control unit.
        '''
        if not self.sok.is_connected:
            raise PS.PS_Exception('not connected')

        wave = tuple(int(w) for w in wave)
        tmout = self.sok.timeout
        self.log.debug('starting download %s, timeout %s s', ch, tmout)
        self.sok.timeout = self.sok.read_timeout
        start_dl_cmd = 'd'+str(ch)+TERM
        self.sok.write(start_dl_cmd)
        buf = ''.join(str(w)+TERM for w in wave) + ';' + TERM
        self.log.debug('buf contains %d characters' % len(buf))
        start_t = time()
        self.sok.timeout = AVG_TIME_PER_SAMPLE * len(wave) + TIME_BASE
        self.sok.write(buf)
        ret = self.sok.readline().strip()
        if not ret.startswith('ok'):
            raise WaveformException(ret)
        duration = time()-start_t
        self.log.info('download {0} finished after {1} s'.format(ch, duration))
        if self.verify:
            self.log.debug('verifying...')
            success = False
        else:
            success = True

        for r in range(1,1+self.verify):
            upstart_t = time()
            wup = self.upload(ch,len(wave))
            up_duration = time()-upstart_t
            if tuple(wup)==tuple(wave):
                self.log.info('verification succeeded at {0}. attempt'.format(r))
                success = True
                break
            else:
                self.log.warn('verification failed, attempt {0}'.format(r))
                dump_to_file(self, wup, wave, r)

        if not success:
            raise LoadException('verification failed')
        self._duration = duration
        return ret

    @bebusy('cancel')
    def cancel(self):
        self.sok.write('!'+TERM)
        r = self.sok.readline().strip()
        self.log.debug('cancel: %s',r)
        return r

