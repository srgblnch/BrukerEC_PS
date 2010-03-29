#!/usr/bin/env python2.5
# -*- coding: utf-8 -*-
#
# factory.py
# This file is part of tango-ds (http://sourceforge.net/projects/tango-ds/)
#
# tango-ds is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# tango-ds is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with tango-ds.  If not, see <http://www.gnu.org/licenses/>.

from PyTango import READ, READ_WRITE, DevBoolean, DevDouble, DevString, \
    DevVoid, SCALAR, DevShort, AttrQuality, DevUShort, DevULong, DevLong
import ps_standard as PS
from ps_util import txt
from time import time

VDQ = PS.VDQ

# default ways to convert to DevXYZ valuers
CONVERT = {
    DevDouble : lambda x: float(x.replace(' ','')),
    DevString : lambda x: x,
    DevBoolean: lambda x: bool(int(x)),
    DevShort  : int,
    DevUShort : int,
    DevULong : int,
    DevLong : int
}

# defaults way to convert DevXYZ into strings for the EC commands
CONVERT_WRITE = {
    DevBoolean : int,
    DevDouble  : repr
}

DEVICE = None
CLASS = None


def start(device_class, class_class):
  global DEVICE
  global CLASS
  DEVICE = device_class
  CLASS = class_class

def add_cmd(cmdname,
    ind = [DevVoid,''],
    outd = [DevVoid,''],
    extra = None
):
    """Adds a command to the device class without adding any method
       to the DS
    """
    extra = PS.get_cmd_extra(cmdname, extra)
    desc = [ ind, outd, extra ]
    CLASS.cmd_list[cmdname] = desc

def add_ec_cmd(cmdname,
    cmdstr,
    ind = [DevVoid,''],
    outd = [DevVoid,''],
    extra = None
):
    """Adds Tango command that executes
       one EC command
    """
    add_cmd(cmdname, ind, outd, extra)

    # define command only if not defined otherwise
    if hasattr(DEVICE, cmdname): return

    @PS.ExceptionHandler
    def cfun(inst):
        return inst.cmd(cmdstr)
    setattr(DEVICE, cmdname, cfun)

def add_attribute(aname, rw=READ, tp=DevString, query='auto',
      extra=None):
    '''Adds a scalar attribute.
       If no read function is defined, create_rfun() will be used to generate
       a suitable default.
    '''
    extra = PS.get_attr_extra(aname, extra)
    rfun_name = 'read_'+aname
    CLASS.attr_list[aname] = [[tp, SCALAR, rw] , extra ]

    if not hasattr(DEVICE, rfun_name):
        # creates generic read function
        rfun = create_rfun(aname, query=query)
        setattr(DEVICE, rfun_name, rfun)

def add_ec_attr(
        aname, mnemonic=None, rw=READ, tp=DevDouble,
        extra=None,
        qconv=None,
        wconv=None,
        query='auto'
        ):
    '''Adds a scalars attribute to the server that roughly corresponds
       to a mnemonic being read or written.
       If no read_XYZ function exists, one will be created by create_rfun().
       Likewise, if no query_XYZ functions exists, one will be created.
    '''
    extra = PS.get_attr_extra(aname, extra)

    rfun_name = 'read_'+aname
    qfun_name = 'query_'+aname
    if mnemonic is None and not hasattr(DEVICE, qfun_name):
        msg = "neither mnemonic given to add_ec_attr %s nor query method defined" % (aname,)
        raise Exception(msg)

    CLASS.attr_list[aname] = [[tp, SCALAR, rw] , extra ]

    if not hasattr(DEVICE, rfun_name):
        rfun = create_rfun(aname, query=query)
        setattr(DEVICE, rfun_name, rfun)

    if not hasattr(DEVICE, qfun_name):
        if qconv is None:
          qconv = CONVERT[tp]
        qfun = create_ec_qfun(mnemonic, conv=qconv)
        setattr(DEVICE, qfun_name, qfun)

    if rw==READ: return

    wfun_name = 'write_'+aname

    if not hasattr(DEVICE, wfun_name):
        if wconv is None:
            wconv = CONVERT_WRITE.get(tp,str)
        wfun = create_ec_wfun(mnemonic, conv=wconv)
        setattr(DEVICE, wfun_name, wfun)

def create_ec_qfun( mnemonic, conv=lambda x:x):
    # attribute write function
    def qfun(inst):
        val_str = inst.cmd_seq(mnemonic+'/')[0]
        val = conv(val_str)
        return VDQ(val,q=PS.AQ_VALID)
    return qfun

def create_ec_wfun(mnemonic, conv):
    # attribute write function
    @PS.ExceptionHandler
    def wfun(inst, wattr):
        inst.write_ec_attribute(wattr, mnemonic, conv)
    return wfun

def name(f, n):
  f.__name__ = n
  return f

def create_rfun(aname, query='auto'):

    @PS.ExceptionHandler
    def rfun(inst, attr):
        aname = attr.get_name()
        vdq = inst.vdq(aname, query=query)
        vdq.set_attr(attr)
    rfun.func_name = 'read_' + aname

    return rfun

