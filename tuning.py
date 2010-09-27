# -*- coding: utf-8 -*-

# py standard library
from time import sleep
import traceback

# TANGO imports
import PyTango as Tg
import PowerSupply.standard as PS

PSEUDO_ATTR = PS.PseudoAttr()

class Tuner(object):

    writeable = False
    attr_list = None

    def add_RegulationTuneable(self):

        @PS.AttrExc
        def read_RegulationTuneable(inst, attr):
            if inst.tuner:
                value = attr.get_write_value()
                attr.set_value(value)
            else:
                attr.set_quality(PS.AQ_INVALID)

        @PS.AttrExc
        def write_RegulationTuneable(inst, attr):
            value = attr.get_write_value()
            inst.push_change_event('RegulationTuneable', value)
            if inst.tuner:
                inst.tuner.writeable = value
            else:
                raise PS.PS_Exception('could not write attribute RegulationTuneable because initialization not finished.')

        attr = Tg.Attr('RegulationTuneable', Tg.DevBoolean, Tg.READ_WRITE)
        self.impl.add_attribute(attr,
            r_meth=read_RegulationTuneable,
            w_meth=write_RegulationTuneable)
        while True:
            try:
                self.impl.set_change_event('RegulationTuneable', True)
                break
            except ValueError:
                traceback.print_exc()
        aprop = Tg.UserDefaultAttrProp()
        aprop.set_description('whether modifying parameters related to regulation loop should be allowed or not. only set this to true when you really, really know what you are doing AND have a backup.')
        attr.set_default_properties(aprop)

    def __init__(self, impl, pstype):
        self.impl = impl
        self.pstype = pstype
        self.add_RegulationTuneable()
        # the tuneable flag is held in write_value of RegulationTuneable attribute
        self.RegulationTuneable = impl._attr('RegulationTuneable')
        self.RegulationTuneable.set_write_value(False)

        # Regulation Loop Control
        self.has_buck = len(pstype.module)>1
        master_slave = len(pstype.module)>2

        self.attr_list = []
        self.add_ports()
        if master_slave:
            self.add_ports(slave=True)

    def add_attr(self, aname, rp, slave_index, **extra):
        impl = self.impl
        extra['format'] = '%7.4f'
        index = rp.index+slave_index
        @PS.AttrExc
        def read_f(inst, attr):
            inst.obj_vdq(rp.cobj, index).set_attr(attr)

        @PS.AttrExc
        def write_f(inst, wattr):
            if not self.RegulationTuneable.get_write_value():
                raise PS.PS_Exception('tuning regulation parameters disabled')
            write_value = wattr.get_write_value()
            inst.obj_set(rp.cobj, index, repr(write_value))

        attr = Tg.Attr(aname, Tg.DevDouble, rp.rw)
        aprop = Tg.UserDefaultAttrProp()

        if 'desc' in extra:
            extra['description'] = extra['desc']
            del extra['desc']

        attr.set_disp_level(Tg.DispLevel.EXPERT)

        for key,val in extra.iteritems():
            getattr(aprop, 'set_'+key)(val)

        # retries for 10 seconds...
        for x in range(0,40):
            try:
                self.impl.add_attribute(attr, r_meth=read_f, w_meth=write_f)
                break
            except Exception, exc:
                self.impl.log.debug(str(exc), exc_info=1)
                sleep(0.25)

        attr.set_default_properties(aprop)
        self.attr_list.append(attr)

        # sets a useful write value
        if rp.rw == Tg.READ_WRITE:
            try:
                read_value = impl.obj_vdq(rp.cobj, index).value
                write_attr = impl._attr(aname)
                write_attr.set_write_value(read_value)
                impl.log.info('attribute %r added %r',aname, read_value)
            except Tg.DevFailed, fail:
                err = fail[-1]
                impl.log.info('attribute %r added but failed reading (%s)', aname, err.desc)

    def remove_all(self):
        for attr in self.attr_list:
            try:
                self.impl.remove_attribute(attr.get_name())
            except ValueError,exc:
                self.impl.log.debug(attr.get_name()+': '+str(exc))

    def add_ports(self, slave=False):
        # changing regulation parameters is disabled
        # as it is normally not done and inappropriate values might damage the power supply
        pre = 'Slave' if slave else ''
        slave_index = 2 if slave else 0
        for aname, rp in self.pstype.REG_PARAM.iteritems():
            self.add_attr(pre+aname, rp, slave_index)
