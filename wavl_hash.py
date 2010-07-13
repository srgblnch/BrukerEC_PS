#!/usr/bin/python

from zlib import crc32
import numpy

# maximal value used in internal representation of waveform (18 bits)
PT_NOMINAL =  0x3ffff 

# module shared between client and server
# used to generate wave id

def to_raw(I_nominal, waveform):
    '''converts waveform (floats) into integer representation used
       by loading waveforms to Bruker power supply
       @param I_nominal nominal current of power supply
       @param waveform input waveform
    '''
    return tuple( int( (y*PT_NOMINAL)/I_nominal ) for y in waveform )

def calc_hash(I_nominal, waveform):
    '''Calculates hash value for the specified waveform.
       @param I nominal current of the power supply
       @param waveform input waveform
    '''
    dat = (int(I_nominal),) + to_raw(I_nominal, waveform)
    ar = numpy.array(dat, numpy.int32)
    return crc32(ar.data)

def hex_u32(value):
    """Converts a hash code into string of unsigned hexadecimal digits.       
       -1 will be 2*31 result in fffffffff
    """
    if value < 0:
        value += 2**32
    return "%x" % value

def unhex_i32(string):
  """Converts string of (unsigned) hexadecimal digits back into signed 32 bit integer
  """
  val = int(string,16)
  if val >=2**31:
      val -= 2**32
  return val 

def fingerprint(I_nominal, waveform):
    '''Returns fingerprint (str) that should be shown to the user.
       @param I_nominal nominal current of power supply
       @param waveform input waveform
    '''
    hash = calc_hash(I_nominal, waveform)
    return hex_u32(hash)

if __name__ == '__main__':
    for x in (-1, -2**31, 0, 2**31-1, 3):
	    print x, hex_u32(x), unhex_i32(hex_u32(x))
    import sys
    if len(sys.argv) < 3:
        print 'usage: ', sys.argv[0], ' I_nominal FILE_NAME'
        sys.exit(1)
    I = float(sys.argv[1])
    wav_fname = sys.argv[2]
    wav = numpy.loadtxt(wav_fname)
    print fingerprint(I, wav)

