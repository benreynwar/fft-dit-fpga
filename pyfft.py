# Copyright (c) 2012 Ben Reynwar
# Released under MIT License (see LICENSE.txt)

"""
Gives expected results of FFT DIT stages to compare with verilog code.
"""

import cmath
import math
from numpy import fft as nfft
from itertools import chain

def fftstages(cs):
    """
    Returns a list of the output from FFT DIT stages.

    Args:
        cs: A list of complex numbers.
       
    Returns:
        A list of lists of complex numbers.
        Each list corresponds to the ouput from a FFT DIT stage.
        The final list of complex numbers should be the correct
        FFT of the input 'cs'.
    """
    N = len(cs)
    
    if math.log(N)/math.log(2) != int(math.log(N)/math.log(2)):
        raise ValueError("Length must be a power of 2")
    if N == 1:
        return [cs]
    ess = fftstages(cs[::2])
    oss = fftstages(cs[1::2])
    stages = []
    for es, os in zip(ess, oss):
        stages.append(list(chain(*zip(es, os))))
    fs = []
    for k in range(0, len(cs)):
        tf = cmath.exp(-2*cmath.pi*1j*k/N)
        if k < len(cs)/2:
            f = ess[-1][k] + tf*oss[-1][k]
        else:
            f = ess[-1][k-N/2] + tf*oss[-1][k-N/2]
        fs.append(f)
    stages.append(fs)
    return stages

