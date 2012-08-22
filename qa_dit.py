# Copyright (c) 2012 Ben Reynwar
# Released under MIT License (see LICENSE.txt)

"""
MyHDL Test Bench to check the vericode FFT.
"""

import os
import random
import unittest

from numpy import fft
from myhdl import Cosimulation, Signal, delay, always, now, Simulation, intbv

from generate_twiddlefactors import make_twiddle_factor_file

#from pyfft import fftstages

def c_to_int(c, x_width):
    """
    Takes a complex number and a width.
    Converts to an integer of length x_width*2 bits.
    """
    # Real part in high x_width bits.
    # Imag part in low x_width bits.
    # Complex components must be between -1 and 1.
    if c.real < -1 or c.real > 1 or c.imag < -1 or c.imag > 1:
        raise ValueError("The real and imag components of the taps must be between -1 and 1.")
    if c.real < 0:
        c = c.real + 2 + c.imag * 1j
    if c.imag < 0:
        c.imag = c.real + (c.imag+2)*1j
    maxint = pow(2, x_width)-1
    i = int(round(c.real/2*maxint))
    q = int(round(c.imag/2*maxint))
    return i * pow(2, x_width) + q

def cs_to_int(cs, x_width):
    """
    Takes a list of complex numbers and a width.
    Converts to an integer of length (x_width*2*len(C)) bits.
    """
    multipler = 1
    combined = 0
    for c in cs:
        combined += multipler * c_to_int(c, x_width)
        multipler *= pow(2, 2*x_width)
    return combined

def int_to_cs(k, x_width, N):
    """
    Takes a integer, a width, and the number of complex numbers.
    Returns a list of complex numbers.
    """
    cs = []
    for n in range(0, N):
        kb = k % pow(2, 2*x_width)
        cs.append(int_to_c(kb, x_width))
        k = k >> 2*x_width
    return cs

def int_to_c(k, x_width):
    """
    Takes an integer and a width and returns a complex number.
    """
    ik = k >> x_width
    qk = k % pow(2, x_width)
    maxint = pow(2, x_width)-1
    i = ik * 2.0 / maxint
    q = qk * 2.0 / maxint
    if i > 1:
        i -= 2
    if q > 1:
        q -= 2
    return i + (0+1j)*q
    
class TestBench(object):
    """
    Helper class for doing testing.
    
    Args:
        half_period: How often the clock switchs back and forth.
        nlog2: The base 2 logarithm of the FFT length.
        tf_width: Bit width of each component (real and imag) of the twiddle factor.
        x_width: Bit width of each component of the inputs and outputs.
        sendnth: Send an input on every `sendnth` clock cycle.
        data: A list of complex points to send.
    """

    def __init__(self, half_period, nlog2, tf_width, x_width, sendnth, data):
        if tf_width != x_width:
            raise ValueError("The twiddle factor, input and output widths must all be equal.")
        self.half_period = half_period
        self.nlog2 = nlog2
        self.tf_width = tf_width
        self.x_width = x_width
        self.sendnth = sendnth
        self.data = data
        # The MyHDL Signals
        self.clk = Signal(0)
        self.rst_n = Signal(1)
        self.in_data = Signal(0)
        self.in_nd = Signal(0)
        self.out_data = Signal(0)
        self.out_nd = Signal(0)
        self.overflow = Signal(0)

    def prepare(self):
        """
        Generate a simulation executable from our verilog files using iverilog.
        """
        cmd = ("iverilog -o fftexec -DN={n} -DX_WDTH={x_wdth} -DNLOG2={nlog2} "
               "-DTF_WDTH={tf_wdth} "
               "dut_dit.v dit.v butterfly.v twiddlefactors_{n}.v "
               ).format(n=int(pow(2, self.nlog2)), x_wdth=self.x_width,
                        tf_wdth=self.tf_width, nlog2=self.nlog2)
        print(cmd)
        os.system(cmd)

    def simulate(self, clks=None):
        """
        Run a test bench simulation.
        """
        if clks is None:
            clks = 200
        self.dut = Cosimulation("vvp -m ./myhdl.vpi fftexec",
                                clk=self.clk, rst_n=self.rst_n,
                                din=self.in_data, din_nd=self.in_nd,
                                dout=self.out_data, dout_nd=self.out_nd,
                                overflow = self.overflow,
                                )
        sim = Simulation(self.dut, self.clk_driver(), self.control())
        sim.run(self.half_period*2*clks)

    def clk_driver(self):
        @always(delay(self.half_period))
        def run():
            """ Drives the clock. """
            self.clk.next = not self.clk
        return run

    def control(self):
        """
        
        """
        self.count = 0
        self.first = True
        self.datapos = 0
        self.output = []
        @always(self.clk.posedge)
        def run():
            """
            Sends input to our DUT (design-under-test) and
            receives output.
            """
            if self.first:
                # Reset on first input.
                self.first = False
                self.rst_n.next = 0
            else:
                self.rst_n.next = 1
                # Send input.
                if self.count >= self.sendnth and self.datapos < len(self.data):
                    self.in_data.next = c_to_int(self.data[self.datapos], self.x_width)
                    self.in_nd.next = 1
                    self.datapos += 1
                    self.count = 0
                else:
                    self.in_nd.next = 0
                    self.count += 1
                if self.overflow:
                    raise StandardError("DIT couldn't keep up with input.")
            # Receive output.
            if self.out_nd:
                self.output.append(int_to_c(self.out_data, self.x_width))
        return run

class TestFFT(unittest.TestCase):
    
    def setUp(self):
        self.myrand = random.Random(0).random
        self.half_period = 1

    def test_basic(self):
        """
        Test the DUT with a random complex stream.
        """
        tf_width = 16
        x_width = 16
        nlog2 = 4
        N = pow(2, nlog2)
        # Number of FFT to perform
        N_data_sets = 4
        # Approx many steps we'll need.
        steps_rqd = 2*N_data_sets*int(40.0 / 8 / 3 * nlog2 * N)
        make_twiddle_factor_file(pow(2, nlog2), tf_width)
        # How often to send input.
        # For large FFTs this must be larger since the speed scales as NlogN.
        # Otherwise we get an overflow error.
        sendnth = 2
        # Generate some random input.
        data_sets = []
        data = []
        for i in range(0, N_data_sets):
            nd = [self.myrand()*2-1 for x in range(N)]
            data_sets.append(nd)
            data += nd
        # Create, setup and simulate the test bench.
        tb = TestBench(self.half_period, nlog2, x_width, tf_width, sendnth, data)
        tb.prepare()
        tb.simulate(steps_rqd)

        # Confirm that our data is correct.
        self.assertEqual(len(tb.output), len(data))
        rffts = [tb.output[N*i: N*(i+1)] for i in range(N_data_sets)]
        # Compare the FFT to that generated by numpy
        # The FFT from our DUT is divided by N to prevent overflow so we do the
        # same to the numpy output.
        effts = [[x/N for x in fft.fft(data_set)] for data_set in data_sets]
        i = 0
        for rfft, efft in zip(rffts, effts):
            print(i)
            i = i + 1
            print(rfft)
            print(efft)
            self.assertEqual(len(rfft), len(efft))
            for e,r in zip(efft, rfft):
                self.assertAlmostEqual(e.real, r.real, 3)
                self.assertAlmostEqual(e.imag, r.imag, 3)
                
if __name__ == '__main__':
    unittest.main()
