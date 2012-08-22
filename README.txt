Decimation-In-Time Fast Fourier Transform

I've tried to make the implementation simple and well documented.
I have not tried to make it efficient.

dit.v - Contains main module.
buffer.v - Contains a module for a single butterfly step.

generate_twiddlefactors.py - Contains function to generate a verilog file with twiddlefactors.
twiddlefactors_N.v.t - Template used to generate verilog file.

dut_dit.v - A wrapper around the 'dit' module to allow verification with MyHDL.

qa_dit.py - A MyHDL test bench for verification.
            Requires MyHDL, iverilog and numpy to be installed.

pyfft.py - Generates output of intermediate FFT stages.  Useful for debugging.
