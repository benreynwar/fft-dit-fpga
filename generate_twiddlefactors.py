# Copyright (c) 2012 Ben Reynwar
# Released under MIT License (see LICENSE.txt)

import cmath, math

from jinja2 import Environment, FileSystemLoader

def f_to_istr(width, f):
    """
    f is between 0 and 1.
    If f is 1 we want binary to be 010000000 (maxno).

    Used for generating the twiddle factor module.
    """
    if f < 0 or f > 1:
        raise ValueError("f must be between 0 and 1")
    maxno = pow(2, width-2)
    return str(int(round(f * maxno)))

def make_twiddle_factor_file(N, tf_width, template_fn='twiddlefactors_N.v.t', output_fn=None):
    """
    Generates a verilog file containing a twiddle factor module from a template file.
    """
    if output_fn is None:
        output_fn = 'twiddlefactors_{0}.v'.format(N)
    env = Environment(loader=FileSystemLoader('.'))
    template = env.get_template(template_fn)
    f_out = open(output_fn, 'w')
    Nlog2 = int(math.log(N, 2))
    tfs = []
    for i in range(0, N/2):
        tf = {}
        tf['i'] = i
        v = cmath.exp(-i*2j*cmath.pi/N)
        if v.real > 0:
            tf['re_sign'] = ''
        else:
            tf['re_sign'] = '-'
            v = -v.real + (0+1j)*v.imag
        if v.imag > 0:
            tf['im_sign'] = ''
        else:
            tf['im_sign'] = '-'
            v = v.real - (0+1j)*v.imag
        tf['re'] = f_to_istr(tf_width, v.real)
        tf['im'] = f_to_istr(tf_width, v.imag)
        tfs.append(tf)
    f_out.write(template.render(tf_width=tf_width, tfs=tfs, Nlog2=Nlog2))    
    f_out.close()
    

