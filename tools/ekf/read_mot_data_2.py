import csv
import sys
import struct
from math import sqrt

SLIP_END = chr(0xC0)
SLIP_ESC = chr(0xDB)
SLIP_ESC_END = chr(0xDC)
SLIP_ESC_ESC = chr(0xDD)

def slip_decode(in_buf):
    if SLIP_END not in in_buf:
        return (None, in_buf)

    out_buf = ''
    esc_flag = False
    for i in range(len(in_buf)):
        if esc_flag:
            if in_buf[i] == SLIP_ESC_ESC:
                out_buf += SLIP_ESC
            elif in_buf[i] == SLIP_ESC_END:
                out_buf += SLIP_END
            esc_flag = False
        elif in_buf[i] == SLIP_ESC:
            esc_flag = True
        elif in_buf[i] == SLIP_END:
            return (out_buf, in_buf[i+1:])
        else:
            out_buf += in_buf[i]

buf = ''

struct_fmt = '<ff'

data = {'theta_est':[],'theta_m':[]}

with open(sys.argv[1], 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    reader.next()
    for row in reader:
        buf += row[2][2:].decode('hex')
        frame, buf = slip_decode(buf)
        if frame is not None:
            frame_data = dict(zip(('theta_est','theta_m'),struct.unpack(struct_fmt,frame)))
            #frame_data['u_alpha'] *= 1./sqrt(3)
            #frame_data['u_beta'] *= 1./sqrt(3)

            #frame_data['u_alpha'] *= 1./sqrt(2./3.)
            #frame_data['u_beta'] *= 1./sqrt(2./3.)

            #frame_data['i_alpha'] = frame_data['i_alpha'] * (2./3.)/sqrt(2./3.) # for non-power-invariant transform
            #frame_data['i_beta'] = frame_data['i_beta'] * (2./3.)/sqrt(2./3.)


            for key,val in frame_data.iteritems():
                data[key].append(val)

import matplotlib.pyplot as plt


plt.plot(data['theta_m'], color='g')
plt.plot(data['theta_est'], color='b')
plt.show()

from scipy.io import savemat
savemat('mot_data.mat',data)
