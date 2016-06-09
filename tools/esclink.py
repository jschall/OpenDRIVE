import serial
import sys

ser = serial.Serial(
    port=sys.argv[1],
    baudrate=921600
)

SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

def parse_msg(buf):
    print [x.encode('hex') for x in buf]

buf = []
while True:
    buf.append(ser.read())
    if ord(buf[-1]) == SLIP_END:
        buf.pop()
        parse_msg(buf)
        buf = []
