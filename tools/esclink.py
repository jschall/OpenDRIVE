import serial
import sys
import struct

ser = serial.Serial(
    port=sys.argv[1],
    baudrate=921600
)

SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

MSG_TYPE_PARAM_REQUEST_BY_INDEX = 0
MSG_TYPE_PARAM_VALUE = 1
MSG_TYPE_PARAM_INDEX_INVALID = 2

message_formats = {}
message_formats[MSG_TYPE_PARAM_REQUEST_BY_INDEX] = "<HBH"
message_formats[MSG_TYPE_PARAM_VALUE] = "HB16sfH"
message_formats[MSG_TYPE_PARAM_INDEX_INVALID] = "<HBH"

def crc16_ccitt(crc, data):
    msb = crc >> 8
    lsb = crc & 255
    for c in data:
        x = ord(c) ^ msb
        x ^= (x >> 4)
        msb = (lsb ^ (x >> 3) ^ (x << 4)) & 255
        lsb = (x ^ (x << 5)) & 255
    return (msb << 8) + lsb

def extract_msg_id_field(buf):
    return ord(buf[1])<<8|ord(buf[0])

def extract_msg_crc16_field(buf):
    return ord(buf[-1])<<8|ord(buf[-2])

def msg_valid(buf):
    msg_id = extract_msg_id_field(buf)

    if msg_id not in message_formats.keys():
        return False

    if len(buf) != struct.calcsize(message_formats[msg_id]):
        return False

    return crc16_ccitt(0, buf[0:-2]) == extract_msg_crc16_field(buf)

def get_frame_blocking():
    buf = []
    while True:
        byte = ord(ser.read())

        if byte == SLIP_END and len(buf) > 0:
            return ''.join([chr(x) for x in buf])

        if len(buf) > 0 and byte == SLIP_ESC_END and buf[-1] == SLIP_ESC:
            buf[-1] = SLIP_END
        elif len(buf) > 0 and byte == SLIP_ESC_ESC and buf[-1] == SLIP_ESC:
            buf[-1] = SLIP_ESC
        else:
            buf.append(byte)

while True:
    buf = get_frame_blocking()
    if msg_valid(buf):
        print struct.unpack(message_formats[extract_msg_id_field(buf)], buf)
        print " ".join([x.encode('hex') for x in buf])
