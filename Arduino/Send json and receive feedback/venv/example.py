import time
import json
import serial
from pprint import pprint
import random

if __name__ == "__main__":
    print ("Ready...")
    ser  = serial.Serial("COM4", baudrate= 115200, timeout=1)
    data = "{\"sensor\": \"gps\", \"time\":1351824120,\"data\":[48.7897932, 2.39302]1234567890-1234567890-1234567890-1234567890-1234567890-1234567890--}"
    #data["operation"] = "sequence"



    time.sleep(5)
    data=json.dumps(data)
    print (data)
    if ser.isOpen():
        ser.write(data.encode('ascii'))
        ser.flush()
        incoming = ser.readline()
        while(incoming == 0):
            incoming = ser.readline().decode('ascii')
        print (incoming)
        ser.close()
    else:
        print ("opening error")