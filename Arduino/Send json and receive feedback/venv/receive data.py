import json
import serial
import time


serialcomm = serial.Serial('COM4', 9600, timeout=1)
serialcomm.reset_input_buffer()

while True:
    data = serialcomm.readline().decode("utf-8")
    try:
        dict_json = json.loads(data)
        print(dict_json)
    except json.JSONDecodeError as e:
        print("JSON: ", e)
    time.sleep(2)
    serialcomm.reset_input_buffer()
