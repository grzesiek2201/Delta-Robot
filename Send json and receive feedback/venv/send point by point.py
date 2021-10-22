import json
import serial
import time

data_1 = {
    "index_of_point": 0,
    "interpolation": 2,
    "velocity": 10,
    "acceleration": 5,
    "coordinates": {
        "x": 1.0,
        "y": 2.0,
        "z": 3.0
    }
}

data_2 = {
    "index_of_point": 1,
    "interpolation": 3,
    "velocity": 11,
    "acceleration": 6,
    "coordinates": {
        "x": 2.0,
        "y": 3.0,
        "z": 4.0
    }
}

if __name__ == "__main__":
    serialcomm = serial.Serial(port="COM3", baudrate=9600, timeout=1)
    serialcomm.reset_output_buffer()

    data_out_1 = json.dumps(data_1)
    data_out_2 = json.dumps(data_2)

    time.sleep(1)

    try:
        serialcomm.open()
        print("Serial opened")
    except:
        print("Serial not opened")

    if serialcomm.isOpen():
        print("Sending data...")
        serialcomm.write(data_out_1.encode('ascii'))
        print(data_out_1)
        serialcomm.flush()
        print("Data sent.")

        print("Sending data...")
        serialcomm.write(data_out_2.encode('ascii'))
        print(data_out_2)
        serialcomm.flush()
        print("Data sent.")

    while serialcomm.inWaiting() == 0:
        pass  # .decode('utf-8')
    incoming = serialcomm.readall().decode('ascii')
    print(incoming)
    serialcomm.reset_input_buffer()
