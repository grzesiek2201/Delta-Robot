import json
import serial
import time


data = {
    "index_of_point": ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14"],
    "interpolation": [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
    "velocity": [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
    "acceleration": [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
    "coordinates": {
        "x": [11.1, 22.2, 33.3, 44.4, 55.5, 11.1, 22.2, 33.3, 44.4, 55.5, 11.1, 22.2, 33.3, 44.4, 55.5],
        "y": [22.2, 44.4, 66.6, 88.8, 108.08, 22.2, 44.4, 66.6, 88.8, 108.08, 22.2, 44.4, 66.6, 88.8, 108.08],
        "z": [33.3, 66.6, 99.9, 133.3, 166.6, 33.3, 66.6, 99.9, 133.3, 166.6, 33.3, 66.6, 99.9, 133.3, 166.6]
    }
}
data_2 = {
    "index_of_point": 1,
    "interpolation": 2,
    "velocity": 10,
    "acceleration": 5,
    "coordinates": {
        "x": 1.0,
        "y": 2.0,
        "z": 3.0
    }
}

data_3 = {
    "index_of_point": 2,
    "interpolation": 3,
    "velocity": 11,
    "acceleration": 6,
    "coordinates": {
        "x": 2.0,
        "y": 3.0,
        "z": 4.0
    }
}

data_4 = {
    "index_of_point": 3,
    "interpolation": 4,
    "velocity": 12,
    "acceleration": 7,
    "coordinates": {
        "x": 3.0,
        "y": 4.0,
        "z": 5.0
    }
}

if __name__ == "__main__":
    serialcomm = serial.Serial(port="COM3", baudrate=9600, timeout=1)
    serialcomm.reset_output_buffer()

    time.sleep(5)

    data_out = json.dumps(data)
    data_out_2 = json.dumps(data_2)
    data_out_3 = json.dumps(data_3)
    data_out_4 = json.dumps(data_4)
    # print(data_out)
    # print(data_out_2)
    for i in range(1):
        try:
            serialcomm.open()
            print("Serial opened")
        except:
            print("Serial not opened")
        # time.sleep(1)


        if serialcomm.isOpen():
            print("Sending data...")
            serialcomm.write(data_out.encode('ascii'))
            print(data_out)
            serialcomm.flush()

            print("Data sent.")
            while serialcomm.inWaiting() == 0:
                pass # .decode('utf-8')
            incoming = serialcomm.readall().decode('ascii')
            print(incoming)
            serialcomm.reset_input_buffer()

        else:
            print("opening error")
        time.sleep(1)
