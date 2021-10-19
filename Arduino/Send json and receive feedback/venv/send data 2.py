import json
import serial
import time


data = {
    "index_of_point": 1,
    "interpolation": 1,
    "velocity": 1,
    "acceleration": 1,
    "coordinates": [11.1, 22.2, 33.3]
}
data_2 = {
    "index_of_point": 2,
    "interpolation": 2,
    "velocity": 2,
    "acceleration": 2,
    "coordinates": [22.2, 33.3, 44.4]
}

data_3 = {
    "index_of_point": 2,
    "interpolation": 3,
    "velocity": 11,
    "acceleration": 6,
    "coordinates": [2.0, 3.0, 4.0]
}

data_4 = {
    "index_of_point": 3,
    "interpolation": 4,
    "velocity": 12,
    "acceleration": 7,
    "coordinates": [3.0, 4.0, 5.0]
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
    for i in range(4):
        try:
            serialcomm.open()
            print("Serial opened")
        except:
            print("Serial not opened")
        # time.sleep(1)


        if serialcomm.isOpen():
            print("Sending data...")
            if i == 0:
                serialcomm.write(data_out.encode('ascii'))
                print(data_out)
                serialcomm.flush()
            elif i == 1:
                serialcomm.write(data_out_2.encode('ascii'))
                print(data_out_2)
                serialcomm.flush()
            elif i == 2:
                serialcomm.write(data_out_3.encode('ascii'))
                print(data_out_3)
                serialcomm.flush()
            elif i == 3:
                serialcomm.write(data_out_4.encode('ascii'))
                print(data_out_4)
                serialcomm.flush()
            print("Data sent.")

            while serialcomm.inWaiting() == 0:
                pass # .decode('utf-8')
            incoming = serialcomm.readline().decode('ascii')
            print(incoming)
            serialcomm.reset_input_buffer()
            #serialcomm.close()

        else:
            print("opening error")
        #serialcomm.close()
        time.sleep(1)
