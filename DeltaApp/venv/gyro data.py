import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import numpy as np
import time
import datetime as dt

GRAPH_WIDTH = 100

ser = serial.Serial('COM4', 115200, timeout=1)

acc_x_values = []
acc_y_values = []
acc_z_values = []

x_values = []

fig = plt.figure(figsize=(7, 7), dpi=100)
ax = fig.add_subplot(111)
ax.set_xlabel("Time")
ax.set_ylabel("Amplitude")
ax.grid()

ser.flushInput()

def getData(i):

    for loop_no in range(0, 5):

        data = ser.readline().decode('utf-8')
        try:
            deserialized_data = json.loads(data)
        except json.decoder.JSONDecodeError as e:
            print(e)
        else:
            acc_x_values.append(int(deserialized_data["acc"][0]))
            acc_y_values.append(int(deserialized_data["acc"][1]))
            acc_z_values.append(int(deserialized_data["acc"][2]))

            if len(acc_x_values) >= GRAPH_WIDTH:
                del acc_x_values[0]
                del acc_y_values[0]
                del acc_z_values[0]

            x_values.append(dt.datetime.now().strftime("%M:%S:%f"))
            if len(x_values) >= GRAPH_WIDTH:
                del x_values[0]

    ax.clear()
    ax.plot(x_values, acc_x_values, 'g', label="acc_x")
    ax.plot(x_values, acc_y_values, 'b', label="acc_y")
    ax.plot(x_values, acc_z_values, 'r', label="acc_z")
    ax.set_xlabel("Time")
    ax.set_ylabel("Amplitude")
    ax.grid()
    ax.xaxis.set_major_locator(plt.MaxNLocator(10))

    plt.legend(loc="upper left")


# ani = animation.FuncAnimation(fig, getData, interval=5)
#
# plt.show()

acc_data = [[],[],[]]
time_data = []
if __name__ == '__main__':
    start = time.time()
    end = time.time()
    while end - start < 5:
        data = ser.readline().decode('utf-8')
        try:
            acc = json.loads(data)
        except json.decoder.JSONDecodeError as e:
            print(e)
        else:
            acc_data[0].append(acc["acc"][0])
            acc_data[1].append(acc["acc"][1])
            acc_data[2].append(acc["acc"][2])
            time_data.append(dt.datetime.now().strftime("%S:%f"))

        end = time.time()

ax.plot(time_data, acc_data[0], 'g', label="acc_x")
ax.plot(time_data, acc_data[1], 'r', label="acc_y")
ax.plot(time_data, acc_data[2], 'b', label="acc_z")
ax.set_xlabel("Time [s:ns]")
ax.set_ylabel("Acceleration [9,81 m/s^2]")
ax.grid()
ax.xaxis.set_major_locator(plt.MaxNLocator(10))

plt.show()