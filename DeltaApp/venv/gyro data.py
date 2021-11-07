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
        while ser.inWaiting() == 0:
            pass

        data = ser.readline().decode('utf-8')

        deserialized_data = json.loads(data)

        acc_x_values.append(int(deserialized_data["acc_x"]))
        acc_y_values.append(int(deserialized_data["acc_y"]))
        acc_z_values.append(int(deserialized_data["acc_z"]))

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



ani = animation.FuncAnimation(fig, getData, interval=100)

plt.show()