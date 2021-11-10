import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import datetime as dt

GRAPH_WIDTH = 100

ser = serial.Serial('COM4', 115200, timeout=1)

fig = plt.figure(figsize=(7, 7), dpi=100)
ax = fig.add_subplot(111)
ax.set_xlabel("Time")
ax.set_ylabel("Amplitude")
ax.grid()

x_values = []
y_values = []

ser.flushInput()

def getData(i):

    for loop_no in range(0, 3):
        while ser.inWaiting() == 0:
            pass

        data:float = float(ser.readline().decode('utf-8'))
        y_values.append(data)

        if len(y_values) >= GRAPH_WIDTH:
            del y_values[0]

        x_values.append(dt.datetime.now().strftime("%M:%S:%f"))
        if len(x_values) >= GRAPH_WIDTH:
            del x_values[0]

    ax.clear()
    ax.plot(x_values, y_values, 'g', label="Joint 1")
    ax.set_xlabel("Time")
    ax.set_ylabel("Angle [deg]")
    ax.grid()
    ax.xaxis.set_major_locator(plt.MaxNLocator(10))
    ax.set_ylim(-180, 180)

    plt.legend(loc="upper left")


ani = animation.FuncAnimation(fig, getData, interval=100)

plt.show()