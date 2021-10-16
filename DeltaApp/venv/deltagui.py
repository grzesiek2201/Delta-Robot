import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
import matplotlib.pyplot as plt
import deltarobot

FONT = "Times New Roman"
TEXT_SIZE = 12

class DeltaGUI():

    def __init__(self):

        self.delta = deltarobot.DeltaRobot()###

        self.jobid = 0
        self.position = [0, 0, 0] # not used anywhere yet, use it by adding a certain amount while holding JOG?
        self.root = tk.Tk()
        self.root.title("Delta GUI")
        self.root.config(padx=50, pady=50, bg="white")


        #Labels
        self.x_position_label = tk.Label(self.root, text="X [mm]", font=(FONT, TEXT_SIZE-3), bg="White")
        self.x_position_label.grid(row=0, column=2)
        self.y_position_label = tk.Label(self.root, text="Y [mm]", font=(FONT, TEXT_SIZE-3), bg="White")
        self.y_position_label.grid(row=0, column=3)
        self.z_position_label = tk.Label(self.root, text="Z [mm]", font=(FONT, TEXT_SIZE-3), bg="White")
        self.z_position_label.grid(row=0, column=4)

        self.position_label = tk.Label(self.root, text="Position", font=(FONT, TEXT_SIZE), bg="White")
        self.position_label.grid(padx=10, row=1, column=1)

        self.speed_label = tk.Label(self.root, text="Speed", font=(FONT, TEXT_SIZE), bg="White")
        self.speed_label.grid(padx=10, pady=10, row=2, column=1)

        self.interpolation_label = tk.Label(self.root, text="Interpolation", font=(FONT, TEXT_SIZE), bg="White")
        self.interpolation_label.grid(padx=10, pady=10, row=3, column=1)

        self.jog_label = tk.Label(self.root, text="JOG", font=(FONT, TEXT_SIZE), bg="White")
        self.jog_label.grid(padx=10, pady=10, row=4, column=1, rowspan=3)


        #Entries
        self.x_position_entry = tk.Entry(self.root, width=10)
        self.x_position_entry.grid(row=1, column=2)
        self.x_position_entry.insert(0, "0")

        self.y_position_entry = tk.Entry(self.root, width=10)
        self.y_position_entry.grid(row=1, column=3)
        self.y_position_entry.insert(0, "0")

        self.z_position_entry = tk.Entry(self.root, width=10)
        self.z_position_entry.grid(row=1, column=4)
        self.z_position_entry.insert(0, "0")


        #Buttons
        #ADD COMMAND=
        self.x_plus_button = tk.Button(self.root, text="X+", width=7)
        self.x_plus_button.bind('<ButtonPress-1>', lambda event, axis="x", direction=0: self.move_JOG(axis, direction))
        self.x_plus_button.bind('<ButtonRelease-1>', lambda event: self.stop_JOG)
        self.x_plus_button.grid(pady=5, row=4, column=2)

        self.x_minus_button = tk.Button(self.root, text="X-", width=7)
        self.x_minus_button.grid(pady=5, row=4, column=3)

        self.y_plus_button = tk.Button(self.root, text="Y+", width=7)
        self.y_plus_button.grid(pady=5, row=5, column=2)

        self.y_minus_button = tk.Button(self.root, text="Y-", width=7)
        self.y_minus_button.grid(pady=5, row=5, column=3)

        self.z_plus_button = tk.Button(self.root, text="Z+", width=7)
        self.z_plus_button.grid(pady=5, row=6, column=2)

        self.z_minus_button = tk.Button(self.root, text="Z-", width=7)
        self.z_minus_button.grid(pady=5, row=6, column=3)

        self.move_button = tk.Button(self.root, text="Move", font=(FONT, TEXT_SIZE), bg="White", command=self.updatePlot)
        self.move_button.grid(pady=5, row=2, column=3, rowspan=2, columnspan=2)

        # self.update_button = tk.Button(self.root, text="Update", font=(FONT, TEXT_SIZE), bg="White", command=self.updatePlot)
        # self.update_button.grid(pady=5, row=7, column=15)


        #Combobox
        self.speed_tuple = ('100%', '90%', '80%', '70%', '60%', '50%', '40%', '30%', '20%', '10%')
        self.speeds = tk.StringVar()
        self.speed_combobox = ttk.Combobox(self.root, textvariable=self.speeds, width=7)
        self.speed_combobox['values'] = self.speed_tuple
        self.speed_combobox['state'] = 'readonly'
        self.speed_combobox.current(9)
        self.speed_combobox.grid(row=2, column=2)

        self.interpolation_tuple = ('Joint', 'Linear', 'Circular')
        self.interpolations = tk.StringVar()
        self.interpolation_combobox = ttk.Combobox(self.root, textvariable=self.interpolations, width=7)
        self.interpolation_combobox['values'] = self.interpolation_tuple
        self.interpolation_combobox['state'] = 'readonly'
        self.interpolation_combobox.current(0)
        self.interpolation_combobox.grid(row=3, column=2)


        #Canvas
        # self.canvas = tk.Canvas(width=650, height=450, bg="white", highlightthickness=0)
        # self.plot_img =tk.PhotoImage(file="plot_example.png")
        # self.canvas.create_image(400, 200, image=self.plot_img)
        # self.canvas.grid(row=0, column=5, columnspan=10, rowspan=10)

        self.createPlot()
        tk.mainloop()

    def createPlot(self):
        # create figure
        self.fig = plt.figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="3d")

        # create canvas from backend of matplotlib so it can be displayed in gui
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=5, rowspan=10, columnspan=10)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def updatePlot(self):
        """Update the plot with supplied x y z coordinates"""
        point_x, point_y, point_z = self.getCoordinates()

        self.ax.clear()

        try:
            self.ax.plot(point_x[0], point_y[0], point_z[0], 'black')  # Link 1
            self.ax.plot(point_x[1], point_y[1], point_z[1], 'black')  # Link 2
            self.ax.plot(point_x[2], point_y[2], point_z[2], 'black')  # Link 3
            self.ax.plot(point_x[3], point_y[3], point_z[3])  # Effector
            self.ax.plot(point_x[4], point_y[4], point_z[4], 'ro')  # TCP
            self.ax.plot(self.delta.Bvx, self.delta.Bvy, self.delta.Bvz)
            self.ax.set_xlim(-800, 800)
            self.ax.set_ylim(-800, 800)
            self.ax.set_zlim(-2000, 500)
        except TypeError:
            pass

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def getCoordinates(self):
        """Sends x,y,z coordinates, speed % and interpolation type as a list through serial port"""
        x_coordinate = float(self.x_position_entry.get())
        y_coordinate = float(self.y_position_entry.get())
        z_coordinate = float(self.z_position_entry.get())

        point_x, point_y, point_z = self.delta.calculateIK(x_coordinate, y_coordinate, z_coordinate)

        speed_value = self.speed_combobox.get()
        #now convert to numerical value, preferably double
        interpolation_type = self.interpolation_combobox.get()
        # now convert to numerical value, preferably double
        # send data to arduino
        return point_x, point_y, point_z

    def JOG(self, axis, direction):
        """Jogs the selected axis in a selected direction with constant speed."""

        if axis == "x":
            if direction == 0:
                pass
            else:
                pass
        elif axis == "y":
            if direction == 0:
                pass
            else:
                pass
        elif axis == "z":
            if direction == 0:
                pass
            else:
                pass

    #### moving while X+ is pressed ####
    def move_JOG(self, axis, direction):
        print(f"Moving JOG axis {axis} in {direction} direction.")
        self.jobid = self.root.after(300, self.move_JOG, axis, direction)

    def stop_JOG(self):
        self.root.after_cancel(self.jobid)
        print("Stopped JOG")
