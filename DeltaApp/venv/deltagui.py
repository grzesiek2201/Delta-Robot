import tkinter as tk
from json import JSONDecodeError
from tkinter import messagebox
from tkinter import ttk
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
import matplotlib.pyplot as plt
import deltarobot
from tkinter.filedialog import asksaveasfile
import json
import ntpath
from collisionException import *
import serial
import numpy as np
import datetime as dt
import serial.tools.list_ports
import time

FONT = "Times New Roman"
TEXT_SIZE = 12
BAUDRATE = 115200
START_BYTE = '<0>'
PROGRAM_BYTE = '<1>'
DELAY_BYTE = '<3>'
WAIT_BYTE = '<4>'
SET_BYTE = '<5>'
HOME_BYTE = '<6>'
TEACH_IN_BYTE = '<7>'
ENABLE_BYTE = '<8>'


class DeltaGUI:

    def __init__(self):

        self.delta = deltarobot.DeltaRobot()
        self.current_file_name = None
        # self.jobid = None
        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        self.velocity = 0
        self.acceleration = 0
        self.interpolation = 0
        self.root = tk.Tk()
        self.root.geometry('1100x700')
        self.root.title("Delta GUI")
        self.root.config(padx=50, pady=20, bg="white")
        self.root.iconbitmap("delta icon.ico")
        self.program_popup_opened = False
        self.plots_opened = False
        self.plot_width = 100
        self.plots_id = 0
        self.position_units = tk.IntVar()
        self.online = tk.BooleanVar()
        self.ser = None
        self.serial_connected = False
        self.arduino_available = False

        self.x_values = []
        self.y_values = [[], [], []]
        self.plots_animation = None

        self.point_data = {
            "index_of_point": [],
            "interpolation": [],
            "velocity": [],
            "acceleration": [],
            "coordinates": {
                "x": [],
                "y": [],
                "z": []
            }
        }
        self.func_data = {
            "func_type": [],
            "pt_no": [],
            "value": [],
            "pin_no": []
        }

        # Menus
        self.menubar = tk.Menu(self.root)
        self.root.config(menu=self.menubar)
        self.menubar.add_command(label='Exit', command=self.exit)
        self.menubar.add_command(label='Program', command=self.programCreator)
        self.plot_menu = tk.Menu(self.menubar, tearoff=0)
        self.menubar.add_cascade(label='Plots', menu=self.plot_menu)
        self.plot_menu.add_command(label='Joints', command=lambda: self.create2DPlot(angular=True))
        self.plot_menu.add_command(label='XYZ', command=lambda: self.create2DPlot(angular=False))
        self.menubar.add_command(label="Available COMs", command=showAvailableComs)

        self.serialPortFrame(self.root, row=0, column=1, rowspan=1, columnspan=3, sticky='s')
        self.manualControlFrame(self.root, row=1, column=1, rowspan=1, columnspan=3, sticky='w')
        self.jogFrame(self.root, row=2, column=1, columnspan=3, sticky='w')
        self.statusFrame(self.root, row=8, column=1, columnspan=3, sticky='w')
        self.robotFrame(self.root, row=0, column=4, rowspan=15)

        # Buttons
        self.start_button = tk.Button(self.root, text="Start", font=(FONT, TEXT_SIZE), bg="White",
                                      command=self.startCommand)
        self.start_button.grid(row=4, column=1)

        self.enable_button = tk.Button(self.root, text="Enable motors", font=(FONT, TEXT_SIZE), bg="White",
                                       command=self.enableCommand)
        self.enable_button.grid(row=5, column=1)

        self.disable_button = tk.Button(self.root, text="Disable motors", font=(FONT, TEXT_SIZE), bg="White",
                                        command=self.disableCommand)
        self.disable_button.grid(row=6, column=1)

        self.home_button = tk.Button(self.root, text="Home", font=(FONT, TEXT_SIZE), bg="White")
        self.home_button.grid(row=7, column=1)

        self.readEncoders()
        self.onlineSimulation()

        tk.mainloop()

    def exit(self):
        self.root.quit()
        self.root.destroy()

    def connect(self):
        """ Connects to the specified COM port """
        port = self.serial_port_entry.get()
        try:
            if self.ser == None:
                self.ser = serial.Serial(port, BAUDRATE, timeout=1)
                self.port_status_label.config(text=f"Connected to {port}")
                self.serial_connect_button.config(text="Disconnect")
                self.serial_connected = True
            else:
                if self.ser.isOpen():
                    self.ser.close()
                    self.port_status_label.config(text="Disconnected")
                    self.serial_connect_button.config(text="Connect")
                    self.serial_connected = False
                else:
                    self.ser = serial.Serial(port, BAUDRATE, timeout=1)
                    self.port_status_label.config(text=f"Connected to {port}")
                    self.serial_connect_button.config(text="Disconnect")
                    self.serial_connected = True
        except serial.SerialException as e:
            message = str(e).partition(':')[0]
            self.port_status_label.config(text=message)
            print(message)

    def onlineSimulation(self):
        """ If the robot is online, simulate it's position based on the encoders' readings """
        if self.serial_connected and self.online.get() == 1:
            try:
                data = (self.angles[0], self.angles[1], self.angles[2])
            except ValueError as e:
                print(e)
            else:
                self.update3DPlot(manual=True, online=True, angles=data)

        self.root.after(50, self.onlineSimulation)

    def readEncoders(self):
        """ Reads position from encoders in a continuous loop, thus the position is always up to date """
        if self.serial_connected:
            print("read encoders")
            if self.ser.in_waiting != 0:
                data = self.ser.readline().decode('utf-8')
                print(f"{data = }")
                ## read all the angles, not just one
                try:
                    data = json.loads(data)
                except ValueError as e:
                    print(e)
                else:
                    self.angles[0] = data["deg"][0]
                    self.angles[1] = 0.0
                    self.angles[2] = data["deg"][1]
                    print(f"{self.angles = }")
                if self.angles[0] == -1111:
                    self.arduino_available = True
                elif self.angles[0] == -2222:
                    self.arduino_available = False
            self.ser.reset_input_buffer()
        self.root.after(50, self.readEncoders)

    def changeUnits(self):

        if self.position_units.get() == 0:
            self.x_position_label.config(text="X [mm]")
            self.y_position_label.config(text="Y [mm]")
            self.z_position_label.config(text="Z [mm]")
        elif self.position_units.get() == 1:
            self.x_position_label.config(text="φ1 [deg]")
            self.y_position_label.config(text="φ2 [deg]")
            self.z_position_label.config(text="φ3 [deg]")

    def startCommand(self):
        if self.serial_connected:
            message = START_BYTE + '{"start": true}' + '>'
            self.ser.write(message.encode('utf-8'))

            incoming = self.ser.readall().decode('utf-8')
            print(incoming)

        else:
            tk.messagebox.showwarning(title="Not connected", message="Not connected to device.")

    def enableCommand(self):
        if self.serial_connected:
            message = ENABLE_BYTE + '{"enable": true}' + '>'
            self.ser.write(message.encode('utf-8'))

            incoming = self.ser.readall().decode('utf-8')
            print(incoming)

        else:
            tk.messagebox.showwarning(title="Not connected", message="Not connected to device.")

    def disableCommand(self):
        if self.serial_connected:
            message = ENABLE_BYTE + '{"enable": false}' + '>'
            self.ser.write(message.encode('utf-8'))

            incoming = self.ser.readall().decode('utf-8')
            print(incoming)

        else:
            tk.messagebox.showwarning(title="Not connected", message="Not connected to device.")

    def teachInCommand(self):
        if self.serial_connected:
            converted_point = self.delta.calculateFPK(self.angles[0], self.angles[1], self.angles[2])
            converted_point = [round(point, 2) for point in converted_point]
            self.addPointToList(supply_values=True, point=converted_point)

        else:
            tk.messagebox.showwarning(title="Not connected", message="Not connected to device.")

    def serialPortFrame(self, master, row=0, column=0, rowspan=1, columnspan=1, sticky=''):
        """ Frame on main screen responsible for Serial Port connection """
        # Create Serial port frame
        self.fr_ser = tk.Frame(master, bg="White")
        self.fr_ser.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan, sticky=sticky)

        # Labels - Serial port frame
        self.serial_port_label = tk.Label(self.fr_ser, text="Serial port", font=(FONT, TEXT_SIZE), bg="White")
        self.serial_port_label.grid(padx=10, pady=10, row=0, column=0, columnspan=2)
        self.port_status_label = tk.Label(self.fr_ser, text="Not connected", font=(FONT, TEXT_SIZE), bg="White",
                                          wraplength=250, width=20)
        self.port_status_label.grid(row=0, column=4, columnspan=10, rowspan=1, sticky='w')

        # Entries - Serial port frame
        self.serial_port_entry = tk.Entry(self.fr_ser, width=10, bg="White")
        self.serial_port_entry.insert(0, "COM4")
        self.serial_port_entry.grid(row=0, column=2)

        # Buttons - Serial port frame
        self.serial_connect_button = tk.Button(self.fr_ser, text="Connect", bg="White", width=10, command=self.connect)
        self.serial_connect_button.grid(padx=5, row=0, column=3)

    def manualControlFrame(self, master, row=0, column=0, rowspan=1, columnspan=1, sticky=''):
        """ Frame on main screen responsible for manual control """
        # Create manual control frame
        self.fr_man_ctrl = tk.Frame(master, bg="White")
        self.fr_man_ctrl.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan, sticky=sticky)

        # Label - manual control frame
        self.x_position_label = tk.Label(self.fr_man_ctrl, text="X [mm]", font=(FONT, TEXT_SIZE - 3), bg="White")
        self.x_position_label.grid(row=2, column=2)

        self.y_position_label = tk.Label(self.fr_man_ctrl, text="Y [mm]", font=(FONT, TEXT_SIZE - 3), bg="White")
        self.y_position_label.grid(row=2, column=3)

        self.z_position_label = tk.Label(self.fr_man_ctrl, text="Z [mm]", font=(FONT, TEXT_SIZE - 3), bg="White")
        self.z_position_label.grid(row=2, column=4)

        self.position_label = tk.Label(self.fr_man_ctrl, text="Position", font=(FONT, TEXT_SIZE), bg="White")
        self.position_label.grid(padx=10, row=3, column=1)

        self.velocity_label = tk.Label(self.fr_man_ctrl, text="Velocity", font=(FONT, TEXT_SIZE), bg="White")
        self.velocity_label.grid(padx=10, pady=10, row=4, column=1)

        self.acceleration_label = tk.Label(self.fr_man_ctrl, text="Acceleration", font=(FONT, TEXT_SIZE), bg="White")
        self.acceleration_label.grid(padx=10, pady=10, row=5, column=1)

        self.interpolation_label = tk.Label(self.fr_man_ctrl, text="Interpolation", font=(FONT, TEXT_SIZE), bg="White")
        self.interpolation_label.grid(padx=10, pady=10, row=6, column=1)

        # Entries - manual control frame
        self.x_position_entry = tk.Entry(self.fr_man_ctrl, width=10)
        self.x_position_entry.grid(row=3, column=2)
        self.x_position_entry.insert(0, "0")
        self.x_position_entry.bind("<Return>", self.manualMove)

        self.y_position_entry = tk.Entry(self.fr_man_ctrl, width=10)
        self.y_position_entry.grid(row=3, column=3)
        self.y_position_entry.insert(0, "0")
        self.y_position_entry.bind("<Return>", self.manualMove)

        self.z_position_entry = tk.Entry(self.fr_man_ctrl, width=10)
        self.z_position_entry.grid(row=3, column=4)
        self.z_position_entry.insert(0, "0")
        self.z_position_entry.bind("<Return>", self.manualMove)

        # Buttons - manual control frame
        self.move_button = tk.Button(self.fr_man_ctrl, text="Move", font=(FONT, TEXT_SIZE - 1), bg="White")
        self.move_button.grid(pady=5, row=3, column=5, rowspan=1, columnspan=1)
        self.move_button.bind('<Button-1>', self.manualMove)

        # Radiobutton - manual control frame
        self.position_units_angles = tk.Radiobutton(self.fr_man_ctrl, text="deg", variable=self.position_units, value=1,
                                                    bg="White", highlightcolor="White", command=self.changeUnits)
        self.position_units_angles.grid(row=1, column=2)
        self.position_units_mm = tk.Radiobutton(self.fr_man_ctrl, text="mm", variable=self.position_units, value=0,
                                                bg="White",
                                                command=self.changeUnits)
        self.position_units_mm.grid(row=1, column=1)

        self.online_radio = tk.Radiobutton(self.fr_man_ctrl, text="Online", variable=self.online, value=True,
                                           bg="White",
                                           highlightcolor="White")
        self.online_radio.grid(row=1, column=5)
        self.offline_radio = tk.Radiobutton(self.fr_man_ctrl, text="Offline", variable=self.online, value=False,
                                            bg="White",
                                            highlightcolor="White")
        self.offline_radio.grid(row=1, column=4)

        # Combobox - manual control frame
        self.velocity_tuple = ('10%', '20%', '30%', '40%', '50%', '60%', '70%', '80%', '90%', '100%')
        self.velocities = tk.StringVar()
        self.velocity_combobox = ttk.Combobox(self.fr_man_ctrl, textvariable=self.velocities, width=7)
        self.velocity_combobox['values'] = self.velocity_tuple
        self.velocity_combobox['state'] = 'readonly'
        self.velocity_combobox.current(0)
        self.velocity_combobox.grid(row=4, column=2)

        self.acceleration_tuple = ('10%', '20%', '30%', '40%', '50%', '60%', '70%', '80%', '90%', '100%')
        self.accelerations = tk.StringVar()
        self.acceleration_combobox = ttk.Combobox(self.fr_man_ctrl, textvariable=self.accelerations, width=7)
        self.acceleration_combobox['values'] = self.acceleration_tuple
        self.acceleration_combobox['state'] = 'readonly'
        self.acceleration_combobox.current(0)
        self.acceleration_combobox.grid(row=5, column=2)

        self.interpolation_tuple = ('Joint', 'Linear', 'Circular')
        self.interpolations = tk.StringVar()
        self.interpolation_combobox = ttk.Combobox(self.fr_man_ctrl, textvariable=self.interpolations, width=7)
        self.interpolation_combobox['values'] = self.interpolation_tuple
        self.interpolation_combobox['state'] = 'readonly'
        self.interpolation_combobox.current(0)
        self.interpolation_combobox.grid(row=6, column=2)

    def jogFrame(self, master, row=0, column=0, rowspan=1, columnspan=1, sticky=''):
        """ Frame on main screen responsible for Jogging """
        # Create JOG Frame
        self.fr_jog = tk.Frame(master, bg="White")
        self.fr_jog.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan, sticky=sticky)

        # Labels - JOG Frame
        self.jog_label = tk.Label(self.fr_jog, text="JOG", font=(FONT, TEXT_SIZE), bg="White")
        self.jog_label.grid(padx=27, pady=10, row=0, column=1, rowspan=3)

        self.coord_current_label = tk.Label(self.fr_jog, text="x: 0.0 [mm] y: 0.0 [mm] z: 0.0 [mm]", font=(FONT, 9),
                                            bg="White", width=35)
        self.coord_current_label.grid(row=3, column=2, rowspan=1, columnspan=6)
        self.angle_current_label = tk.Label(self.fr_jog, font=(FONT, 9), bg="White", width=30)
        self.angle_current_label.grid(row=4, column=2, rowspan=1, columnspan=6)

        # Buttons - JOG Frame
        self.x_plus_button = tk.Button(self.fr_jog, text="X+", width=7)
        self.x_plus_button.bind('<ButtonPress-1>', lambda event, axis="x", direction=0: self.moveJog(event, axis=axis,
                                                                                                     direction=direction))
        # self.x_plus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.x_plus_button.grid(padx=7, pady=5, row=0, column=2)

        self.x_minus_button = tk.Button(self.fr_jog, text="X-", width=7)
        self.x_minus_button.bind('<ButtonPress-1>', lambda event, axis="x", direction=1: self.moveJog(event, axis=axis,
                                                                                                      direction=direction))
        # self.x_minus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.x_minus_button.grid(padx=7, pady=5, row=0, column=3)

        self.y_plus_button = tk.Button(self.fr_jog, text="Y+", width=7)
        self.y_plus_button.bind('<ButtonPress-1>', lambda event, axis="y", direction=0: self.moveJog(event, axis=axis,
                                                                                                     direction=direction))
        # self.y_plus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.y_plus_button.grid(padx=7, pady=5, row=1, column=2)

        self.y_minus_button = tk.Button(self.fr_jog, text="Y-", width=7)
        self.y_minus_button.bind('<ButtonPress-1>', lambda event, axis="y", direction=1: self.moveJog(event, axis=axis,
                                                                                                      direction=direction))
        # self.y_minus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.y_minus_button.grid(padx=7, pady=5, row=1, column=3)

        self.z_plus_button = tk.Button(self.fr_jog, text="Z+", width=7)
        self.z_plus_button.bind('<ButtonPress-1>', lambda event, axis="z", direction=0: self.moveJog(event, axis=axis,
                                                                                                     direction=direction))
        # self.z_plus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.z_plus_button.grid(padx=7, pady=5, row=2, column=2)

        self.z_minus_button = tk.Button(self.fr_jog, text="Z-", width=7)
        self.z_minus_button.bind('<ButtonPress-1>', lambda event, axis="z", direction=1: self.moveJog(event, axis=axis,
                                                                                                      direction=direction))
        # self.z_minus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.z_minus_button.grid(padx=7, pady=5, row=2, column=3)

    def robotFrame(self, master, row=0, column=0, rowspan=1, columnspan=1, sticky=''):
        """ Frame on main screen responsible for visualising the robot """
        # Create robot Frame
        self.fr_robot = tk.Frame(master, bg="White")
        self.fr_robot.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan, sticky=sticky)

        # Labels
        self.coord_warning_label = tk.Label(self.fr_robot, text="Coordinates out of range!", font=(FONT, TEXT_SIZE),
                                            bg="White")
        self.typeerror_entry_label = tk.Label(self.fr_robot, text="Input must be a number!", font=(FONT, TEXT_SIZE),
                                              bg="White")
        self.collision_label = tk.Label(self.fr_robot, text="Collision detected! Please change the point coordinates.",
                                        font=(FONT, TEXT_SIZE), bg="White")
        self.robot_frame_size_label = tk.Label(self.fr_robot, text="x", font=(FONT, TEXT_SIZE),
                                               bg="White", fg="White")
        self.robot_frame_size_label.grid(row=8, column=4)
        # Canvas
        self.create3DPlot(self.fr_robot)

    def statusFrame(self, master, row=0, column=0, rowspan=1, columnspan=1, sticky=''):

        self.fr_status = tk.Frame(master, bg="White")
        self.fr_status.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan, sticky=sticky)

        self.current_loaded_program_label = tk.Label(self.fr_status, text="Loaded program: ",
                                                     font=(FONT, TEXT_SIZE - 2),
                                                     bg="White")
        self.current_loaded_program_label.grid(row=0, column=0)
        self.fr_status_program_name = tk.Label(self.fr_status, text="", font=(FONT, TEXT_SIZE - 2), bg="White",
                                               wraplength=250)
        self.fr_status_program_name.grid(row=0, column=1, sticky='w')

    def programCreator(self):
        """ Creates a popup window that can be used to create, edit, open and save programmes """
        if self.program_popup_opened:
            self.program_creator.focus()
            return
        self.program_popup_opened = True
        self.current_file_name = None
        # Create window
        self.program_creator = tk.Toplevel(self.root)
        self.program_creator.title("Program creator")
        self.program_creator.config(padx=10, pady=10, bg="white")
        self.program_creator.iconbitmap("delta icon.ico")
        self.program_creator.protocol("WM_DELETE_WINDOW", self.onCloseSave)

        self.programPointFrame(self.program_creator)
        self.addPointFrame(self.program_creator)
        self.programCreatorMenu(self.program_creator)
        self.uploadProgramFrame(self.program_creator)
        self.programFunctionsFrame(self.program_creator)
        self.addFunctionFrame(self.program_creator)

    def create2DPlot(self, angular=True):
        """ Creates a popup window with joint angles and XYZ positions plots """
        if self.plots_opened:
            self.plots.focus()
            return
        self.plots_opened = True
        # Create window
        self.plots = tk.Toplevel(self.root)
        self.plots.title("Joints angles / XYZ positions plot")
        self.plots.config(padx=10, pady=10, bg="White")
        self.plots.iconbitmap("delta icon.ico")
        self.plots.protocol("WM_DELETE_WINDOW", self.onClose)

        # Create figure
        self.fig_plots = plt.figure(figsize=(7, 7), dpi=100)
        self.bx = self.fig_plots.add_subplot(111)
        self.bx.set_xlabel("Time")
        self.bx.set_ylabel("Amplitude")
        self.bx.grid()
        self.bx.clear()

        # Create canvas from backend of matplotlib so it can be displayed in gui
        self.canvas_plots = FigureCanvasTkAgg(self.fig_plots, master=self.plots)
        self.canvas_plots.draw()
        self.canvas_plots.get_tk_widget().grid(row=0, column=0, rowspan=1, columnspan=1, sticky='')
        self.toolbar_plots = NavigationToolbar2Tk(self.canvas_plots, self.plots, pack_toolbar=False)
        self.toolbar_plots.update()
        self.toolbar_plots.grid(row=1, column=0, rowspan=1, columnspan=1, sticky='')

        self.fig_plots.canvas.draw()
        self.fig_plots.canvas.flush_events()

        self.update2DPlot(angular)

    def update2DPlot(self, angular=True, xlabel="x", ylabel="y"):
        """ Updates the 2D plot. Arguments are axis to be uploaded, x and y label names, plot label name """

        if not self.serial_connected:
            self.plots_id = self.plots.after(50, self.update2DPlot, angular)
            return
        else:
            axis = self.bx
            try:
                data = [self.angles[0], self.angles[1], self.angles[2]]
                if not angular:
                    data = self.delta.calculateFPK(data[0], data[1], data[2])
                #print(f"{data = }")
            except ValueError as e:
                print(e)
                if len(self.y_values[0]) == 0:
                    self.y_values[0].append(0.0)
                    self.y_values[1].append(0.0)
                    self.y_values[2].append(0.0)
                else:
                    self.y_values[0].append(self.y_values[0][-1])
                    self.y_values[1].append(self.y_values[1][-1])
                    self.y_values[2].append(self.y_values[2][-1])
            else:
                self.y_values[0].append(data[0])
                self.y_values[1].append(data[1])
                self.y_values[2].append(data[2])

            # Delete first element if the plot is too long
            if len(self.y_values[0]) >= self.plot_width:
                del self.y_values[0][0]
                del self.y_values[1][0]
                del self.y_values[2][0]
            # Update x values - insert current time
            self.x_values.append(dt.datetime.now().strftime("%M:%S:%f"))
            # Delete first element if the plot is too long
            if len(self.x_values) >= self.plot_width:
                del self.x_values[0]
            # Plot values and configure plot
            axis.clear()
            axis.plot(self.x_values, self.y_values[0], 'g', label="Joint 1/ Axis X")
            axis.plot(self.x_values, self.y_values[1], 'r', label="Joint 2/ Axis Y")
            axis.plot(self.x_values, self.y_values[2], 'b', label="Joint 3/ Axis Z")
            axis.set_xlabel(xlabel)
            axis.set_ylabel(ylabel)
            axis.grid()
            axis.xaxis.set_major_locator(plt.MaxNLocator(5))

            axis.legend(loc="upper left")

            self.fig_plots.canvas.draw()
            self.fig_plots.canvas.flush_events()

            self.plots_id = self.plots.after(50, self.update2DPlot, angular)

    def programPointFrame(self, master):
        # Create frame - program points
        self.fr_program_point = tk.Frame(master, bg="White")
        self.fr_program_point.grid(row=0, column=0, rowspan=2, sticky='nsew')

        # Treeview fr program points
        style = ttk.Style()
        style.layout("Treeview", [('Treeview.treearea', {'sticky': 'nswe'})])  # remove the border
        self.program_tree = ttk.Treeview(self.fr_program_point,
                                         columns=('no.', 'X', 'Y', 'Z', 'v', 'a', 'Interpolation'), height=15)

        self.program_tree.column("#0", width=0, stretch='no')
        self.program_tree.column("no.", anchor='center', width=50, minwidth=50)
        self.program_tree.column("X", anchor='center', width=70, minwidth=70)
        self.program_tree.column("Y", anchor='center', width=70, minwidth=70)
        self.program_tree.column("Z", anchor='center', width=70, minwidth=70)
        self.program_tree.column("v", anchor='center', width=50, minwidth=50)
        self.program_tree.column("a", anchor='center', width=50, minwidth=50)
        self.program_tree.column("Interpolation", anchor='center', width=100, minwidth=100)

        self.program_tree.heading("#0", text="")
        self.program_tree.heading("no.", text="no.")
        self.program_tree.heading("X", text="X[mm]")
        self.program_tree.heading("Y", text="Y[mm]")
        self.program_tree.heading("Z", text="Z[mm]")
        self.program_tree.heading("v", text="v[%]")
        self.program_tree.heading("a", text="a[%]")
        self.program_tree.heading("Interpolation", text="Interpolation")

        self.program_tree.grid(row=0, column=0, columnspan=8)

        # Buttons fr program points
        self.fr_program_point_delete_selected_button = tk.Button(self.fr_program_point, text="Delete selected",
                                                                 font=(FONT, TEXT_SIZE),
                                                                 bg="White", command=lambda: self.deletePointFromList(
                self.program_tree))
        self.fr_program_point_delete_selected_button.grid(padx=10, row=3, column=0)
        self.fr_program_point_move_up_button = tk.Button(self.fr_program_point, text="⬆", width=5,
                                                         font=(FONT, TEXT_SIZE), bg="White",
                                                         command=self.moveUpSelected)
        self.fr_program_point_move_up_button.grid(padx=0, row=3, column=2)
        self.fr_program_point_move_up_button = tk.Button(self.fr_program_point, text="⬇", width=5,
                                                         font=(FONT, TEXT_SIZE), bg="White",
                                                         command=self.moveDownSelected)
        self.fr_program_point_move_up_button.grid(padx=0, row=3, column=3)

        # Labels fr program points
        self.fr_program_point_program_name_label = tk.Label(self.fr_program_point, text="", font=(FONT, 9), bg="White",
                                                            width=55)
        self.fr_program_point_program_name_label.grid(row=2, column=0, columnspan=5)

    def programFunctionsFrame(self, master):

        self.fr_func = tk.Frame(master, bg="White")
        self.fr_func.grid(pady=30, row=3, column=0, rowspan=2, sticky='nswe')

        # Treeview fr program points
        style = ttk.Style()
        style.layout("Treeview", [('Treeview.treearea', {'sticky': 'nswe'})])  # remove the border
        self.func_tree = ttk.Treeview(self.fr_func, columns=('function', 'point no.', 'value', 'pin'), height=10)

        self.func_tree.column("#0", width=0, stretch='no')
        self.func_tree.column("function", anchor='center', width=100, minwidth=70)
        self.func_tree.column("point no.", anchor='center', width=100, minwidth=50)
        self.func_tree.column("value", anchor='center', width=100, minwidth=70)
        self.func_tree.column("pin", anchor='center', width=100, minwidth=70)

        self.func_tree.heading("#0", text="")
        self.func_tree.heading("function", text="Function")
        self.func_tree.heading("point no.", text="Before point")
        self.func_tree.heading("value", text="Value")
        self.func_tree.heading("pin", text="Pin number")

        self.func_tree.grid(row=0, column=0, columnspan=8)

        # Buttons
        self.fr_func_point_delete_selected_button = tk.Button(self.fr_func, text="Delete selected",
                                                              font=(FONT, TEXT_SIZE),
                                                              bg="White",
                                                              command=lambda: self.deletePointFromList(self.func_tree))
        self.fr_func_point_delete_selected_button.grid(padx=10, row=1, column=0)

    def addPointFrame(self, master):
        # Create frame - add point
        self.fr_add_point = tk.Frame(master, bg="White")
        self.fr_add_point.grid(padx=20, row=0, column=1, rowspan=1, sticky='nsew')

        # Labels fr add point
        self.x_label = tk.Label(self.fr_add_point, text="X [mm]", bg="White")
        self.x_label.grid(padx=0, row=0, column=0)
        self.y_label = tk.Label(self.fr_add_point, text="Y [mm]", bg="White")
        self.y_label.grid(padx=0, row=0, column=1)
        self.z_label = tk.Label(self.fr_add_point, text="Z [mm]", bg="White")
        self.z_label.grid(padx=0, row=0, column=2)

        # Entries fr add point
        self.x_entry = tk.Entry(self.fr_add_point, width=10)
        self.x_entry.grid(padx=10, row=1, column=0)
        self.x_entry.insert(0, "0")
        self.x_entry.bind("<Return>", self.addPointToList)
        self.y_entry = tk.Entry(self.fr_add_point, width=10)
        self.y_entry.grid(padx=10, pady=10, row=1, column=1)
        self.y_entry.insert(0, "0")
        self.y_entry.bind("<Return>", self.addPointToList)
        self.z_entry = tk.Entry(self.fr_add_point, width=10)
        self.z_entry.grid(padx=10, row=1, column=2)
        self.z_entry.insert(0, "0")
        self.z_entry.bind("<Return>", self.addPointToList)

        # Buttons fr add points
        self.add_point_button = tk.Button(self.fr_add_point, text="Add point", font=(FONT, TEXT_SIZE), bg="White",
                                          width=12)
        self.add_point_button.grid(padx=10, row=5, column=0, columnspan=3)
        self.add_point_button.bind("<ButtonPress-1>", self.addPointToList)

        self.teach_in_button = tk.Button(self.fr_add_point, text="Teach-in", font=(FONT, TEXT_SIZE), bg="White",
                                         width=12,
                                         command=self.teachInCommand)
        self.teach_in_button.grid(padx=10, pady=10, row=6, column=0, columnspan=3)

        # Labels fr add point
        self.v_label = tk.Label(self.fr_add_point, text="Velocity", font=(FONT, TEXT_SIZE), bg="White")
        self.v_label.grid(row=2, column=0)
        self.a_label = tk.Label(self.fr_add_point, text="Acceleration", font=(FONT, TEXT_SIZE), bg="White")
        self.a_label.grid(row=3, column=0)
        self.i_label = tk.Label(self.fr_add_point, text="Interpolation", font=(FONT, TEXT_SIZE), bg="White")
        self.i_label.grid(row=4, column=0)

        # Combobox fr add point
        self.velocity_fr_add_combobox = ttk.Combobox(self.fr_add_point, textvariable=self.velocities, width=7)
        self.velocity_fr_add_combobox['values'] = self.velocity_tuple
        self.velocity_fr_add_combobox['state'] = 'readonly'
        self.velocity_fr_add_combobox.current(0)
        self.velocity_fr_add_combobox.grid(pady=10, row=2, column=1)

        self.acc_fr_add_combobox = ttk.Combobox(self.fr_add_point, textvariable=self.accelerations, width=7)
        self.acc_fr_add_combobox['values'] = self.acceleration_tuple
        self.acc_fr_add_combobox['state'] = 'readonly'
        self.acc_fr_add_combobox.current(0)
        self.acc_fr_add_combobox.grid(pady=10, row=3, column=1)

        self.interpolation_fr_add_combobox = ttk.Combobox(self.fr_add_point, textvariable=self.interpolations, width=7)
        self.interpolation_fr_add_combobox['values'] = self.interpolation_tuple
        self.interpolation_fr_add_combobox['state'] = 'readonly'
        self.interpolation_fr_add_combobox.current(0)
        self.interpolation_fr_add_combobox.grid(pady=20, row=4, column=1)

    def addFunctionFrame(self, master):

        self.fr_add_func = tk.Frame(master, bg="White")
        self.fr_add_func.grid(padx=20, pady=35, row=3, column=1, sticky='nsew')

        # Labels - add function frame
        self.fr_add_func_function_label = tk.Label(self.fr_add_func, text="Function", font=(FONT, TEXT_SIZE), width=8,
                                                   bg="White")
        self.fr_add_func_function_label.grid(pady=10, row=0, column=0)
        self.fr_add_func_point_no_label = tk.Label(self.fr_add_func, text="Point no.", font=(FONT, TEXT_SIZE), width=8,
                                                   bg="White")
        self.fr_add_func_point_no_label.grid(pady=10, row=1, column=0)
        self.fr_add_func_value_label = tk.Label(self.fr_add_func, text="Value", font=(FONT, TEXT_SIZE), width=8,
                                                bg="White")
        self.fr_add_func_value_label.grid(pady=10, row=2, column=0)
        self.fr_add_func_pin_label = tk.Label(self.fr_add_func, text="Pin number", font=(FONT, TEXT_SIZE), width=8,
                                              bg="White")
        self.fr_add_func_pin_label.grid(pady=10, row=3, column=0)

        # Buttons = add fucntion frame
        self.fr_add_func_add_button = tk.Button(self.fr_add_func, text="Add function", font=(FONT, TEXT_SIZE),
                                                bg="White", width=12, command=self.addFuncToList)
        self.fr_add_func_add_button.grid(pady=10, row=4, column=0, columnspan=2, sticky='se')

        # Entries - add function frame
        self.fr_add_func_point_no_entry = tk.Entry(self.fr_add_func, width=10)
        self.fr_add_func_point_no_entry.grid(padx=10, row=1, column=1)
        self.fr_add_func_point_no_entry.insert(0, "0")

        self.fr_add_func_value_entry = tk.Entry(self.fr_add_func, width=10)
        self.fr_add_func_value_entry.grid(padx=10, row=2, column=1)
        self.fr_add_func_value_entry.insert(0, "0")

        self.fr_add_func_pin_entry = tk.Entry(self.fr_add_func, width=10)
        self.fr_add_func_pin_entry.grid(padx=10, row=3, column=1)
        self.fr_add_func_pin_entry.insert(0, "")

        # Combobox - add function frame
        self.fr_add_func_functions_tuple = ('Wait time', 'Wait input', 'Set output')
        self.fr_add_func_functions = tk.StringVar()
        self.fr_add_func_function_combobox = ttk.Combobox(self.fr_add_func, textvariable=self.fr_add_func_functions,
                                                          width=10)
        self.fr_add_func_function_combobox['values'] = self.fr_add_func_functions_tuple
        self.fr_add_func_function_combobox['state'] = 'readonly'
        self.fr_add_func_function_combobox.current(0)
        self.fr_add_func_function_combobox.grid(padx=10, row=0, column=1)

        self.fr_add_func_inout_tuple = ('Input', 'Output')

    def uploadProgramFrame(self, master):
        # Create frame - upload program
        self.fr_upload_program = tk.Frame(master, bg="White")
        self.fr_upload_program.grid(padx=20, pady=0, row=1, column=1)

        # Buttons = fr upload program
        self.upload_button = tk.Button(self.fr_upload_program, text="Upload", font=(FONT, TEXT_SIZE), width=10,
                                       bg='White', command=self.uploadProgram)
        self.upload_button.grid(padx=10, row=0, column=0, columnspan=1)

    def programCreatorMenu(self, master):
        self.program_creator_menu = tk.Menu(master)
        master.config(menu=self.program_creator_menu)
        self.program_creator_menu.add_command(label="Open", command=self.openProgram)
        self.program_creator_menu.add_command(label="Save", command=self.saveProgram)
        self.program_creator_menu.add_command(label="Save as", command=self.saveProgramAs)

    def updateProgramPoints(self, master, data):
        """ updates the points in treeview """
        self.program_tree.delete(*self.program_tree.get_children())
        for index in range(len(data["index_of_point"])):
            self.program_tree.insert('', index='end', text='', values=(data["index_of_point"][index],
                                                                       data["coordinates"]["x"][index],
                                                                       data["coordinates"]["y"][index],
                                                                       data["coordinates"]["z"][index],
                                                                       data["velocity"][index],
                                                                       data["acceleration"][index],
                                                                       data["interpolation"][index]))

    def updateFunctions(self, master, data):
        """ updates the functions in treeview """
        self.func_tree.delete(*self.func_tree.get_children())
        for index in range(len(data["value"])):
            self.func_tree.insert('', index='end', text='', values=(data["func_type"][index],
                                                                    data["pt_no"][index],
                                                                    data["value"][index],
                                                                    data["pin_no"][index]))

    def addPointToList(self, event=None, supply_values=False, point=None):
        """ Add point with all its parameters to the treeview list """
        # Get respected parameters
        new_index_of_point = len(self.program_tree.get_children())
        vel = self.velocity_fr_add_combobox.get()
        acc = self.acc_fr_add_combobox.get()
        interpolation = self.interpolation_fr_add_combobox.get()
        if supply_values:
            x, y, z = point
        else:
            x = self.x_entry.get()
            y = self.y_entry.get()
            z = self.z_entry.get()
        # Insert parameters into the treeview
        self.program_tree.insert('', index='end', text='', values=(new_index_of_point, x, y, z, vel, acc, interpolation))
        # Clear the entries, enter 0
        self.x_entry.delete(0, "end")
        self.x_entry.insert(0, "0")
        self.y_entry.delete(0, "end")
        self.y_entry.insert(0, "0")
        self.z_entry.delete(0, "end")
        self.z_entry.insert(0, "0")

    def addFuncToList(self):

        if self.fr_add_func_function_combobox.get() == "Wait time":
            self.fr_add_func_pin_entry.delete(0, 'end')
            self.fr_add_func_pin_entry.insert(0, '0')

        self.func_tree.insert('', index='end', text='', values=(self.fr_add_func_function_combobox.get(),
                                                                self.fr_add_func_point_no_entry.get(),
                                                                self.fr_add_func_value_entry.get(),
                                                                self.fr_add_func_pin_entry.get()))

    def deletePointFromList(self, master):  # only in GUI, no effect on points yet
        for item in master.selection():
            master.delete(item)

        new_index = 0
        for item in self.program_tree.get_children():
            self.program_tree.set(item, column=0, value=new_index)
            new_index += 1

    def moveUpSelected(self):
        for higher_item in self.program_tree.selection():
            # Get higher item's index (lower_index is the one on top)
            higher_index = self.program_tree.index(higher_item)
            # Get lower item
            lower_item = self.program_tree.prev(higher_item)
            # Get lower item's index
            lower_index = self.program_tree.index(lower_item)

            # Set new indexes
            self.program_tree.set(higher_item, column=0, value=lower_index)
            self.program_tree.set(lower_item, column=0, value=higher_index)

            self.program_tree.move(higher_item, self.program_tree.parent(higher_item), lower_index)

    def moveDownSelected(self):  # only in GUI, no effect on points yet

        for lower_item in self.program_tree.selection():
            # Get lower item's index (lower_index is the one on top)
            lower_index = self.program_tree.index(lower_item)
            # Get higher item
            higher_item = self.program_tree.next(lower_item)
            # Get higher item's index
            higher_index = self.program_tree.index(higher_item)
            if higher_index != 0:
                # Set new indexes
                self.program_tree.set(lower_item, column=0, value=higher_index)
                self.program_tree.set(higher_item, column=0, value=lower_index)

                self.program_tree.move(lower_item, self.program_tree.parent(lower_item), higher_index)

    def onCloseSave(self):
        if tk.messagebox.askyesno(title="Exit?", message="Do you want to save the program?"):
            self.saveProgram()
        self.program_creator.destroy()
        self.program_popup_opened = False

    def onClose(self):
        self.plots.destroy()
        self.plots_opened = False
        self.root.after_cancel(self.plots_id)

    def pointsToJson(self):

        # Clear the structure before assigning new values
        self.point_data = {
            "index_of_point": [],
            "interpolation": [],
            "velocity": [],
            "acceleration": [],
            "coordinates": {
                "x": [],
                "y": [],
                "z": []
            }
        }

        # Load values to the point_data structure
        for line in self.program_tree.get_children():
            values = self.program_tree.item(line)['values']
            self.point_data["index_of_point"].append(values[0])
            self.point_data["interpolation"].append(values[6])
            self.point_data["velocity"].append(values[4])
            self.point_data["acceleration"].append(values[5])
            self.point_data["coordinates"]["x"].append(values[1])
            self.point_data["coordinates"]["y"].append(values[2])
            self.point_data["coordinates"]["z"].append(values[3])

        # Convert to json file
        data = json.dumps(self.point_data)
        return data

    def funcsToJson(self):

        # Clear the structure before assigning new values
        self.func_data = {
            "func_type": [],
            "pt_no": [],
            "value": [],
            "pin_no": []
        }
        # Load values to the func_data structure
        for line in self.func_tree.get_children():
            values = self.func_tree.item(line)['values']
            self.func_data["func_type"].append(values[0])
            self.func_data["pt_no"].append(values[1])
            self.func_data["value"].append(values[2])
            self.func_data["pin_no"].append(values[3])

        # Convert to json file
        data = json.dumps(self.func_data)
        print(f"{data = }")
        return data

    def convertParameters(self):
        """ Convert parameters from 50% -> 50, Joint -> 0, Linear -> 1, Circular -> 2 etc."""
        for index in range(len(self.point_data['index_of_point'])):
            if self.point_data['interpolation'][index] == "Joint":
                self.point_data['interpolation'][index] = 0
            elif self.point_data['interpolation'][index] == "Linear":
                self.point_data['interpolation'][index] = 1
            elif self.point_data['interpolation'][index] == "Circular":
                self.point_data['interpolation'][index] = 2
            self.point_data['velocity'][index] = str(self.point_data['velocity'][index]).partition('%')[0]
            self.point_data['acceleration'][index] = str(self.point_data['acceleration'][index]).partition('%')[0]

    def convertFuncs(self):
        """ Convert parameters from Wait time -> 3, Wait input -> 4 etc."""

        for index in range(len(self.func_data['func_type'])):
            if self.func_data['func_type'][index] == "Wait time":
                self.func_data['func_type'][index] = DELAY_BYTE
            elif self.func_data['func_type'][index] == "Wait input":
                self.func_data['func_type'][index] = WAIT_BYTE
            elif self.func_data['func_type'][index] == "Set output":
                self.func_data['func_type'][index] = SET_BYTE

    def uploadProgram(self):

        # If file is not saved, inform the user
        if not self.current_file_name:
            tk.messagebox.showinfo(title="Cannot upload", message="Save file before upload.")
            self.program_creator.focus()
        elif not self.serial_connected:
            tk.messagebox.showinfo(title="Cannot upload", message="Connect to device before uploading.")
            self.program_creator.focus()
        else:
            self.uploadProgramPoints()
            #self.uploadProgramFunctions()

    def uploadProgramPoints(self):

        # Update point_data structure
        self.pointsToJson()
        # Convert to a 'only-numbers' format
        self.convertParameters()
        # Create temporary variables
        temp_data_point = {"n": 0,
                           "i": 0,
                           "v": 0,
                           "a": 0,
                           "c": [0, 0, 0]
                           }

        # Only start uploading when Arduino is available #####################################
        # while not self.arduino_available:              ############################### FIGURE IT OUT PLS
        #     time.sleep(0.1)                            ##### WAIT TILL YOU CAN SEND INFO
        #     self.readEncoders()

        # For each point represented by index_of_point rewrite the point's parameters into temporary variable
        # and check for collision or out of range errors
        for index in self.point_data['index_of_point']:
            temp_data_point['n'] = self.point_data['index_of_point'][index]
            temp_data_point['i'] = self.point_data['interpolation'][index]
            temp_data_point['v'] = self.point_data['velocity'][index]
            temp_data_point['a'] = self.point_data['acceleration'][index]
            temp_data_point['c'][0] = self.point_data['coordinates']['x'][index]
            temp_data_point['c'][1] = self.point_data['coordinates']['y'][index]
            temp_data_point['c'][2] = self.point_data['coordinates']['z'][index]
            try:
                self.delta.calculateIPK((temp_data_point['c'][0], temp_data_point['c'][1],
                                         temp_data_point['c'][2]))
            except TypeError as e:
                tk.messagebox.showwarning(title="Out of range",
                                          message=f"Point {index} is out of range!\nPlease change coordinates.")
                self.program_creator.focus()
                print(e)
            except ValueError as e:
                tk.messagebox.showwarning(title="Wrong data",
                                          message=f"Point {index} has incorrect parameters, please correct them.")
                self.program_creator.focus()
                print(e)
            except CollisionException:
                tk.messagebox.showwarning(title="Collision detected",
                                          message=f"Moving to point {index} will result in a collision!")
                self.program_creator.focus()
                print("Collision detected! Please check the given coordinates.")

            else:
                # For each point represented by index_of_point rewrite the point's parameters into temporary variable
                # and then dump it into json format
                temp_send_point = json.dumps(temp_data_point)
                data_to_send = PROGRAM_BYTE + '<' + temp_send_point + '>'  # Add 0 to represent that the data that is being sent is point parameters
                print(f"{data_to_send = }")
                data_to_send = data_to_send.replace(" ", "")

                self.ser.write(data_to_send.encode('utf-8'))

                while self.ser.inWaiting() == 0:
                    pass  # .decode('utf-8')
                incoming = self.ser.readall().decode('utf-8')
                print(incoming)

    def uploadProgramFunctions(self):

        # Update func_data structure
        self.funcsToJson()
        # Convert to a 'only-numbers' format
        self.convertFuncs()

        temp_data_func = {
            "pt_no": [0],
            "value": [0],
            "pin_no": [0]
        }

        for index in range(0, len(self.func_data['value'])):
            func_type = self.func_data['func_type'][index]
            temp_data_func['pt_no'] = self.func_data['pt_no'][index]
            temp_data_func['value'] = self.func_data['value'][index]
            temp_data_func['pin_no'] = self.func_data['pin_no'][index]

            temp_send_func = json.dumps(temp_data_func)

            data_to_send = str(
                func_type) + temp_send_func  # Add func_type number to represent that the data that is being
            # sent is a certain function
            print(f"{data_to_send = }")

            self.ser.write(data_to_send.encode('utf-8'))

            while self.ser.inWaiting() == 0:
                pass  # .decode('utf-8')
            incoming = self.ser.readall().decode('utf-8')
            print(incoming)

    def openProgram(self):

        files = [('Text Document', '*.txt')]
        self.current_file_name = tk.filedialog.askopenfilename(filetypes=files)
        try:
            with open(self.current_file_name, 'r') as data_file:
                point_data = data_file.readline()
                point_data = json.loads(point_data)
                func_data = data_file.readline()
                func_data = json.loads(func_data)
                print(f"{point_data = }")
                print(f"{func_data = }")
                for key in point_data:
                    self.point_data[key] = point_data[key]
                for key in func_data:
                    self.func_data[key] = func_data[key]

        except FileNotFoundError as e:
            print(e)
        except JSONDecodeError:
            tk.messagebox.showinfo(title="Error reading file",
                                   message="Error while reading the file.\nCheck the file format (json formatting).")
        else:
            self.updateProgramPoints(self.program_tree, self.point_data)
            self.updateFunctions(self.func_tree, self.func_data)
            program_name = ntpath.basename(self.current_file_name)
            self.fr_program_point_program_name_label.config(text=program_name)
            self.fr_status_program_name.config(text=program_name)

        self.program_creator.focus()

    def saveProgram(self):

        # Check for open file
        if self.current_file_name:
            with open(self.current_file_name, 'w') as data_file:
                data_file.write(f"{self.pointsToJson()}\n{self.funcsToJson()}")
        else:
            self.saveProgramAs()

    def saveProgramAs(self):

        # Dialog window to save the file
        files = [('Text Document', '*.txt')]
        self.current_file_name = tk.filedialog.asksaveasfilename(filetypes=files, defaultextension=files)
        # Save program to file
        try:
            with open(self.current_file_name, 'w') as data_file:
                data_file.write(f"{self.pointsToJson()}\n{self.funcsToJson()}")
        except FileNotFoundError:
            pass
        self.program_creator.focus()

    def manualMove(self, event):
        """ If coordinates are in range and not colliding, upload the plot and then load parameters and send them to arduino"""
        #print(f"{self.online.get() = }, {self.serial_connected = }")
        if not self.serial_connected and self.online.get():
            tk.messagebox.showinfo(title="Cannot upload", message="Connect to device before uploading.")

        elif self.serial_connected and not self.online.get():
            tk.messagebox.showinfo(title="Warning",
                                   message="Offline mode is selected but there is connection with the device.")
        else:
            # while self.serial_connected and self.online.get() and not self.arduino_available:
            #     pass

            temp_data_point = {"n": 0,
                               "i": 0,
                               "v": 0,
                               "a": 0,
                               "c": [0, 0, 0]
                               }

            if self.update3DPlot(event, manual=True, onlycheck=self.online.get()) and self.online.get():
                temp_data_point['n'] = 0
                temp_data_point['i'] = self.interpolation
                temp_data_point['v'] = int(int(self.velocity) / 10)
                temp_data_point['a'] = int(int(self.acceleration) / 10)
                temp_data_point['c'][0] = round(float(self.x_position_entry.get()), 2)
                temp_data_point['c'][1] = round(float(self.y_position_entry.get()), 2)
                temp_data_point['c'][2] = round(float(self.z_position_entry.get()), 2)

                temp_send_point = json.dumps(temp_data_point)
                data_to_send = PROGRAM_BYTE + '<' + temp_send_point + '>' # Add PROGRAM_BYTE to represent that the data that is being sent is point parameters

                print(f"{data_to_send = }")
                data_to_send = data_to_send.replace(" ", "")
                print(f"{data_to_send = }")
                self.ser.write(data_to_send.encode('utf-8'))

                while self.ser.inWaiting() == 0:
                    pass  # .decode('utf-8')
                incoming = self.ser.readall().decode('utf-8')
                print(incoming)

    def create3DPlot(self, master):
        # Create figure
        self.fig_robot = plt.figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig_robot.add_subplot(111, projection="3d")
        self.ax.set_xlim(-800, 800)
        self.ax.set_ylim(-800, 800)
        self.ax.set_zlim(-2000, 500)

        # Create canvas from backend of matplotlib so it can be displayed in gui
        self.canvas_robot = FigureCanvasTkAgg(self.fig_robot, master=master)
        self.canvas_robot.draw()
        self.canvas_robot.get_tk_widget().grid(row=0, column=0, rowspan=1, columnspan=6, sticky='n')
        self.fig_robot.canvas.draw()
        self.fig_robot.canvas.flush_events()

    def update3DPlot(self, event=None, manual=False, xyz=(0, 0, 0), online=False, angles=(0, 0, 0), onlycheck=False):
        """Update the plot with supplied x y z coordinates"""
        try:
            # If jogging or auto, manual = False, if moving based on entered position - manual = True
            if manual:
                position = self.readCoordinates(online, angles)
            else:
                position = xyz
            self.delta.calculateIPK(position)  # catch an out of range error
            temp = self.delta.vert_coords.coordinates_x[1]  # catch an error when creating model that is out of range
        except TypeError as e:
            self.typeerror_entry_label.grid_forget()
            self.collision_label.grid_forget()
            self.coord_warning_label.grid(row=8, column=3, rowspan=1)  # show 'out of range' label
            print(e)
        except ValueError as e:
            print("Write only number in the coordinates entries.")
            self.typeerror_entry_label.grid(row=8, column=3, rowspan=1)
            print(e)
        except CollisionException:
            print("Collision detected! Please check the given coordinates.")
            self.collision_label.grid(row=8, column=3, rowspan=1)
        else:
            if not onlycheck:
                self.updateCurrentPosition(position, update_param=manual)
            # Hide any remaining labels
            self.coord_warning_label.grid_forget()
            self.typeerror_entry_label.grid_forget()
            self.collision_label.grid_forget()
            # Display current position
            self.coord_current_label.config(
                text=f"x: {round(self.position[0], 2)} [mm] y: {round(self.position[1], 2)} [mm] z: {round(self.position[2], 2)} [mm]")
            self.coord_current_label.grid(row=3, column=2, rowspan=1, columnspan=6)
            self.angle_current_label.config(
                text=f"φ1: {round(self.angles[0] * 180 / np.pi, 2)}° φ2: {round(self.angles[1] * 180 / np.pi, 2)}° "
                     f"φ3: {round(self.angles[2] * 180 / np.pi, 2)}°")
            self.angle_current_label.grid(row=4, column=2, rowspan=1, columnspan=6)

            if not onlycheck:
                self.ax.clear()
                self.plotObjects()

                self.fig_robot.canvas.draw()
                self.fig_robot.canvas.flush_events()

            return 1

    def plotObjects(self):

        for i in range(6):
            self.ax.plot(self.delta.vert_coords.coordinates_x[i], self.delta.vert_coords.coordinates_y[i],
                         self.delta.vert_coords.coordinates_z[i], 'black')  # links
        self.ax.plot(self.delta.vert_coords.coordinates_x[6], self.delta.vert_coords.coordinates_y[6],
                     self.delta.vert_coords.coordinates_z[6])  # Effector
        self.ax.plot(self.delta.vert_coords.coordinates_x[7], self.delta.vert_coords.coordinates_y[7],
                     self.delta.vert_coords.coordinates_z[7], 'ro')  # TCP
        self.ax.plot(self.delta.Bvx, self.delta.Bvy, self.delta.Bvz)  # Base
        # Set axes limits
        self.ax.set_xlim(-300, 300)
        self.ax.set_ylim(-300, 300)
        self.ax.set_zlim(-605, 100)

        # Plot legs
        self.addRightPrismToPlot([[0.866 * self.delta.ub - 20, 0.866 * self.delta.ub + 20, 0.866 * self.delta.ub + 20,
                                   0.866 * self.delta.ub - 20],
                                  [-self.delta.wb - 20, -self.delta.wb - 20, -self.delta.wb + 20, -self.delta.wb + 20]],
                                 z=-605, height=605)
        self.addRightPrismToPlot([[-0.866 * self.delta.ub - 20, -0.866 * self.delta.ub + 20,
                                   -0.866 * self.delta.ub + 20, -0.866 * self.delta.ub - 20],
                                  [-self.delta.wb - 20, -self.delta.wb - 20, -self.delta.wb + 20, -self.delta.wb + 20]],
                                 z=-605, height=605)
        self.addRightPrismToPlot([[-20, 20, 20, -20],
                                  [self.delta.ub - 20, self.delta.ub - 20, self.delta.ub + 20, self.delta.ub + 20]],
                                 z=-605, height=605)

    def addElementsToPlot(self, *elements, color="black"):
        """ Add elements to plot described by x,y,z coordinates and connect them by lines """
        for element in elements:
            self.ax.plot(element[0], element[1], element[2], color=color)

    def addRightPrismToPlot(self, *args, z, height):
        """ Add vertices in order so that first one is next to the second one an so on. vertices = [[x1, x2, x3], [y1, y2, 3]] """
        bottom_polygon = [[], [], []]

        for xy_coord in args:
            bottom_polygon[0] = xy_coord[0]
            bottom_polygon[0].append(bottom_polygon[0][0])  # add the last point to close the polygon
            bottom_polygon[1] = xy_coord[1]
            bottom_polygon[1].append(bottom_polygon[1][0])  # add the last point to close the polygon
            bottom_polygon[2] = [z for _ in range(len(xy_coord[0]))]

            top_polygon = bottom_polygon.copy()
            top_polygon[2] = [z + height for length in range(len(xy_coord[0]))]

            self.addElementsToPlot(bottom_polygon, top_polygon)

            # Draw edges
            for i in range(len(xy_coord[0])):
                edge = [[bottom_polygon[0][i], bottom_polygon[0][i]], [bottom_polygon[1][i], bottom_polygon[1][i]],
                        [z, z + height]]
                self.addElementsToPlot(edge)

    def readCoordinates(self, online=False, angles=(0, 0, 0)):
        """ Returns a tuple of lists of x, y and z coordinates to draw"""

        if online:
            P = self.delta.calculateFPK(angles[0], angles[1], angles[2])
            x_coordinate = P[0]
            y_coordinate = P[1]
            z_coordinate = P[2]
            #print(f"{x_coordinate = }, {y_coordinate = }, {z_coordinate = }")

        else:
            x_coordinate = float(self.x_position_entry.get())
            y_coordinate = float(self.y_position_entry.get())
            z_coordinate = float(self.z_position_entry.get())

            # If units are degrees, use FPK to get x y z
            if self.position_units.get() == 1:
                P = self.delta.calculateFPK(x_coordinate, y_coordinate, z_coordinate)
                x_coordinate = P[0]
                y_coordinate = P[1]
                z_coordinate = P[2]

        return x_coordinate, y_coordinate, z_coordinate

        # now convert to numerical value
        # send data to arduino

    def updateCurrentPosition(self, xyz, update_param=False):
        """ Updates current position and parameter values """
        self.position[0] = xyz[0]
        self.position[1] = xyz[1]
        self.position[2] = xyz[2]
        #print(f"{round(self.position[0] , 2) =}, {round(self.position[1] , 2) =}, {round(self.position[2] , 2) =}")

        if update_param:
            self.velocity = str(self.velocity_combobox.get()).strip('%')
            self.acceleration = str(self.acceleration_combobox.get()).strip('%')
            self.interpolation = self.interpolation_combobox.get()
            if self.interpolation == "Joint":
                self.interpolation = 0
            elif self.interpolation == "Linear":
                self.interpolation = 1
            elif self.interpolation == "Circular":
                self.interpolation = 2

    def Jog(self, axis, direction):
        """Jogs the selected axis in a selected direction with constant speed."""
        if axis == "x":
            if direction == 0:
                self.update3DPlot(event=None, manual=False,
                                  xyz=(self.position[0] + 5, self.position[1], self.position[2]))
            elif direction == 1:
                self.update3DPlot(event=None, manual=False,
                                  xyz=(self.position[0] - 5, self.position[1], self.position[2]))
        elif axis == "y":
            if direction == 0:
                self.update3DPlot(event=None, manual=False,
                                  xyz=(self.position[0], self.position[1] + 5, self.position[2]))
            elif direction == 1:
                self.update3DPlot(event=None, manual=False,
                                  xyz=(self.position[0], self.position[1] - 5, self.position[2]))
        elif axis == "z":
            if direction == 0:
                self.update3DPlot(event=None, manual=False,
                                  xyz=(self.position[0], self.position[1], self.position[2] + 5))
            elif direction == 1:
                self.update3DPlot(event=None, manual=False,
                                  xyz=(self.position[0], self.position[1], self.position[2] - 5))

    #### move once ####
    def moveJog(self, event, axis, direction):
        print(f"Moving JOG axis {axis} in {direction} direction.")
        self.Jog(axis=axis, direction=direction)
        #print(f"{round(self.position[0] , 2) =}, {round(self.position[1] , 2) =}, {round(self.position[2] , 2) =}")
        # self.jobid = self.root.after(100, self.moveJog, event, axis, direction)

    #### not used currently ####
    def stopJog(self, event):
        self.root.after_cancel(self.jobid)
        print("Stopped JOG")


def showAvailableComs():
    """ Shows a pop-up window with available COM ports"""
    ports = serial.tools.list_ports.comports()
    available_ports = ""
    for port, desc, hwid in sorted(ports):
        available_ports += str(port) + ': ' + str(desc) + '\n'
    tk.messagebox.showinfo(title="COM ports", message=available_ports)