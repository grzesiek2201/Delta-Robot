import tkinter as tk
from json import JSONDecodeError
from tkinter import messagebox
from tkinter import ttk
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
import matplotlib.pyplot as plt
import deltarobot
from tkinter.filedialog import asksaveasfile
import json
from ntpath import basename
from collisionException import *
import serial
import serial.tools.list_ports
import datetime as dt
import time
import pandas as pd


FONT = "Times New Roman"
TEXT_SIZE = 12
BAUDRATE = 115200
TIMEOUT = 0.1
START_BYTE = '<0>'
PROGRAM_BYTE = '<1>'
MOVE_BYTE = '<2>'
DELAY_BYTE = '<3>'
WAIT_BYTE = '<4>'
SET_BYTE = '<5>'
HOME_BYTE = '<6>'
TEACH_IN_BYTE = '<7>'  ### not used now ###
ENABLE_BYTE = '<8>'
END_MESSAGE = '<#>'
PLOT_WIDTH = 100
MAX_PROGRAM_LENGTH = 25


class DeltaGUI:

    def __init__(self):

        self.delta = deltarobot.DeltaRobot()
        self.current_file_name = None
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
        self.settings_window = None
        self.program_popup_opened = False
        self.plot_angles_opened = False
        self.plot_xyz_opened = False
        self.update_angles = False
        self.update_xyz = False
        self.trajectory_var = tk.IntVar()
        self.position_units = tk.IntVar()
        self.online = tk.BooleanVar()
        self.ser = None
        self.serial_connected = False
        self.send_manual = False
        self.send_program = False
        self.send_start = False
        self.send_stop = False
        self.send_enable = False
        self.send_disable = False
        self.send_home = False
        self.send_jog = False
        self.send_move_home = False
        self.jog = 0
        self.jog_step = 15

        self.x_values_angles = []
        self.y_values_angles = [[], [], []]
        self.x_values_xyz = []
        self.y_values_xyz = [[], [], []]
        self.trajectory_points = [[], [], []]

        self.plot_parts = [0, 0, 0, 0, 0, 0, 0, 0]
        self.trajectory_id = None
        self.init = True

        self.message_sent = True

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
        self.file_menu = tk.Menu(self.menubar, tearoff=0)
        self.menubar.add_cascade(label='File', menu=self.file_menu)
        self.file_menu.add_command(label="Settings", command=self.settingsWindow)
        self.file_menu.add_command(label='Exit', command=self.exit)
        self.menubar.add_command(label='Program', command=self.programCreator)
        self.plot_menu = tk.Menu(self.menubar, tearoff=0)
        self.menubar.add_cascade(label='Plots', menu=self.plot_menu)
        self.plot_menu.add_command(label='Joints', command=self.create2DPlotAngles)
        self.plot_menu.add_command(label='XYZ', command=self.create2DPlotxyz)
        self.menubar.add_command(label="Available COMs", command=showAvailableComs)

        self.serialPortFrame(self.root, row=0, column=1, rowspan=1, columnspan=3, sticky='s')
        self.manualControlFrame(self.root, row=1, column=1, rowspan=1, columnspan=3, sticky='w')
        self.jogFrame(self.root, row=2, column=1, columnspan=3, sticky='w')
        self.robotFrame(self.root, row=0, column=4, rowspan=15)
        self.commandFrame(self.root, row=3, column=1, rowspan=1, columnspan=1)
        self.statusFrame(self.root, row=4, column=1, columnspan=3, sticky='w')

        self.t = time.time()  ###############################
        self.refresh_times = []  ################################
        self.elapsed_time = time.time()  ################################
        self.savecsv = True

        self.loadConfig()
        self.readEncoders()

        tk.mainloop()

    def exit(self):
        """ Close the application """
        self.root.quit()
        self.root.destroy()

    def connect(self):
        """ Connect to/disconnect from the COM port """
        port = self.serial_port_entry.get()
        try:
            # if no serial port exists, create one and connect
            if self.ser is None:
                self.ser = serial.Serial()
                self.connectToComm(port, open=True)
            # else if serial port exists and is opened close it, if exists and is closed - open it
            else:
                if self.ser.isOpen():
                    self.connectToComm(port, open=False)
                else:
                    self.connectToComm(port, open=True)
        except serial.SerialException as e:
            message = str(e).partition(':')[0]
            self.port_status_label.config(text=message)
            print(message)

    def connectToComm(self, port, open=True):
        """ Open/close serial port and configure house-keeping values and labels """
        if open:
            self.openSerialPort(port)
            self.port_status_label.config(text=f"Connected to {port}")
            self.serial_connect_button.config(text="Disconnect")
            self.serial_connected = True
        if not open:
            self.ser.close()
            self.port_status_label.config(text="Disconnected")
            self.serial_connect_button.config(text="Connect")
            self.serial_connected = False

    def openSerialPort(self, port):
        """ Open already existing serial port with resetting it """
        self.ser.port = port
        self.ser.baudrate = BAUDRATE
        self.ser.timeout = TIMEOUT
        self.ser.dtr = 0  # in order not to reset the arduino everytime when a serial is connected
        self.ser.open()

    def saveRefreshTimes(self):
        """ Saves refresh times into a csv file """
        dataframe = pd.DataFrame(self.refresh_times)
        dataframe.to_csv('refresh_times.csv', index=False, header=False, mode='a')

        tk.messagebox.showinfo(message="Refresh times saved.")

    def readEncoders(self):
        """ Reads position from encoders in a continuous loop, thus the position is always up to date """
        if self.serial_connected:
            if self.ser.in_waiting != 0:  # if there's data to read
                try:
                    data = self.ser.readline().decode('utf-8')
                    data = json.loads(data)
                    info_angle = data["deg"][0]
                    # print(f"{data = }")
                except UnicodeDecodeError as e:
                    pass
                    print(f"{e= }")
                except ValueError as e:
                    pass
                    print(f"{e= }")
                except UnboundLocalError as e:
                    pass
                    print(f"{e= }")
                except TypeError as e:
                    pass
                    print(f"{e= }")
                else:
                    self.sendData()

                    end_time = time.time()
                    if self.t - self.elapsed_time > 15:
                        self.refresh_times.append(end_time - self.t)
                    self.t = time.time()

                    if self.online.get():  # read angles
                        writeDataToAngles(data, self.angles)
                        self.update3DPlot(manual=True, online=True,  # update visualization
                                          angles=(self.angles[0], self.angles[1], self.angles[2]))
                        if self.update_angles:
                            self.update2DPlotAngles()
                        if self.update_xyz:
                            self.update2DPlotxyz()

            try:
                while self.ser.in_waiting:
                    waste = self.ser.readline()
                # self.ser.reset_input_buffer()
            except serial.serialutil.PortNotOpenError as e:
                print(e)
            except serial.serialutil.SerialException as e:
                print(e)

        # if self.t - self.elapsed_time > 45 and self.savecsv:
        #     self.saveRefreshTimes()
        #     self.savecsv = False

        self.root.after(10, self.readEncoders)

    def sendData(self):
        """ Send data to device """
        # if self.arduino_available:  # if able to send data
        if self.send_start:
            self.startCommand()  # if there was a command to start
        if self.send_stop:
            self.stopCommand()  # if there was a command to stop
        if self.send_enable:
            self.enableCommand()  # if there was a command to enable motors
        if self.send_disable:
            self.disableCommand()  # if there was a command to disable motors
        if self.send_home:
            self.homeCommand()  # if there was a command to calibrate motors
        if self.send_manual:
            self.manualMove()  # if there was a command to move manually
        if self.send_move_home:
            self.manualMove(fi_values=(0, 0, 0))
        elif self.send_jog:
            self.jogMove()  # if there was a command to jog
        elif self.send_program:
            self.uploadProgram()  # if there was a command to send a program

    def changeUnits(self):
        """ Change labels between [mm] and [deg] based on selected mode """
        if self.position_units.get() == 0:
            self.x_position_label.config(text="X [mm]")
            self.y_position_label.config(text="Y [mm]")
            self.z_position_label.config(text="Z [mm]")
        elif self.position_units.get() == 1:
            self.x_position_label.config(text="φ1 [deg]")
            self.y_position_label.config(text="φ2 [deg]")
            self.z_position_label.config(text="φ3 [deg]")

    def sendMessage(self, message):
        """ Sends a provided message to the self.ser serial port """
        if self.serial_connected:
            self.ser.write(message.encode('utf-8'))
            self.ser.write(END_MESSAGE.encode('utf-8'))
            return False
        else:
            tk.messagebox.showwarning(title="Not connected", message="Not connected to device.")

    def startCommand(self):
        """ Send start command """
        message = START_BYTE + '<{"start": true}>'
        self.send_start = self.sendMessage(message)
        print(message)

    def stopCommand(self):
        """ Send stop command """
        message = START_BYTE + '<{"start": false}>'
        self.send_stop = self.sendMessage(message)
        print(message)

    def enableCommand(self):
        """ Send enable command """
        message = ENABLE_BYTE + '<{"enable": 0}>'
        self.send_enable = self.sendMessage(message)
        print(message)

    def disableCommand(self):
        """ Send disable command """
        message = ENABLE_BYTE + '<{"enable": 1}>'
        self.send_disable = self.sendMessage(message)
        print(message)

    def homeCommand(self):
        """ Send home command """
        message = HOME_BYTE + '<{"home": 1}>'
        self.send_home = self.sendMessage(message)

    def teachInCommand(self):
        """ Get current position from encoders and add it to the program point list """
        if self.serial_connected:
            converted_point = self.delta.calculateFPK(self.angles)
            converted_point = [round(point, 2) for point in converted_point]
            self.addPointToList(get_current_position=True, point=converted_point)
        else:
            tk.messagebox.showwarning(title="Not connected", message="Not connected to device.")

    def serialPortFrame(self, master, row=0, column=0, rowspan=1, columnspan=1, sticky=''):
        """ Frame on main screen responsible for Serial Port connection """
        # Create Serial port frame
        self.fr_ser = tk.Frame(master, bg="White")
        self.fr_ser.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan, sticky=sticky)
        #, highlightbackground="blue", highlightthickness=1

        # Labels - Serial port frame
        self.serial_port_label = tk.Label(self.fr_ser, text="Serial port", font=(FONT, TEXT_SIZE), bg="White")
        self.serial_port_label.grid(padx=10, pady=10, row=0, column=0, columnspan=2)
        self.port_status_label = tk.Label(self.fr_ser, text="Not connected", font=(FONT, TEXT_SIZE), bg="White",
                                          wraplength=250, width=20)
        self.port_status_label.grid(row=0, column=4, columnspan=10, rowspan=1, sticky='w')

        # Entries - Serial port frame
        self.serial_port_entry = tk.Entry(self.fr_ser, width=10, bg="White")
        self.serial_port_entry.insert(0, "COM3")
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
        self.move_button.bind('<Button-1>', self.setSendManual)

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
        self.x_plus_button.bind('<ButtonPress-1>', lambda event, axis="x", direction=0: self.Jog(event, axis=axis,
                                                                                                 direction=direction))
        # self.x_plus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.x_plus_button.grid(padx=7, pady=5, row=0, column=2)

        self.x_minus_button = tk.Button(self.fr_jog, text="X-", width=7)
        self.x_minus_button.bind('<ButtonPress-1>', lambda event, axis="x", direction=1: self.Jog(event, axis=axis,
                                                                                                  direction=direction))
        # self.x_minus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.x_minus_button.grid(padx=7, pady=5, row=0, column=3)

        self.y_plus_button = tk.Button(self.fr_jog, text="Y+", width=7)
        self.y_plus_button.bind('<ButtonPress-1>', lambda event, axis="y", direction=0: self.Jog(event, axis=axis,
                                                                                                 direction=direction))
        # self.y_plus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.y_plus_button.grid(padx=7, pady=5, row=1, column=2)

        self.y_minus_button = tk.Button(self.fr_jog, text="Y-", width=7)
        self.y_minus_button.bind('<ButtonPress-1>', lambda event, axis="y", direction=1: self.Jog(event, axis=axis,
                                                                                                  direction=direction))
        # self.y_minus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.y_minus_button.grid(padx=7, pady=5, row=1, column=3)

        self.z_plus_button = tk.Button(self.fr_jog, text="Z+", width=7)
        self.z_plus_button.bind('<ButtonPress-1>', lambda event, axis="z", direction=0: self.Jog(event, axis=axis,
                                                                                                 direction=direction))
        # self.z_plus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.z_plus_button.grid(padx=7, pady=5, row=2, column=2)

        self.z_minus_button = tk.Button(self.fr_jog, text="Z-", width=7)
        self.z_minus_button.bind('<ButtonPress-1>', lambda event, axis="z", direction=1: self.Jog(event, axis=axis,
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
        """ Frame on main screen responsible for showing status of the robot """
        self.fr_status = tk.Frame(master, bg="White")
        self.fr_status.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan, sticky=sticky)

        # Labels
        self.current_loaded_program_label = tk.Label(self.fr_status, text="Loaded program: ",
                                                     font=(FONT, TEXT_SIZE - 2),
                                                     bg="White")
        self.current_loaded_program_label.grid(row=0, column=0)
        self.fr_status_program_name = tk.Label(self.fr_status, text="", font=(FONT, TEXT_SIZE - 2), bg="White",
                                               wraplength=250)
        self.fr_status_program_name.grid(row=0, column=1, sticky='w')

    def commandFrame(self, master, row=0, column=0, rowspan=1, columnspan=1, sticky=''):
        """ Frame on main screen responsible for sending basic commands to the robot """
        self.fr_command = tk.Frame(master, bg="White")
        self.fr_command.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan, sticky=sticky)

        # Buttons
        self.fr_command_start_button = tk.Button(self.fr_command, text="Start", font=(FONT, TEXT_SIZE), bg="White",
                                                 command=self.setSendStart)
        self.fr_command_start_button.grid(row=1, column=1, columnspan=1)

        self.fr_command_stop_button = tk.Button(self.fr_command, text="Stop", font=(FONT, TEXT_SIZE), bg="White",
                                                command=self.setSendStop)
        self.fr_command_stop_button.grid(row=2, column=1)

        self.fr_command_enable_button = tk.Button(self.fr_command, text="Enable motors", font=(FONT, TEXT_SIZE),
                                                  bg="White",
                                                  command=self.setSendEnable)
        self.fr_command_enable_button.grid(row=1, column=2, columnspan=1)

        self.fr_command_disable_button = tk.Button(self.fr_command, text="Disable motors", font=(FONT, TEXT_SIZE),
                                                   bg="White",
                                                   command=self.setSendDisable)
        self.fr_command_disable_button.grid(row=2, column=2, columnspan=1)

        self.fr_command_home_button = tk.Button(self.fr_command, text="Calibrate", font=(FONT, TEXT_SIZE), bg="White",
                                                command=self.setSendHome)
        self.fr_command_home_button.grid(row=3, column=1, columnspan=1)

        self.fr_command_home_position = tk.Button(self.fr_command, text="Home", font=(FONT, TEXT_SIZE), bg="White",
                                                  command=self.setMoveHome)
        self.fr_command_home_position.grid(row=4, column=1, columnspan=1)

        self.fr_command_trajectory_checkbox = tk.Checkbutton(self.fr_command, text="Trajectory plot",
                                                             font=(FONT, TEXT_SIZE), bg="White",
                                                             variable=self.trajectory_var)
        self.fr_command_trajectory_checkbox.grid(row=5, column=1, columnspan=1)

    def programCreator(self):
        """ Creates a popup window that can be used to create, edit, open and save programs """
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

    def settingsWindow(self):
        """ Create and show a popup window where TCP offset can be configured """
        if self.settings_window is not None:
            self.settings_window.focus()
            return
        self.settings_window = tk.Toplevel(self.root)
        self.settings_window.title("Settings")
        self.settings_window.config(padx=10, pady=10, bg="White")
        self.settings_window.iconbitmap("delta icon.ico")
        self.settings_window.protocol("WM_DELETE_WINDOW", self.onCloseSettings)

        # Labels
        self.tcp_x_label = tk.Label(self.settings_window, text="x offset [mm]:", font=(FONT, TEXT_SIZE), bg="White")
        self.tcp_x_label.grid(padx=5, pady=5, row=2, column=1)
        self.tcp_y_label = tk.Label(self.settings_window, text="y offset [mm]:", font=(FONT, TEXT_SIZE), bg="White")
        self.tcp_y_label.grid(padx=5, pady=5, row=3, column=1)
        self.tcp_z_label = tk.Label(self.settings_window, text="z offset [mm]:", font=(FONT, TEXT_SIZE), bg="White")
        self.tcp_z_label.grid(padx=5, pady=5, row=4, column=1)
        self.jog_step_label = tk.Label(self.settings_window, text="Jog step [mm]:", font=(FONT, TEXT_SIZE), bg="White")
        self.jog_step_label.grid(padx=5, pady=15, row=5, column=1)
        self.z_limit_label = tk.Label(self.settings_window, text="Z range limit [mm]:", font=(FONT, TEXT_SIZE), bg="White")
        self.z_limit_label.grid(padx=5, pady=15, row=6, column=1)

        # Entries
        self.tcp_x_entry = tk.Entry(self.settings_window, width=10)
        self.tcp_x_entry.grid(padx=5, pady=5, row=2, column=2)
        self.tcp_x_entry.insert(0, self.delta.TCP[0])
        self.tcp_y_entry = tk.Entry(self.settings_window, width=10)
        self.tcp_y_entry.grid(padx=5, pady=5, row=3, column=2)
        self.tcp_y_entry.insert(0, self.delta.TCP[1])
        self.tcp_z_entry = tk.Entry(self.settings_window, width=10)
        self.tcp_z_entry.grid(padx=5, pady=5, row=4, column=2)
        self.tcp_z_entry.insert(0, self.delta.TCP[2])
        self.jog_step_entry = tk.Entry(self.settings_window, width=10)
        self.jog_step_entry.grid(padx=5, pady=15, row=5, column=2)
        self.jog_step_entry.insert(0, str(self.jog_step))
        self.z_limit_entry = tk.Entry(self.settings_window, width=10)
        self.z_limit_entry.grid(padx=5, pady=15, row=6, column=2)
        self.z_limit_entry.insert(0, self.delta.z_limit)

        # Buttons
        self.tcp_accept_button = tk.Button(self.settings_window, text="Write", font=(FONT, TEXT_SIZE - 1), bg="White",
                                           command=self.writeSettings)
        self.tcp_accept_button.grid(padx=5, pady=5, row=3, column=3)
        self.tcp_save_button = tk.Button(self.settings_window, text="Save", font=(FONT, TEXT_SIZE - 1), bg="White",
                                           command=self.saveConfig)
        self.tcp_save_button.grid(padx=5, pady=5, row=4, column=3)

    def programPointFrame(self, master):
        """ Frame to display program points """
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
        self.program_tree.bind("<Double-1>", self.selectProgramPoint)

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

    def selectProgramPoint(self, event):
        """ Selects and writes data from selected point into entries and comboboxes """
        interpolation = 0
        item = self.program_tree.selection()[0]
        values = self.program_tree.item(item, 'values')

        if values[6] == "Linear":
            interpolation = 1
        elif values[6] == "Circular":
            interpolation = 2
        self.x_entry.delete(0, "end")
        self.x_entry.insert(0, values[1])
        self.y_entry.delete(0, "end")
        self.y_entry.insert(0, values[2])
        self.z_entry.delete(0, "end")
        self.z_entry.insert(0, values[3])
        print(int(int(values[4].partition('%')[0]) / 10) - 1)
        self.velocity_fr_add_combobox.current(int(int(values[4].partition('%')[0]) / 10) - 1)
        self.acc_fr_add_combobox.current(int(int(values[5].partition('%')[0]) / 10) - 1)
        self.interpolation_fr_add_combobox.current(interpolation)

    def programFunctionsFrame(self, master):
        """ Frame to display program functions """
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
        self.func_tree.heading("point no.", text="After point")
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
        """ Frame to add points to program """
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
        self.x_entry.insert(0, "0.0")
        self.x_entry.bind("<Return>", self.addPointToList)
        self.y_entry = tk.Entry(self.fr_add_point, width=10)
        self.y_entry.grid(padx=10, pady=10, row=1, column=1)
        self.y_entry.insert(0, "0.0")
        self.y_entry.bind("<Return>", self.addPointToList)
        self.z_entry = tk.Entry(self.fr_add_point, width=10)
        self.z_entry.grid(padx=10, row=1, column=2)
        self.z_entry.insert(0, "0.0")
        self.z_entry.bind("<Return>", self.addPointToList)

        # Buttons fr add points
        self.add_point_button = tk.Button(self.fr_add_point, text="Add point", font=(FONT, TEXT_SIZE), bg="White",
                                          width=12)
        self.add_point_button.grid(padx=10, pady=5, row=5, column=0, columnspan=3)
        self.add_point_button.bind("<ButtonPress-1>", self.addPointToList)

        self.teach_in_button = tk.Button(self.fr_add_point, text="Teach-in", font=(FONT, TEXT_SIZE), bg="White",
                                         width=12,
                                         command=self.teachInCommand)
        self.teach_in_button.grid(padx=10, pady=5, row=6, column=0, columnspan=3)

        self.edit_point_button = tk.Button(self.fr_add_point, text="Edit point", font=(FONT, TEXT_SIZE), bg="White",
                                           width=12, command=self.editPoint)
        self.edit_point_button.grid(padx=10, pady=5, row=7, column=0, columnspan=3)

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
        """ Frame to add functions to program """
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
        """ Frame to upload program """
        # Create frame - upload program
        self.fr_upload_program = tk.Frame(master, bg="White")
        self.fr_upload_program.grid(padx=20, pady=0, row=1, column=1)

        # Buttons = fr upload program
        self.upload_button = tk.Button(self.fr_upload_program, text="Upload", font=(FONT, TEXT_SIZE), width=10,
                                       bg='White', command=self.setSendProgram)
        self.upload_button.grid(padx=10, row=0, column=0, columnspan=1)

    def create2DPlotAngles(self):
        """ Create a popup window with joint angles and XYZ positions plots """
        if self.plot_angles_opened:
            self.plot_angles.focus()
            return
        self.plot_angles_opened = True
        # Create window
        self.plot_angles = tk.Toplevel(self.root)
        self.plot_angles.title("Joints angles")
        self.plot_angles.config(padx=10, pady=10, bg="White")
        self.plot_angles.iconbitmap("delta icon.ico")
        self.plot_angles.protocol("WM_DELETE_WINDOW", self.onCloseAngles)

        # Create figure
        self.fig_plot_angles = plt.figure(figsize=(7, 7), dpi=100)
        self.angles_ax = self.fig_plot_angles.add_subplot(111)
        self.angles_ax.set_xlabel("Time")
        self.angles_ax.set_ylabel("Amplitude")
        self.angles_ax.grid()
        self.angles_ax.clear()

        # Create canvas from backend of matplotlib so it can be displayed in gui
        self.canvas_plot_angles = FigureCanvasTkAgg(self.fig_plot_angles, master=self.plot_angles)
        self.canvas_plot_angles.draw()
        self.canvas_plot_angles.get_tk_widget().grid(row=0, column=0, rowspan=1, columnspan=1, sticky='')
        self.toolbar_plot_angles = NavigationToolbar2Tk(self.canvas_plot_angles, self.plot_angles, pack_toolbar=False)
        self.toolbar_plot_angles.update()
        self.toolbar_plot_angles.grid(row=1, column=0, rowspan=1, columnspan=1, sticky='')

        self.fig_plot_angles.canvas.draw()
        self.fig_plot_angles.canvas.flush_events()

        self.update_angles = True

    def create2DPlotxyz(self):
        """ Create a popup window with joint angles and XYZ positions plots """
        if self.plot_xyz_opened:
            self.plot_xyz.focus()
            return
        self.plot_xyz_opened = True
        # Create window
        self.plot_xyz = tk.Toplevel(self.root)
        self.plot_xyz.title("XYZ positions plot")
        self.plot_xyz.config(padx=10, pady=10, bg="White")
        self.plot_xyz.iconbitmap("delta icon.ico")
        self.plot_xyz.protocol("WM_DELETE_WINDOW", self.onClosexyz)

        # Create figure
        self.fig_plot_xyz = plt.figure(figsize=(7, 7), dpi=100)
        self.xyz_ax = self.fig_plot_xyz.add_subplot(111)
        self.xyz_ax.set_xlabel("Time")
        self.xyz_ax.set_ylabel("Amplitude")
        self.xyz_ax.grid()
        self.xyz_ax.clear()

        # Create canvas from backend of matplotlib so it can be displayed in gui
        self.canvas_plot_xyz = FigureCanvasTkAgg(self.fig_plot_xyz, master=self.plot_xyz)
        self.canvas_plot_xyz.draw()
        self.canvas_plot_xyz.get_tk_widget().grid(row=0, column=0, rowspan=1, columnspan=1, sticky='')
        self.toolbar_plot_xyz = NavigationToolbar2Tk(self.canvas_plot_xyz, self.plot_xyz, pack_toolbar=False)
        self.toolbar_plot_xyz.update()
        self.toolbar_plot_xyz.grid(row=1, column=0, rowspan=1, columnspan=1, sticky='')

        self.fig_plot_xyz.canvas.draw()
        self.fig_plot_xyz.canvas.flush_events()

        self.update_xyz = True

    def update2DPlotAngles(self, xlabel="time", ylabel="angle[deg]"):
        """ Updates the 2D plot. Arguments are x and y label names """
        if not self.serial_connected:  # if not connected, call this function regularly anyways
            return
        else:
            axis = self.angles_ax
            try:
                data = [self.angles[0], self.angles[1], self.angles[2]]
            except ValueError as e:
                print(e)
                addValuesIfIncorrect(self.y_values_angles)
            else:
                self.y_values_angles[0].append(data[0])
                self.y_values_angles[1].append(data[1])
                self.y_values_angles[2].append(data[2])
            # Delete first element if the plot is too long
            if len(self.y_values_angles[0]) >= PLOT_WIDTH:
                del self.y_values_angles[0][0]
                del self.y_values_angles[1][0]
                del self.y_values_angles[2][0]
            # Update x values - insert current time
            self.x_values_angles.append(dt.datetime.now().strftime("%M:%S:%f"))
            # Delete first element if the plot is too long
            if len(self.x_values_angles) >= PLOT_WIDTH:
                del self.x_values_angles[0]
            # Plot values and configure plot
            plot(axis, self.x_values_angles, self.y_values_angles, xlabel=xlabel, ylabel=ylabel,
                 labels=("Joint 1", "Joint 2", "Joint 3"))

            self.fig_plot_angles.canvas.draw()
            self.fig_plot_angles.canvas.flush_events()

    def update2DPlotxyz(self, xlabel="time", ylabel="d[mm]"):
        """ Updates the 2D plot. Arguments are x and y label names """
        if not self.serial_connected:
            return
        else:
            axis = self.xyz_ax
            try:
                data = [self.angles[0], self.angles[1], self.angles[2]]
                data = self.delta.calculateFPK(data)
            except ValueError as e:
                print(e)
                addValuesIfIncorrect(self.y_values_xyz)
            else:
                self.y_values_xyz[0].append(data[0])
                self.y_values_xyz[1].append(data[1])
                self.y_values_xyz[2].append(data[2])

            # Update x values - insert current time
            self.x_values_xyz.append(dt.datetime.now().strftime("%M:%S:%f"))
            # Delete first element if the plot is too long
            if len(self.x_values_xyz) >= PLOT_WIDTH:
                del self.x_values_xyz[0]
            # Delete first element if the plot is too long
            if len(self.y_values_xyz[0]) >= PLOT_WIDTH:
                del self.y_values_xyz[0][0]
                del self.y_values_xyz[1][0]
                del self.y_values_xyz[2][0]
            # Plot values and configure plot
            plot(axis, self.x_values_xyz, self.y_values_xyz, xlabel=xlabel, ylabel=ylabel,
                 labels=("X axis", "Y axis", "Z axis"))

            self.fig_plot_xyz.canvas.draw()
            self.fig_plot_xyz.canvas.flush_events()

    def programCreatorMenu(self, master):
        """ Menu bar in the program_creator pop-up window """
        self.program_creator_menu = tk.Menu(master)
        master.config(menu=self.program_creator_menu)
        self.program_creator_menu.add_command(label="Open", command=self.openProgram)
        self.program_creator_menu.add_command(label="Save", command=self.saveProgram)
        self.program_creator_menu.add_command(label="Save as", command=self.saveProgramAs)

    def writeSettings(self):
        """ Set value written in the configtcp popup window entries into delta object """
        try:
            self.delta.TCP[0] = round(float(self.tcp_x_entry.get()), 2)
            self.delta.TCP[1] = round(float(self.tcp_y_entry.get()), 2)
            self.delta.TCP[2] = round(float(self.tcp_z_entry.get()), 2)
            self.jog_step = round(float(self.jog_step_entry.get()), 2)
            self.delta.z_limit = round(float(self.z_limit_entry.get()), 2)
            if self.jog_step <= 0:
                tk.messagebox.showwarning(title="Wrong input", message="Step value has to be greater than 0")
                self.settings_window.focus()
                raise KeyError('Step value has to be greater than 0')
        except ValueError as e:
            tk.messagebox.showwarning(title="Wrong input", message="Input must be a number")
            self.settings_window.focus()
            print(e)
        except KeyError as e:
            print(e)

    def saveConfig(self):
        with open("config.txt", 'w') as file:
            config = f"{self.delta.TCP[0]}, {self.delta.TCP[1]}, {self.delta.TCP[2]}, {self.jog_step}, " \
                     f"{self.delta.z_limit}"
            file.write(config)

    def loadConfig(self):
        with open("config.txt", 'r') as file:
            config_string = file.read().replace(" ", "")
            config_values = config_string.split(',')
            try:
                config_values = [round(float(item), 2) for item in config_values]
                self.delta.TCP[0], self.delta.TCP[1], self.delta.TCP[2], self.jog_step, self.delta.z_limit = config_values
            except ValueError as e:
                print(e)
                self.delta.TCP[0], self.delta.TCP[1], self.delta.TCP[2], self.jog_step, self.delta.z_limit = (0, 0, 0, 5, -400)

    def updateProgramPoints(self, tree, data):
        """ Update the points in treeview """
        tree.delete(*self.program_tree.get_children())
        for index in range(len(data["index_of_point"])):
            tree.insert('', index='end', text='', values=(data["index_of_point"][index],
                                                          data["coordinates"]["x"][index],
                                                          data["coordinates"]["y"][index],
                                                          data["coordinates"]["z"][index],
                                                          data["velocity"][index],
                                                          data["acceleration"][index],
                                                          data["interpolation"][index]))

    def updateFunctions(self, tree, data):
        """ Update the functions in treeview """
        tree.delete(*self.func_tree.get_children())
        for index in range(len(data["value"])):
            tree.insert('', index='end', text='', values=(data["func_type"][index],
                                                          data["pt_no"][index],
                                                          data["value"][index],
                                                          data["pin_no"][index]))

    def addPointToList(self, event=None, get_current_position=False, point=None):
        """ Add point with all its parameters to the treeview list """
        # Get respected parameters
        new_index_of_point = len(self.program_tree.get_children())
        vel = self.velocity_fr_add_combobox.get()
        acc = self.acc_fr_add_combobox.get()
        interpolation = self.interpolation_fr_add_combobox.get()
        if get_current_position:
            x, y, z = point
            # Insert parameters into the treeview
            self.program_tree.insert('', index='end', text='',
                                     values=(new_index_of_point, x, y, z, vel, acc, interpolation))
        else:
            try:
                x, y, z = round(float(self.x_entry.get()), 2), round(float(self.y_entry.get()), 2), \
                          round(float(self.z_entry.get()), 2)
            except ValueError:
                tk.messagebox.showwarning(title="Illegal data", message="Improper data entered!")
                self.program_creator.focus()
            else:
                if new_index_of_point < MAX_PROGRAM_LENGTH:
                    # Insert parameters into the treeview
                    self.program_tree.insert('', index='end', text='',
                                             values=(new_index_of_point, x, y, z, vel, acc, interpolation))
                else:
                    tk.messagebox.showwarning(title="Illegal data", message="Reached maximum program length")
                    self.program_creator.focus()

    def editPoint(self):
        try:
            item = self.program_tree.selection()[0]
        except IndexError as e:
            print("no item selected: ", e)
        else:
            self.program_tree.set(item, column=1, value=round(float(self.x_entry.get()), 2))
            self.program_tree.set(item, column=2, value=round(float(self.y_entry.get()), 2))
            self.program_tree.set(item, column=3, value=round(float(self.z_entry.get()), 2))
            self.program_tree.set(item, column=4, value=self.velocity_fr_add_combobox.get())
            self.program_tree.set(item, column=5, value=self.acc_fr_add_combobox.get())
            self.program_tree.set(item, column=6, value=self.interpolation_fr_add_combobox.get())

    def addFuncToList(self):
        """ Add program functions into function treeview"""
        if self.fr_add_func_function_combobox.get() == "Wait time":
            self.fr_add_func_pin_entry.delete(0, 'end')
            self.fr_add_func_pin_entry.insert(0, '0')

        number_of_functions = len(self.func_tree.get_children())
        try:
            point_no = int(self.fr_add_func_point_no_entry.get())
            value = int(self.fr_add_func_value_entry.get())
            pin = int(self.fr_add_func_pin_entry.get())
            if point_no < 0 or value < 0 or pin < 0:
                tk.messagebox.showwarning(title="Wrong input", message="Values cannot be smaller than 0")
                self.program_creator.focus()
                raise KeyError("Values cannot be smaller than 0")
        except ValueError as e:
            print(e)
            tk.messagebox.showwarning(title="Wrong input", message="Input must be a whole number!")
            self.program_creator.focus()
        except KeyError as e:
            print(e)
        else:
            if number_of_functions < MAX_PROGRAM_LENGTH:
                self.func_tree.insert('', index='end', text='', values=(self.fr_add_func_function_combobox.get(),
                                                                        point_no,
                                                                        value,
                                                                        pin))
            else:
                tk.messagebox.showwarning(title="Illegal data", message="Reached maximum program length")
                self.program_creator.focus()

    def deletePointFromList(self, master):
        """ Delete point from program points list """
        for item in master.selection():
            master.delete(item)

        new_index = 0
        for item in self.program_tree.get_children():
            self.program_tree.set(item, column=0, value=new_index)
            new_index += 1

    def moveUpSelected(self):
        """ Move program point up in the treeview """
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

    def moveDownSelected(self):
        """ Move program point down in the treeview """
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
        """ Ask for program save and destroy window when closing Program_creator popup window """
        if tk.messagebox.askyesno(title="Exit?", message="Do you want to save the program?"):
            self.saveProgram()
        self.program_creator.destroy()
        self.program_popup_opened = False

    def onCloseAngles(self):
        """ Close angle plotting popup window """
        self.plot_angles.destroy()
        self.plot_angles_opened = False
        self.update_angles = False

    def onClosexyz(self):
        """ Close xyz plotting popup window """
        self.plot_xyz.destroy()
        self.plot_xyz_opened = False
        self.update_xyz = False

    def onCloseSettings(self):
        self.settings_window.destroy()
        self.settings_window = None

    def pointsToJson(self):
        """ Rewrite points from treeview into json format, return as string """
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
        """ Rewrite functions from treeview into json format, return as string """
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
        """ Upload a program to the robot controller """
        success1, success2 = self.uploadProgramPoints(), self.uploadProgramFunctions()
        if not success1 or not success2:
            pass
        else:
            tk.messagebox.showinfo(title="Complete", message="Upload completed")

        self.send_program = False
        self.ser.write(END_MESSAGE.encode('utf-8'))  # send end of message byte

    def uploadProgramPoints(self):
        """ Send program points to robot controller """
        # Update point_data structure
        self.pointsToJson()
        # Convert to a 'only-numbers' format
        self.convertParameters()
        # Create temporary variables
        temp_data_point = {"n": 0,
                           "i": 0,
                           "v": 0,
                           "a": 0,
                           "c": [0.0, 0.0, 0.0]
                           }
        # For each point represented by index_of_point rewrite the point's parameters into temporary variable
        # and check for collision or out of range errors
        for index in self.point_data['index_of_point']:
            writeDataToTempPoint(self.point_data, temp_data_point, index)
            try:
                self.delta.calculateIPK((temp_data_point['c'][0], temp_data_point['c'][1],
                                         temp_data_point['c'][2]))
                temp_data_point['c'][0] -= self.delta.TCP[0]
                temp_data_point['c'][1] -= self.delta.TCP[1]
                temp_data_point['c'][2] -= self.delta.TCP[2]
            except TypeError as e:
                tk.messagebox.showwarning(title="Out of range",
                                          message=f"Point {index} is out of range!\nPlease change coordinates.")
                self.program_creator.focus()
                print(e)
                return 0
            except ValueError as e:
                tk.messagebox.showwarning(title="Wrong data",
                                          message=f"Point {index} has incorrect parameters, please correct them.")
                self.program_creator.focus()
                print(e)
                return 0
            except CollisionException:
                tk.messagebox.showwarning(title="Collision detected",
                                          message=f"Moving to point {index} will result in a collision!")
                self.program_creator.focus()
                print("Collision detected! Please check the given coordinates.")
                return 0
            else:
                # For each point represented by index_of_point rewrite the point's parameters into temporary variable
                # and then dump it into json format
                temp_send_point = json.dumps(temp_data_point)
                data_to_send = PROGRAM_BYTE + '<' + temp_send_point + '>'  # Add 0 to represent that the data that is being sent is point parameters

                data_to_send = data_to_send.replace(" ", "")
                print(f"{data_to_send = }")
                self.ser.write(data_to_send.encode('utf-8'))

                incoming = self.ser.readall().decode('utf-8')
                print(incoming)
                if incoming != 'OK' and temp_data_point['n']:
                    tk.messagebox.showinfo(title="Uploading unsuccessful", message="Program did not upload correctly!")
                    return 0
        return 1

    def uploadProgramFunctions(self):
        """ Send functions to the robot controller """
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
                func_type) + '<' + temp_send_func + '>'  # Add func_type number to represent that the data
            # that is being sent is a certain function
            print(f"{data_to_send = }")

            self.ser.write(data_to_send.encode('utf-8'))

            incoming = self.ser.readall().decode('utf-8')
            if incoming != 'OK':
                tk.messagebox.showinfo(title="Uploading unsuccessful", message="Program did not upload correctly!")
                return 0
        return 1

    def openProgram(self):
        """ Open a program in txt format """
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
            program_name = basename(self.current_file_name)
            self.fr_program_point_program_name_label.config(text=program_name)
            self.fr_status_program_name.config(text=program_name)

        self.program_creator.focus()

    def saveProgram(self):
        """ Save already opened program """
        # Check for open file
        if self.current_file_name:
            with open(self.current_file_name, 'w') as data_file:
                data_file.write(f"{self.pointsToJson()}\n{self.funcsToJson()}")
        else:
            self.saveProgramAs()

    def saveProgramAs(self):
        """ Save program in .txt format """
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

    def manualMove(self, event=None, fi_values=None):
        """ Send point data and manual move command to the robot controller """
        temp_data_point = {"n": 0,
                           "i": 0,
                           "v": 0,
                           "a": 0,
                           "c": [0.0, 0.0, 0.0]
                           }

        if fi_values:
            temp_data_point['n'] = 0
            temp_data_point['i'] = self.interpolation
            temp_data_point['v'] = int(int(self.velocity) / 10)
            temp_data_point['a'] = int(int(self.acceleration) / 10)
            temp_data_point['c'][0] = round(fi_values[0] - self.delta.TCP[0], 2)
            temp_data_point['c'][1] = round(fi_values[1] - self.delta.TCP[1], 2)
            temp_data_point['c'][2] = round(fi_values[2] - self.delta.TCP[2], 2)
            self.ipktoFpk(temp_data_point)
            self.send_move_home = False

        elif self.update3DPlot(event, manual=True, onlycheck=self.online.get()) and self.online.get():
            temp_data_point['n'] = 0
            temp_data_point['i'] = self.interpolation
            temp_data_point['v'] = int(int(self.velocity) / 10)
            temp_data_point['a'] = int(int(self.acceleration) / 10)
            temp_data_point['c'][0] = round(float(self.x_position_entry.get()) - self.delta.TCP[0], 2)
            temp_data_point['c'][1] = round(float(self.y_position_entry.get()) - self.delta.TCP[1], 2)
            temp_data_point['c'][2] = round(float(self.z_position_entry.get()) - self.delta.TCP[2], 2)
            self.send_manual = False

            if self.position_units.get() == 1 or fi_values:
                self.ipktoFpk(temp_data_point)

        else:
            self.send_manual = False
            return

        temp_send_point = json.dumps(temp_data_point)
        data_to_send = MOVE_BYTE + '<' + temp_send_point + '>'  # Add PROGRAM_BYTE to represent that the data that is being sent is point parameters

        data_to_send = data_to_send.replace(" ", "")
        print(f"{data_to_send = }")
        self.ser.write(data_to_send.encode('utf-8'))
        # while self.ser.in_waiting == 0:
        #     pass  # .decode('utf-8')
        # incoming = self.ser.readall().decode('utf-8')
        # print(incoming)
        self.ser.write(END_MESSAGE.encode('utf-8'))  # send end of message byte

    def ipktoFpk(self, temp_data_point):
        temp_data_point['c'][0] += round(self.delta.TCP[0], 2)
        temp_data_point['c'][1] += round(self.delta.TCP[1], 2)
        temp_data_point['c'][2] += round(self.delta.TCP[2], 2)
        P = self.delta.calculateFPK(temp_data_point['c'])
        temp_data_point['c'][0] = round(P[0] - self.delta.TCP[0], 2)
        temp_data_point['c'][1] = round(P[1] - self.delta.TCP[1], 2)
        temp_data_point['c'][2] = round(P[2] - self.delta.TCP[2], 2)

    def jogMove(self):
        """ Send point data offset by JOG function and manual move command to the robot controller """
        temp_data_point = {"n": 0,
                           "i": 0,
                           "v": 0,
                           "a": 0,
                           "c": [0.0, 0.0, 0.0]
                           }
        next_position = 0
        if self.update3DPlot(manual=False, onlycheck=self.online.get(), xyz=self.position) and self.online.get():
            temp_data_point['n'] = 0
            temp_data_point['i'] = 0
            temp_data_point['v'] = 1
            temp_data_point['a'] = 1
            temp_data_point['c'][0] = round(self.position[0], 2)
            temp_data_point['c'][1] = round(self.position[1], 2)
            temp_data_point['c'][2] = round(self.position[2], 2)
            self.addJogSteps(temp_data_point=temp_data_point)

            temp_send_point = json.dumps(temp_data_point)
            data_to_send = MOVE_BYTE + '<' + temp_send_point + '>'  # Add PROGRAM_BYTE to represent that the data that is being sent is point parameters

            data_to_send = data_to_send.replace(" ", "")
            print(f"{data_to_send = }")
            self.ser.write(data_to_send.encode('utf-8'))
            # while self.ser.in_waiting == 0:
            #     pass  # .decode('utf-8')
            # incoming = self.ser.readall().decode('utf-8')
            # print(incoming)
            self.jog = 0
            self.send_jog = False
            self.ser.write(END_MESSAGE.encode('utf-8'))  # send end of message byte

    def create3DPlot(self, master):
        """ Create 3D plot in the master window """
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

        self.plotObjects()

        self.fig_robot.canvas.draw()
        self.fig_robot.canvas.flush_events()

    def update3DPlot(self, event=None, manual=False, xyz=(0, 0, 0), online=False, angles=(0, 0, 0), onlycheck=False):
        """ Update the plot with supplied x y z coordinates """
        try:
            # If jogging or auto, manual = False, if moving based on entered position - manual = True
            if manual:
                position = self.readCoordinates(online, angles)
            else:
                position = xyz
                if self.jog:
                    position = [coord + tcp for coord, tcp in zip(self.position, self.delta.TCP)]
                    position = self.addJogSteps(position=position)
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
            tk.messagebox.showwarning(title="Wrong input", message="Input must be a number!")
            print(e)
        except CollisionException:
            print("Collision detected! Please check the given coordinates.")
            self.collision_label.grid(row=8, column=3, rowspan=1)
        else:
            if not onlycheck:
                self.updateCurrentPosition(position, update_param=manual)
            if not self.online.get():
                self.angles = [angle * 180 / 3.14 for angle in self.delta.fi]
            # Hide any remaining labels
            self.hideErrorLabels()
            # Save point to trajectory
            self.trajectoryPoints()
            # Display current position
            self.labelPosition()

            if not onlycheck:
                if self.trajectory_var.get():
                    try:
                        self.trajectory_id.remove()
                    except AttributeError as e:
                        print(e)
                for item in self.plot_parts:
                    item.remove()

                # self.ax.clear() #######
                self.plotObjects()

                self.fig_robot.canvas.draw()
                self.fig_robot.canvas.flush_events()

            return 1

    def plotObjects(self):
        """ Plot objects - robot kinematic structure and trajectory"""
        if self.init:
            self.ax.plot(self.delta.Bvx, self.delta.Bvy, self.delta.Bvz, 'slategrey')  # Base
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

            self.init = False

        self.plot_parts[0], = self.ax.plot(self.delta.vert_coords.coordinates_x[0], self.delta.vert_coords.coordinates_y[0],
                        self.delta.vert_coords.coordinates_z[0], 'black')  # links
        self.plot_parts[1], = self.ax.plot(self.delta.vert_coords.coordinates_x[1], self.delta.vert_coords.coordinates_y[1],
                        self.delta.vert_coords.coordinates_z[1], 'black')  # links
        self.plot_parts[2], = self.ax.plot(self.delta.vert_coords.coordinates_x[2], self.delta.vert_coords.coordinates_y[2],
                        self.delta.vert_coords.coordinates_z[2], 'black')  # links
        self.plot_parts[3], = self.ax.plot(self.delta.vert_coords.coordinates_x[3], self.delta.vert_coords.coordinates_y[3],
                        self.delta.vert_coords.coordinates_z[3], 'black')  # links
        self.plot_parts[4], = self.ax.plot(self.delta.vert_coords.coordinates_x[4], self.delta.vert_coords.coordinates_y[4],
                        self.delta.vert_coords.coordinates_z[4], 'black')  # links
        self.plot_parts[5], = self.ax.plot(self.delta.vert_coords.coordinates_x[5], self.delta.vert_coords.coordinates_y[5],
                        self.delta.vert_coords.coordinates_z[5], 'black')  # links

        self.plot_parts[6], = self.ax.plot(self.delta.vert_coords.coordinates_x[6], self.delta.vert_coords.coordinates_y[6],
                        self.delta.vert_coords.coordinates_z[6], 'chocolate')  # Effector
        self.plot_parts[7], = self.ax.plot(self.delta.vert_coords.coordinates_x[7], self.delta.vert_coords.coordinates_y[7],
                        self.delta.vert_coords.coordinates_z[7], 'ro')  # TCP

        # Set axes limits
        self.ax.set_xlim(-300, 300)
        self.ax.set_ylim(-300, 300)
        self.ax.set_zlim(-605, 100)

        # Plot trajectory
        if self.trajectory_var.get():
            self.trajectory_id, = self.ax.plot(self.trajectory_points[0], self.trajectory_points[1], self.trajectory_points[2], 'b')

    def addElementsToPlot(self, *elements, color="black"):
        """ Add elements to plot described by x,y,z coordinates and connect them by lines """
        for element in elements:
            self.ax.plot(element[0], element[1], element[2], color=color)

    def addRightPrismToPlot(self, *args, z, height):
        """ Add vertices in order so that first one is next to the second one an so on. vertices = [[x1, x2, x3], [y1, y2, 3]] """
        bottom_polygon = [[], [], []]

        for xy_coord in args:
            bottom_polygon[0] = xy_coord[0]  # copy the whole list of X coordinates
            bottom_polygon[0].append(bottom_polygon[0][0])  # add the last point to close the polygon
            bottom_polygon[1] = xy_coord[1]
            bottom_polygon[1].append(bottom_polygon[1][0])  # add the last point to close the polygon
            bottom_polygon[2] = [z for _ in range(len(xy_coord[0]))]

            top_polygon = bottom_polygon.copy()
            top_polygon[2] = [z + height for length in range(len(xy_coord[0]))]

            self.addElementsToPlot(bottom_polygon, top_polygon, color='slategray')

            # Draw edges
            for i in range(len(xy_coord[0])):
                edge = [[bottom_polygon[0][i], bottom_polygon[0][i]], [bottom_polygon[1][i], bottom_polygon[1][i]],
                        [z, z + height]]
                self.addElementsToPlot(edge, color='slategray')

    def readCoordinates(self, online=False, angles=(0, 0, 0)):
        """ Returns a tuple of lists of x, y and z coordinates to draw"""
        if online:
            P = self.delta.calculateFPK(angles)
            x_coordinate = P[0]
            y_coordinate = P[1]
            z_coordinate = P[2]
        else:
            x_coordinate = float(self.x_position_entry.get())
            y_coordinate = float(self.y_position_entry.get())
            z_coordinate = float(self.z_position_entry.get())
            # If units are degrees, use FPK to get x y z
            if self.position_units.get() == 1:
                P = self.delta.calculateFPK((x_coordinate, y_coordinate, z_coordinate))
                x_coordinate = P[0]
                y_coordinate = P[1]
                z_coordinate = P[2]

        return x_coordinate, y_coordinate, z_coordinate

    def updateCurrentPosition(self, xyz, update_param=False):
        """ Updates current position and parameter values """
        self.position[0] = xyz[0] - self.delta.TCP[0]
        self.position[1] = xyz[1] - self.delta.TCP[1]
        self.position[2] = xyz[2] - self.delta.TCP[2]

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

    def trajectoryPoints(self):
        """ Updates trajectory points """
        self.trajectory_points[0].append(self.position[0])
        self.trajectory_points[1].append(self.position[1])
        self.trajectory_points[2].append(self.position[2])
        if len(self.trajectory_points[0]) > 20:
            del self.trajectory_points[0][0]
            del self.trajectory_points[1][0]
            del self.trajectory_points[2][0]

    def labelPosition(self):
        """ Updates current position in labels """
        self.coord_current_label.config(
            text=f"x: {round(self.position[0] + self.delta.TCP[0], 2)} [mm] "
                 f"y: {round(self.position[1] + self.delta.TCP[1], 2)} [mm] "
                 f"z: {round(self.position[2] + self.delta.TCP[2], 2)} [mm]")
        self.coord_current_label.grid(row=3, column=2, rowspan=1, columnspan=6)
        self.angle_current_label.config(
            text=f"φ1: {round(self.angles[0], 2)}° φ2: {round(self.angles[1], 2)}° "
                 f"φ3: {round(self.angles[2], 2)}°")
        self.angle_current_label.grid(row=4, column=2, rowspan=1, columnspan=6)

    def hideErrorLabels(self):
        """ Hides error labels """
        self.coord_warning_label.grid_forget()
        self.typeerror_entry_label.grid_forget()
        self.collision_label.grid_forget()

    def Jog(self, event, axis, direction):
        """ Interpret the input jog command and call corresponding method """
        if axis == "x":
            if direction == 0:
                self.jog = 1
                self.setSendManual(jog=True)
            elif direction == 1:
                self.jog = 2
                self.setSendManual(jog=True)
        elif axis == "y":
            if direction == 0:
                self.jog = 3
                self.setSendManual(jog=True)
            elif direction == 1:
                self.jog = 4
                self.setSendManual(jog=True)
        elif axis == "z":
            if direction == 0:
                self.jog = 5
                self.setSendManual(jog=True)
            elif direction == 1:
                self.jog = 6
                self.setSendManual(jog=True)

    def addJogSteps(self, temp_data_point=None, position=None):
        """ Add jog steps to the give position """
        if temp_data_point:
            if self.jog == 1:
                temp_data_point['c'][0] = round(temp_data_point['c'][0] + self.jog_step, 2)
            elif self.jog == 2:
                temp_data_point['c'][0] = round(temp_data_point['c'][0] - self.jog_step, 2)
            elif self.jog == 3:
                temp_data_point['c'][1] = round(temp_data_point['c'][1] + self.jog_step, 2)
            elif self.jog == 4:
                temp_data_point['c'][1] = round(temp_data_point['c'][1] - self.jog_step, 2)
            elif self.jog == 5:
                temp_data_point['c'][2] = round(temp_data_point['c'][2] + self.jog_step, 2)
            elif self.jog == 6:
                temp_data_point['c'][2] = round(temp_data_point['c'][2] - self.jog_step, 2)

        elif position:
            if self.jog == 1:
                return position[0] + self.jog_step, position[1], position[2]
            elif self.jog == 2:
                return position[0] - self.jog_step, position[1], position[2]
            elif self.jog == 3:
                return position[0], position[1] + self.jog_step, position[2]
            elif self.jog == 4:
                return position[0], position[1] - self.jog_step, position[2]
            elif self.jog == 5:
                return position[0], position[1], position[2] + self.jog_step
            elif self.jog == 6:
                return position[0], position[1], position[2] - self.jog_step

    def setSendStart(self):
        """ Set a parameter that defines that start command needs to be sent """
        self.send_start = True

    def setSendStop(self):
        """ Set a parameter that defines that stop command needs to be sent """
        self.send_stop = True

    def setSendEnable(self):
        """ Set a parameter that defines that enable command needs to be sent """
        self.send_enable = True

    def setSendDisable(self):
        """ Set a parameter that defines that disable command needs to be sent """
        self.send_disable = True

    def setSendHome(self):
        """ Set a parameter that defines that home command needs to be sent """
        self.send_home = True

    def setMoveHome(self):
        self.send_move_home = True

    def setSendProgram(self, event=None):
        """ Set a parameter that defines that program needs to be sent """
        # If file is not saved, inform the user
        if not self.current_file_name:
            tk.messagebox.showinfo(title="Cannot upload", message="Save file before upload.")
            self.program_creator.focus()
        elif not self.serial_connected:
            tk.messagebox.showinfo(title="Cannot upload", message="Connect to device before uploading.")
            self.program_creator.focus()
        else:
            self.send_program = True

    def setSendManual(self, event=None, jog=False):
        """ Set a parameter that defines that manual move command needs to be sent """
        if not self.serial_connected and self.online.get():
            tk.messagebox.showinfo(title="Cannot upload", message="Connect to device before uploading.")
        elif not self.online.get():
            if not jog:
                self.manualMove()
            else:
                self.jogMove()
        elif jog:
            self.send_jog = True
        else:
            self.send_manual = True


def writeDataToTempPoint(data, temp_data_point, index):
    temp_data_point['n'] = data['index_of_point'][index]
    temp_data_point['i'] = data['interpolation'][index]
    temp_data_point['v'] = int(int(data['velocity'][index]) / 10)
    temp_data_point['a'] = int(int(data['acceleration'][index]) / 10)
    temp_data_point['c'][0] = float(data['coordinates']['x'][index])
    temp_data_point['c'][1] = float(data['coordinates']['y'][index])
    temp_data_point['c'][2] = float(data['coordinates']['z'][index])


def writeDataToAngles(data, angles):
    angles[0] = data["deg"][0]
    angles[1] = data["deg"][1]
    angles[2] = data["deg"][2]


def plot(axis, x_values, y_values, xlabel, ylabel, labels):
    axis.clear()
    axis.plot(x_values, y_values[0], 'g', label=labels[0])
    axis.plot(x_values, y_values[1], 'r', label=labels[1])
    axis.plot(x_values, y_values[2], 'b', label=labels[2])
    axis.set_xlabel(xlabel)
    axis.set_ylabel(ylabel)
    axis.grid()
    axis.xaxis.set_major_locator(plt.MaxNLocator(5))

    axis.legend(loc="upper left")


def addValuesIfIncorrect(values):
    if len(values[0]) == 0:
        values[0].append(0.0)
        values[1].append(0.0)
        values[2].append(0.0)
    else:
        values[0].append(values[0][-1])
        values[1].append(values[1][-1])
        values[2].append(values[2][-1])


def showAvailableComs():
    """ Shows a pop-up window with available COM ports"""
    ports = serial.tools.list_ports.comports()
    available_ports = ""
    for port, desc, hwid in sorted(ports):
        available_ports += str(port) + ': ' + str(desc) + '\n'
    tk.messagebox.showinfo(title="COM ports", message=available_ports)
