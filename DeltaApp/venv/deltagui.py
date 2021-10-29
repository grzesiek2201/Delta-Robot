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


FONT = "Times New Roman"
TEXT_SIZE = 12
BAUDRATE = 9600


class DeltaGUI:

    def __init__(self):

        self.delta = deltarobot.DeltaRobot()
        self.current_file_name = None
        # self.jobid = None
        self.position = [0.0, 0.0, 0.0]
        self.root = tk.Tk()
        self.root.geometry('1100x700')
        self.root.title("Delta GUI")
        self.root.config(padx=50, pady=20, bg="white")
        self.root.iconbitmap("delta icon.ico")
        self.program_popup_opened = False
        self.position_units = tk.IntVar()
        self.ser = None
        self.serial_connected = False

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

        # Menus
        self.menubar = tk.Menu(self.root)
        self.root.config(menu=self.menubar)
        self.menubar.add_command(label='Exit', command=self.root.destroy)
        self.menubar.add_command(label='Program', command=self.programCreator)

        self.serialPortFrame(self.root, row=0, column=1, rowspan=1, columnspan=3, sticky='s')
        self.manualControlFrame(self.root, row=1, column=1, rowspan=1, columnspan=3, sticky='w')
        self.jogFrame(self.root, row=2, column=1, columnspan=3, sticky='w')
        self.statusFrame(self.root, row=3, column=1, columnspan=3, sticky='w')
        self.robotFrame(self.root, row=0, column=4, rowspan=15)

        # Buttons
        self.start_button = tk.Button(self.root, text="Start", font=(FONT, TEXT_SIZE), bg="White")
        self.start_button.grid(row=5, column=1)

        tk.mainloop()

    def connect(self):
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

    def changeUnits(self):

        if self.position_units.get() == 0:
            self.x_position_label.config(text="X [mm]")
            self.y_position_label.config(text="Y [mm]")
            self.z_position_label.config(text="Z [mm]")
        elif self.position_units.get() == 1:
            self.x_position_label.config(text="φ1 [deg]")
            self.y_position_label.config(text="φ2 [deg]")
            self.z_position_label.config(text="φ3 [deg]")

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
        self.x_position_entry.bind("<Return>", self.updatePlot)

        self.y_position_entry = tk.Entry(self.fr_man_ctrl, width=10)
        self.y_position_entry.grid(row=3, column=3)
        self.y_position_entry.insert(0, "0")
        self.y_position_entry.bind("<Return>", self.updatePlot)

        self.z_position_entry = tk.Entry(self.fr_man_ctrl, width=10)
        self.z_position_entry.grid(row=3, column=4)
        self.z_position_entry.insert(0, "0")
        self.z_position_entry.bind("<Return>", self.updatePlot)

        # Buttons - manual control frame
        self.move_button = tk.Button(self.fr_man_ctrl, text="Move", font=(FONT, TEXT_SIZE - 1), bg="White")
        self.move_button.grid(pady=5, row=3, column=5, rowspan=1, columnspan=1)
        self.move_button.bind('<Button-1>', self.updatePlot)

        # Radiobutton - manual control frame
        self.position_units_angles = tk.Radiobutton(self.fr_man_ctrl, text="deg", variable=self.position_units, value=1,
                                                    bg="White", highlightcolor="White", command=self.changeUnits)
        self.position_units_angles.grid(row=1, column=2)
        self.position_units_mm = tk.Radiobutton(self.fr_man_ctrl, text="mm", variable=self.position_units, value=0, bg="White",
                                                command=self.changeUnits)
        self.position_units_mm.grid(row=1, column=1)

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

        self.coord_current_label = tk.Label(self.fr_jog, text="x: 0.0 [mm] y: 0.0 [mm] z: 0.0 [mm]", font=(FONT, 9), bg="White")
        self.coord_current_label.grid(row=3, column=2, rowspan=1, columnspan=4)
        self.angle_current_label = tk.Label(self.fr_jog, font=(FONT, 9), bg="White")
        self.angle_current_label.grid(row=4, column=2, rowspan=1, columnspan=4)

        # Buttons - JOG Frame
        self.x_plus_button = tk.Button(self.fr_jog, text="X+", width=7)
        self.x_plus_button.bind('<ButtonPress-1>', lambda event, axis="x", direction=0: self.moveJog(event, axis=axis, direction=direction))
        # self.x_plus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.x_plus_button.grid(padx=7, pady=5, row=0, column=2)

        self.x_minus_button = tk.Button(self.fr_jog, text="X-", width=7)
        self.x_minus_button.bind('<ButtonPress-1>', lambda event, axis="x", direction=1: self.moveJog(event, axis=axis, direction=direction))
        # self.x_minus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.x_minus_button.grid(padx=7, pady=5, row=0, column=3)

        self.y_plus_button = tk.Button(self.fr_jog, text="Y+", width=7)
        self.y_plus_button.bind('<ButtonPress-1>', lambda event, axis="y", direction=0: self.moveJog(event, axis=axis, direction=direction))
        # self.y_plus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.y_plus_button.grid(padx=7, pady=5, row=1, column=2)

        self.y_minus_button = tk.Button(self.fr_jog, text="Y-", width=7)
        self.y_minus_button.bind('<ButtonPress-1>', lambda event, axis="y", direction=1: self.moveJog(event, axis=axis, direction=direction))
        # self.y_minus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.y_minus_button.grid(padx=7, pady=5, row=1, column=3)

        self.z_plus_button = tk.Button(self.fr_jog, text="Z+", width=7)
        self.z_plus_button.bind('<ButtonPress-1>', lambda event, axis="z", direction=0: self.moveJog(event, axis=axis, direction=direction))
        # self.z_plus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.z_plus_button.grid(padx=7, pady=5, row=2, column=2)

        self.z_minus_button = tk.Button(self.fr_jog, text="Z-", width=7)
        self.z_minus_button.bind('<ButtonPress-1>', lambda event, axis="z", direction=1: self.moveJog(event, axis=axis, direction=direction))
        # self.z_minus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.z_minus_button.grid(padx=7, pady=5, row=2, column=3)

    def robotFrame(self, master, row=0, column=0, rowspan=1, columnspan=1, sticky=''):
        """ Frame on main screen responsible for plotting the robot """
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
        self.createPlot(self.fr_robot)

    def statusFrame(self, master, row=0, column=0, rowspan=1, columnspan=1, sticky=''):
        self.fr_status = tk.Frame(master, bg="White")
        self.fr_status.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan, sticky=sticky)

        self.current_loaded_program_label = tk.Label(self.fr_status, text="Loaded program: ", font=(FONT, TEXT_SIZE),
                                            bg="White")
        self.current_loaded_program_label.grid(row=0, column=0)

    def programCreator(self):

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
        self.program_creator.protocol("WM_DELETE_WINDOW", self.onClose)

        self.programPointFrame(self.program_creator)
        self.addPointFrame(self.program_creator)
        self.programCreatorMenu(self.program_creator)
        self.uploadProgramFrame(self.program_creator)

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
        self.delete_selected_button = tk.Button(self.fr_program_point, text="Delete selected", font=(FONT, TEXT_SIZE),
                                                bg="White", command=lambda: self.deletePointFromList(self.program_tree))
        self.delete_selected_button.grid(padx=10, row=3, column=0)
        self.move_up_button = tk.Button(self.fr_program_point, text="⬆", width=5, font=(FONT, TEXT_SIZE), bg="White",
                                        command=self.moveUpSelected)
        self.move_up_button.grid(padx=0, row=3, column=5)
        self.move_up_button = tk.Button(self.fr_program_point, text="⬇", width=5, font=(FONT, TEXT_SIZE), bg="White",
                                        command=self.moveDownSelected)
        self.move_up_button.grid(padx=0, row=3, column=6)

        # Labels fr program points
        self.program_name_label = tk.Label(self.fr_program_point, text="", font=(FONT, 9), bg="White")
        self.program_name_label.grid(row=2, column=6, columnspan=3)

    def addPointFrame(self, master):
        # Create frame - add point
        self.fr_add_point = tk.Frame(master, bg="White")
        self.fr_add_point.grid(padx=20, row=0, column=1, sticky='nsew')

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
        self.add_point_button = tk.Button(self.fr_add_point, text="Add", font=(FONT, TEXT_SIZE), bg="White", width=12)
        self.add_point_button.grid(padx=10, row=5, column=0, columnspan=3)
        self.add_point_button.bind("<ButtonPress-1>", self.addPointToList)

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
        """updates the points in treeview"""
        self.program_tree.delete(*self.program_tree.get_children())
        for index in range(len(data["index_of_point"])):
            self.program_tree.insert('', index='end', text='', values=(data["index_of_point"][index],
                                                                       data["coordinates"]["x"][index],
                                                                       data["coordinates"]["y"][index],
                                                                       data["coordinates"]["z"][index],
                                                                       data["velocity"][index],
                                                                       data["acceleration"][index],
                                                                       data["interpolation"][index]))

    def addPointToList(self, event):
        new_index_of_point = len(self.program_tree.get_children())
        self.program_tree.insert('', index='end', text='', values=(new_index_of_point,
                                                                   self.x_entry.get(),
                                                                   self.y_entry.get(),
                                                                   self.z_entry.get(),
                                                                   self.velocity_fr_add_combobox.get(),
                                                                   self.acc_fr_add_combobox.get(),
                                                                   self.interpolation_fr_add_combobox.get()))
        self.x_entry.delete(0, "end")
        self.x_entry.insert(0, "0")
        self.y_entry.delete(0, "end")
        self.y_entry.insert(0, "0")
        self.z_entry.delete(0, "end")
        self.z_entry.insert(0, "0")

    def deletePointFromList(self, master):  # only in GUI, no effect on points yet
        for item in master.selection():
            master.delete(item)

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

    def onClose(self):
        if tk.messagebox.askyesno(title="Exit?", message="Do you want to save the program?"):
            self.saveProgram()
        self.program_creator.destroy()
        self.program_popup_opened = False

    def pointsToJson(self):

        # Clear the dictionary before assigning new values
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

        # Load new values to the program_tree dictionary
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
        print(f"{self.point_data= }")

    def uploadProgram(self):

        # self.ser.write('1{"start": true}'.encode('utf-8'))
        # while self.ser.inWaiting() == 0:
        #     pass  # .decode('utf-8')
        # incoming = self.ser.readall().decode('utf-8')
        # print(incoming)
        # If file is not saved, inform the user
        if not self.current_file_name:
            tk.messagebox.showinfo(title="Cannot upload", message="Save file before upload.")
            self.program_creator.focus()
        else:
            # Update point_data structure
            self.pointsToJson()
            # Convert to a 'only-numbers' format
            self.convertParameters()
            # Create temporary variables
            temp_data_point = {"index_of_point": [],
                               "interpolation": [],
                               "velocity": [],
                               "acceleration": [],
                               "coordinates": {
                                   "x": [],
                                   "y": [],
                                   "z": []}
                               }

            # For each point represented by index_of_point rewrite the point's parameters into temporary variable and dump it into json format
            for index in self.point_data['index_of_point']:
                temp_data_point['index_of_point'] = self.point_data['index_of_point'][index]
                temp_data_point['interpolation'] = self.point_data['interpolation'][index]
                temp_data_point['velocity'] = self.point_data['velocity'][index]
                temp_data_point['acceleration'] = self.point_data['acceleration'][index]
                temp_data_point['coordinates']['x'] = self.point_data['coordinates']['x'][index]
                temp_data_point['coordinates']['y'] = self.point_data['coordinates']['y'][index]
                temp_data_point['coordinates']['z'] = self.point_data['coordinates']['z'][index]

                temp_send_point = json.dumps(temp_data_point)
                data_to_send = '0' + temp_send_point  # Add 0 to represent that the data that is being sent is point parameters
                print(f"{temp_send_point = }")

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
                data = json.load(data_file)
                for key in data:
                    self.point_data[key] = data[key]
        except FileNotFoundError as e:
            print(e)
        except JSONDecodeError:
            tk.messagebox.showinfo(title="Error reading file",
                                   message="Error while reading the file.\nCheck the file format (json formatting).")
        else:
            self.updateProgramPoints(self.program_tree, self.point_data)
            program_name = ntpath.basename(self.current_file_name)
            self.program_name_label.config(text=program_name)
        self.program_creator.focus()

    def saveProgram(self):

        # Check for open file
        if self.current_file_name:
            with open(self.current_file_name, 'w') as data_file:
                data_file.write(self.pointsToJson())
        else:
            self.saveProgramAs()

    def saveProgramAs(self):

        # Dialog window to save the file
        files = [('Text Document', '*.txt')]
        self.current_file_name = tk.filedialog.asksaveasfilename(filetypes=files, defaultextension=files)
        # Save program to file
        try:
            with open(self.current_file_name, 'w') as data_file:
                data_file.write(self.pointsToJson())
        except FileNotFoundError:
            pass
        self.program_creator.focus()

    def createPlot(self, master):
        # Create figure
        self.fig = plt.figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_xlim(-800, 800)
        self.ax.set_ylim(-800, 800)
        self.ax.set_zlim(-2000, 500)

        # Create canvas from backend of matplotlib so it can be displayed in gui
        self.canvas = FigureCanvasTkAgg(self.fig, master=master)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, rowspan=1, columnspan=6, sticky='n')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def updatePlot(self, event, manual=False, xyz=(0, 0, 0)):
        """Update the plot with supplied x y z coordinates"""
        try:
            # If jogging or running a program, use manual, if moving based on entered position - don't use manual
            if manual:
                position = xyz
            else:
                position = self.readCoordinates()
            self.delta.calculateIPK(position)  # catch an out of range error
            temp = self.delta.vert_coords.coordinates_x[1]  # catch an error when creating model that is out of range
        except TypeError as e:
            self.typeerror_entry_label.grid_forget()
            self.collision_label.grid_forget()
            self.coord_warning_label.grid(row=8, column=3, rowspan=1)  # show 'out of range' label
            print(e)
        except ValueError:
            print("Write only number in the coordinates entries.")
            self.typeerror_entry_label.grid(row=8, column=3, rowspan=1)
        except CollisionException:
            print("Collision detected! Please check the given coordinates.")
            self.collision_label.grid(row=8, column=3, rowspan=1)
        else:
            self.updateCurrentPosition(position)
            # Hide any remaining labels
            self.coord_warning_label.grid_forget()
            self.typeerror_entry_label.grid_forget()
            self.collision_label.grid_forget()
            # Display current position
            self.coord_current_label.config(
                text=f"x: {self.position[0]} [mm] y: {self.position[1]} [mm] z: {self.position[2]} [mm]")
            self.coord_current_label.grid(row=3, column=2, rowspan=1, columnspan=6)
            self.angle_current_label.config(
                text=f"φ1: {round(self.delta.fi[0] * 180 / np.pi, 2)}° φ2: {round(self.delta.fi[1] * 180 / np.pi, 2)}° "
                     f"φ3: {round(self.delta.fi[2] * 180 / np.pi, 2)}°")
            self.angle_current_label.grid(row=4, column=2, rowspan=1, columnspan=6)

            self.ax.clear()
            self.plotObjects()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

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

    def readCoordinates(self, ):
        """Sends x,y,z coordinates, velocity % and interpolation type as a json through serial port,
        returns a tuple of lists of x, y and z coordinates to draw"""

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

    def updateCurrentPosition(self, xyz):

        ### temporary code, might be implemented differently ###
        self.position[0] = xyz[0]
        self.position[1] = xyz[1]
        self.position[2] = xyz[2]
        print(f"{self.position =}")
        ### temporary code, might be implemented differently ###
        velocity_value = self.velocity_combobox.get()
        acceleration_value = self.acceleration_combobox.get()
        # now convert to numerical value (int or byte)
        interpolation_type = self.interpolation_combobox.get()

    def Jog(self, axis, direction):
        """Jogs the selected axis in a selected direction with constant speed."""
        if axis == "x":
            if direction == 0:
                self.updatePlot(event=None, manual=True, xyz=(self.position[0] + 5, self.position[1], self.position[2]))
            elif direction == 1:
                self.updatePlot(event=None, manual=True, xyz=(self.position[0] - 5, self.position[1], self.position[2]))
        elif axis == "y":
            if direction == 0:
                self.updatePlot(event=None, manual=True, xyz=(self.position[0], self.position[1] + 5, self.position[2]))
            elif direction == 1:
                self.updatePlot(event=None, manual=True, xyz=(self.position[0], self.position[1] - 5, self.position[2]))
        elif axis == "z":
            if direction == 0:
                self.updatePlot(event=None, manual=True, xyz=(self.position[0], self.position[1], self.position[2] + 5))
            elif direction == 1:
                self.updatePlot(event=None, manual=True, xyz=(self.position[0], self.position[1], self.position[2] - 5))

    #### move once ####
    def moveJog(self, event, axis, direction):
        print(f"Moving JOG axis {axis} in {direction} direction.")
        self.Jog(axis=axis, direction=direction)
        print(f"{self.position =}")
        # self.jobid = self.root.after(100, self.moveJog, event, axis, direction)

    #### not used currently ####
    def stopJog(self, event):
        self.root.after_cancel(self.jobid)
        print("Stopped JOG")
