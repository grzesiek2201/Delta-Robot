import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
import matplotlib.pyplot as plt
import deltarobot
from tkinter.filedialog import asksaveasfile

FONT = "Times New Roman"
TEXT_SIZE = 12

class DeltaGUI():

    def __init__(self):

        self.delta = deltarobot.DeltaRobot()###

        self.jobid = None
        self.position = [0, 0, 0] # not used anywhere yet, use it by adding a certain amount while holding JOG?
        self.root = tk.Tk()
        self.root.title("Delta GUI")
        self.root.config(padx=50, pady=20, bg="white")
        self.root.iconbitmap("delta icon.ico")

        #Menus
        self.menubar = tk.Menu(self.root)
        self.root.config(menu=self.menubar)
        self.file_menu = tk.Menu(self.menubar, tearoff=0)
        self.file_menu.add_command(label='Exit', command=self.root.quit)
        self.menubar.add_cascade(label="File", menu=self.file_menu, underline=0)

        self.program_menu = tk.Menu(self.menubar, tearoff=0)
        self.program_menu.add_cascade(label="New", command=self.programPopup)
        self.program_menu.add_cascade(label="Load")

        self.menubar.add_cascade(label="Program", menu=self.program_menu, underline=0)
        #Labels
        self.x_position_label = tk.Label(self.root, text="X [mm]", font=(FONT, TEXT_SIZE-3), bg="White")
        self.x_position_label.grid(row=0, column=2)
        self.y_position_label = tk.Label(self.root, text="Y [mm]", font=(FONT, TEXT_SIZE-3), bg="White")
        self.y_position_label.grid(row=0, column=3)
        self.z_position_label = tk.Label(self.root, text="Z [mm]", font=(FONT, TEXT_SIZE-3), bg="White")
        self.z_position_label.grid(row=0, column=4)

        self.position_label = tk.Label(self.root, text="Position", font=(FONT, TEXT_SIZE), bg="White")
        self.position_label.grid(padx=10, row=1, column=1)

        self.speed_label = tk.Label(self.root, text="Velocity", font=(FONT, TEXT_SIZE), bg="White")
        self.speed_label.grid(padx=10, pady=10, row=2, column=1)

        self.acceleration_label = tk.Label(self.root, text="Acceleration", font=(FONT, TEXT_SIZE), bg="White")
        self.acceleration_label.grid(padx=10, pady=10, row=3, column=1)

        self.interpolation_label = tk.Label(self.root, text="Interpolation", font=(FONT, TEXT_SIZE), bg="White")
        self.interpolation_label.grid(padx=10, pady=10, row=4, column=1)

        self.jog_label = tk.Label(self.root, text="JOG", font=(FONT, TEXT_SIZE), bg="White")
        self.jog_label.grid(padx=10, pady=10, row=4, column=1, rowspan=3)

        self.coord_warning_label = tk.Label(self.root, text="Coordinates out of range!", font=(FONT, TEXT_SIZE), bg="White")

        #Entries
        self.x_position_entry = tk.Entry(self.root, width=10)
        self.x_position_entry.grid(row=1, column=2)
        self.x_position_entry.insert(0, "0")
        self.x_position_entry.bind("<Return>", self.updatePlot)

        self.y_position_entry = tk.Entry(self.root, width=10)
        self.y_position_entry.grid(row=1, column=3)
        self.y_position_entry.insert(0, "0")
        self.y_position_entry.bind("<Return>", self.updatePlot)

        self.z_position_entry = tk.Entry(self.root, width=10)
        self.z_position_entry.grid(row=1, column=4)
        self.z_position_entry.insert(0, "0")
        self.z_position_entry.bind("<Return>", self.updatePlot)


        #Buttons
        #ADD COMMAND=
        self.x_plus_button = tk.Button(self.root, text="X+", width=7)
        self.x_plus_button.bind('<ButtonPress-1>', lambda event, axis="x", direction=0: self.moveJog(event, axis=axis, direction=direction))
        self.x_plus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.x_plus_button.grid(pady=5, row=5, column=2)

        self.x_minus_button = tk.Button(self.root, text="X-", width=7)
        self.x_minus_button.bind('<ButtonPress-1>', lambda event, axis="x", direction=1: self.moveJog(event, axis=axis, direction=direction))
        self.x_minus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.x_minus_button.grid(pady=5, row=5, column=3)

        self.y_plus_button = tk.Button(self.root, text="Y+", width=7)
        self.y_plus_button.bind('<ButtonPress-1>', lambda event, axis="y", direction=0: self.moveJog(event, axis=axis, direction=direction))
        self.y_plus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.y_plus_button.grid(pady=5, row=6, column=2)

        self.y_minus_button = tk.Button(self.root, text="Y-", width=7)
        self.y_minus_button.bind('<ButtonPress-1>', lambda event, axis="y", direction=1: self.moveJog(event, axis=axis, direction=direction))
        self.y_minus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.y_minus_button.grid(pady=5, row=6, column=3)

        self.z_plus_button = tk.Button(self.root, text="Z+", width=7)
        self.z_plus_button.bind('<ButtonPress-1>', lambda event, axis="z", direction=0: self.moveJog(event, axis=axis, direction=direction))
        self.z_plus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.z_plus_button.grid(pady=5, row=7, column=2)

        self.z_minus_button = tk.Button(self.root, text="Z-", width=7)
        self.z_minus_button.bind('<ButtonPress-1>', lambda event, axis="z", direction=1: self.moveJog(event, axis=axis, direction=direction))
        self.z_minus_button.bind('<ButtonRelease-1>', self.stopJog)
        self.z_minus_button.grid(pady=5, row=7, column=3)

        self.move_button = tk.Button(self.root, text="Move", font=(FONT, TEXT_SIZE), bg="White")
        self.move_button.grid(pady=5, row=1, column=5, rowspan=1, columnspan=1)
        self.move_button.bind('<Button-1>', self.updatePlot)

        #Combobox
        self.speed_tuple = ('100%', '90%', '80%', '70%', '60%', '50%', '40%', '30%', '20%', '10%')
        self.speeds = tk.StringVar()
        self.speed_combobox = ttk.Combobox(self.root, textvariable=self.speeds, width=7)
        self.speed_combobox['values'] = self.speed_tuple
        self.speed_combobox['state'] = 'readonly'
        self.speed_combobox.current(9)
        self.speed_combobox.grid(row=2, column=2)

        self.acceleration_tuple = ('100%', '90%', '80%', '70%', '60%', '50%', '40%', '30%', '20%', '10%')
        self.accelerations = tk.StringVar()
        self.acceleration_combobox = ttk.Combobox(self.root, textvariable=self.accelerations, width=7)
        self.acceleration_combobox['values'] = self.acceleration_tuple
        self.acceleration_combobox['state'] = 'readonly'
        self.acceleration_combobox.current(9)
        self.acceleration_combobox.grid(row=3, column=2)

        self.interpolation_tuple = ('Joint', 'Linear', 'Circular')
        self.interpolations = tk.StringVar()
        self.interpolation_combobox = ttk.Combobox(self.root, textvariable=self.interpolations, width=7)
        self.interpolation_combobox['values'] = self.interpolation_tuple
        self.interpolation_combobox['state'] = 'readonly'
        self.interpolation_combobox.current(0)
        self.interpolation_combobox.grid(row=4, column=2)

        #Canvas
        self.createPlot()
        tk.mainloop()

    def programPopup(self):

        # Create window
        self.program_creator = tk.Toplevel(self.root)
        self.program_creator.title("Program creator")
        self.program_creator.config(padx=10, pady=10, bg="white")
        self.program_creator.iconbitmap("delta icon.ico")

        self.programPointFrame(self.program_creator)
        self.addPointFrame(self.program_creator)
        self.saveProgramFrame(self.program_creator)

    def deletePointFromList(self):  # only in GUI, no effect on points yet
        for item in self.program_tree.selection():
            self.program_tree.delete(item)

    def moveUpSelected(self):  # only in GUI, no effect on points yet
        for item in self.program_tree.selection():
            self.program_tree.move(item, self.program_tree.parent(item), self.program_tree.index(item)-1)

    def moveDownSelected(self):  # only in GUI, no effect on points yet
        for item in self.program_tree.selection():
            self.program_tree.move(item, self.program_tree.parent(item), self.program_tree.index(item)+1)

    def programPointFrame(self, master):
        # Create frame - program points
        self.fr_program_point = tk.Frame(master, bg="White")
        self.fr_program_point.grid(row=0, column=0, rowspan=2, sticky='nsew')

        # Treeview fr program points
        style = ttk.Style()
        style.layout("Treeview", [('Treeview.treearea', {'sticky': 'nswe'})])  # remove the border
        self.program_tree = ttk.Treeview(self.fr_program_point, columns=('no.', 'X', 'Y', 'Z', 'v', 'a', 'Interpolation'), height=15)

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
        self.program_tree.insert('', index='end', text="", values=(1, 80.0, 30.0, -1300.0, 10, 50, "Linear"))
        self.program_tree.insert('', index='end', text="", values=(2, 40.0, 30.0, -1350.0, 10, 50, "Linear"))
        self.program_tree.insert('', index='end', text="", values=(3, 30.0, 80.0, -1380.0, 10, 50, "Linear"))

        # Buttons fr program points
        self.delete_selected_button = tk.Button(self.fr_program_point, text="Delete selected", font=(FONT, TEXT_SIZE), bg="White", command=self.deletePointFromList)
        self.delete_selected_button.grid(padx=10, row=2, column=0)
        self.move_up_button = tk.Button(self.fr_program_point, text="⬆", width=5, font=(FONT, TEXT_SIZE), bg="White", command=self.moveUpSelected)
        self.move_up_button.grid(padx=0, row=2, column=5)
        self.move_up_button = tk.Button(self.fr_program_point, text="⬇", width=5, font=(FONT, TEXT_SIZE), bg="White", command=self.moveDownSelected)
        self.move_up_button.grid(padx=0, row=2, column=6)

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
        self.y_entry = tk.Entry(self.fr_add_point, width=10)
        self.y_entry.grid(padx=10, pady=10, row=1, column=1)
        self.y_entry.insert(0, "0")
        self.z_entry = tk.Entry(self.fr_add_point, width=10)
        self.z_entry.grid(padx=10, row=1, column=2)
        self.z_entry.insert(0, "0")

        # Buttons fr add points
        self.add_point_button = tk.Button(self.fr_add_point, text="Add", font=(FONT, TEXT_SIZE), bg="White", width=12)
        self.add_point_button.grid(padx=10, row=5, column=0, columnspan=3)

        # Labels fr add point
        self.v_label = tk.Label(self.fr_add_point, text="Velocity", font=(FONT, TEXT_SIZE), bg="White")
        self.v_label.grid(row=2, column=0)
        self.a_label = tk.Label(self.fr_add_point, text="Acceleration", font=(FONT, TEXT_SIZE), bg="White")
        self.a_label.grid(row=3, column=0)
        self.i_label = tk.Label(self.fr_add_point, text="Interpolation", font=(FONT, TEXT_SIZE), bg="White")
        self.i_label.grid(row=4, column=0)

        # Combobox fr add point
        self.speed_fr_add_combobox = ttk.Combobox(self.fr_add_point, textvariable=self.speeds, width=7)
        self.speed_fr_add_combobox['values'] = self.speed_tuple
        self.speed_fr_add_combobox['state'] = 'readonly'
        self.speed_fr_add_combobox.current(9)
        self.speed_fr_add_combobox.grid(pady=10, row=2, column=1)

        self.acc_fr_add_combobox = ttk.Combobox(self.fr_add_point, textvariable=self.accelerations, width=7)
        self.acc_fr_add_combobox['values'] = self.acceleration_tuple
        self.acc_fr_add_combobox['state'] = 'readonly'
        self.acc_fr_add_combobox.current(9)
        self.acc_fr_add_combobox.grid(pady=10, row=3, column=1)

        self.interpolation_fr_add_combobox = ttk.Combobox(self.fr_add_point, textvariable=self.interpolations, width=7)
        self.interpolation_fr_add_combobox['values'] = self.interpolation_tuple
        self.interpolation_fr_add_combobox['state'] = 'readonly'
        self.interpolation_fr_add_combobox.current(0)
        self.interpolation_fr_add_combobox.grid(pady=20, row=4, column=1)

    def saveProgramFrame(self, master):
        # Create frame - save program
        self.fr_save_program = tk.Frame(master, bg="White")
        self.fr_save_program.grid(padx=20, pady=10, row=1, column=1, sticky='nsew')

        # Labels - fr save program
        self.name_label = tk.Label(self.fr_save_program, text="Name:", font=(FONT, TEXT_SIZE), bg="White")
        self.name_label.grid(padx=10, pady=10, row=0, column=0, sticky='w')

        # Entries - fr save program
        self.name_entry = tk.Entry(self.fr_save_program, width=25)
        self.name_entry.grid(pady=10, row=0, column=1, columnspan=2)

        # Buttons = fr save program
        self.save_button = tk.Button(self.fr_save_program, text="Save program", font=(FONT, TEXT_SIZE), width=16, bg='White', command=self.saveProgram)
        self.save_button.grid(row=1, column=0, columnspan=3, sticky='e')

    def saveProgram(self):
        program_name = str(self.name_entry.get())+'.txt'
        files = [('Text Document', '*.txt')]
        file = "asdadasdadadadadasda"
        file = asksaveasfile(filetypes=files, defaultextension=files)

    def createPlot(self):
        # create figure
        self.fig = plt.figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_xlim(-800, 800)
        self.ax.set_ylim(-800, 800)
        self.ax.set_zlim(-2000, 500)

        # create canvas from backend of matplotlib so it can be displayed in gui
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=2, column=5, rowspan=10, columnspan=10)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def updatePlot(self, event):
        """Update the plot with supplied x y z coordinates"""

        try:
            self.getCoordinates()  # catch an out of range error
            temp = self.delta.vert_coords.coordinates_x[2]  # catch an error when creating model that is out of range
        except TypeError as e:
            self.coord_warning_label.grid(row=0, column=10, rowspan=2)  # show 'out of range' label
            print(e)
        # except IndexError as e:
        #     print("Trying to create the model with coordinates out of range: ")
        #     print(e)
        except ValueError:
            print("Write only number in the coordinates entries.")
        else:
            self.coord_warning_label.grid_forget()

            self.ax.clear()

            for i in range(3):
                self.ax.plot(self.delta.vert_coords.coordinates_x[i], self.delta.vert_coords.coordinates_y[i], self.delta.vert_coords.coordinates_z[i], 'black')  # links
            self.ax.plot(self.delta.vert_coords.coordinates_x[3], self.delta.vert_coords.coordinates_y[3], self.delta.vert_coords.coordinates_z[3])  # Effector
            self.ax.plot(self.delta.vert_coords.coordinates_x[4], self.delta.vert_coords.coordinates_y[4], self.delta.vert_coords.coordinates_z[4], 'ro')  # TCP
            self.ax.plot(self.delta.Bvx, self.delta.Bvy, self.delta.Bvz)
            self.ax.set_xlim(-800, 800)
            self.ax.set_ylim(-800, 800)
            self.ax.set_zlim(-2000, 500)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
    
    def getCoordinates(self):
        """Sends x,y,z coordinates, speed % and interpolation type as a json through serial port,
        returns a tuple of lists of x, y and z coordinates to draw"""
        x_coordinate = float(self.x_position_entry.get())
        y_coordinate = float(self.y_position_entry.get())
        z_coordinate = float(self.z_position_entry.get())
        speed_value = self.speed_combobox.get()
        acceleration_value = self.acceleration_combobox.get()
        # now convert to numerical value (int or byte)
        interpolation_type = self.interpolation_combobox.get()
        # now convert to numerical value
        # send data to arduino

        self.delta.calculateIK(x_coordinate, y_coordinate, z_coordinate)

    def Jog(self, axis, direction):
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
    def moveJog(self, event, axis, direction):
        print(f"Moving JOG axis {axis} in {direction} direction.")
        self.jobid = self.root.after(300, self.moveJog, event, axis, direction)

    def stopJog(self, event):
        self.root.after_cancel(self.jobid)
        print("Stopped JOG")