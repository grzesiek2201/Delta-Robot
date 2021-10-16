import PySimpleGUI as sg
import serial
import time

sg.theme('black')

ser = serial.Serial('COM4', 115200, timeout=1)
layout = [
    [sg.Input(key='-IN-'), sg.Button('Send', bind_return_key=True)],
    ]

window = sg.Window('Hiperadvanced smd feeder 3000', layout)
# while True:
#     ser.write(str.encode("asffwee"))
#     receive = ser.readline()
#     while len(receive) <= 0:
#         receive = ser.readline()
#         time.sleep(1)
#         ser.flush()
#         print(len(receive))
#     print(receive)
#     print(type(receive))


while True:  # The Event Loop
    event, values = window.read()

    if event == 'Send':
        ser.write(str.encode("afwefwe"))
        window.FindElement('-IN-').Update('')
    if event == sg.WIN_CLOSED or event == 'Exit':
        break
    if ser.isOpen():
        receive = ser.readline()
        print(receive)
# time.sleep(5)
# if ser.isOpen():
#     ser.write(str.encode("afwefwe"))
#     receive = ser.readline()
#     while len(receive) == 0:
#         receive=ser.readline()
#     print(receive)

window.close()