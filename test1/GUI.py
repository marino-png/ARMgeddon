#import the libraries 
import serial 
from tkinter import *
import tkinter as tk

#select the right port
commPort = "/dev/cu.usbmodemDC5475D1296C2"
ser = serial.Serial(commPort, baudrate="9600", timeout=1)

#function that transmit values
def turnOnLed():
    ser.write(b'x')

def turnOffLed():
    ser.write(b'z')

def changeBaseAngle(value):
    ser.write(f"{value}\n".encode())

def changeXValue(value):
    ser.write(f"{value}\n".encode())

def changeYValue(value):
    ser.write(f"{value}\n".encode())

#build the gui
root =  Tk()
root.title('ARMgeddon')

btnOn = tk.Button(root, text="Open",command=turnOnLed)
btnOff = tk.Button(root, text="Close",command=turnOffLed)
sliderBase = tk.Scale(
    root,
    from_=0,           # Minimum value of slider
    to=180,            # Maximum value of slider
    orient="horizontal", # Orientation: horizontal or vertical
    length=200,        # Length of the slider
    command=changeBaseAngle # Callback function on value change
)
sliderX = tk.Scale(
    root,
    from_=0,           # Minimum value of slider
    to=10,            # Maximum value of slider
    orient="horizontal", # Orientation: horizontal or vertical
    length=200,        # Length of the slider
    command=changeXValue # Callback function on value change
)
sliderY = tk.Scale(
    root,
    from_=0,           # Minimum value of slider
    to=10,            # Maximum value of slider
    orient="horizontal", # Orientation: horizontal or vertical
    length=200,        # Length of the slider
    command=changeYValue # Callback function on value change
)

labelBase = tk.Label(root, text="base angle")
labelX = tk.Label(root, text="X value")
labelY = tk.Label(root, text="Y value")

btnOn.grid(row=0, column=0)
btnOff.grid(row=0, column=1)
sliderBase.grid(row=1, column=1)
labelX.grid(row=2,column=0)
sliderX.grid(row=2,column=1)
labelY.grid(row=3,column=0)
sliderY.grid(row=3,column=1)
labelBase.grid(row=1,column=0)

root.geometry("350x250")
root.mainloop()