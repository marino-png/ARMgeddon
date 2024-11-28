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

def on_slider_change(value):
    ser.write(f"{value}\n".encode())

#build the gui
root =  Tk()
root.title('ARMgeddon')

btnOn = tk.Button(root, text="Open",command=turnOnLed)
btnOn.grid(row=0, column=0)

btnOff = tk.Button(root, text="Close",command=turnOffLed)
btnOff.grid(row=0, column=1)

sliderBase = tk.Scale(
    root,
    from_=0,           # Minimum value of slider
    to=180,            # Maximum value of slider
    orient="horizontal", # Orientation: horizontal or vertical
    length=200,        # Length of the slider
    command=on_slider_change # Callback function on value change
)
sliderBase.grid(row=1, column=1)

labelBase = tk.Label(root, text="base angle")
labelBase.grid(row=1,column=0)

root.geometry("350x350")
root.mainloop()