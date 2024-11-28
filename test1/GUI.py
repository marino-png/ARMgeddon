import serial 
from tkinter import *
import tkinter as tk

commPort = "/dev/cu.usbmodemDC5475D1296C2"
ser = serial.Serial(commPort, baudrate="9600", timeout=1)

def turnOnLed():
    ser.write(b'x')

def turnOffLed():
    ser.write(b'z')

root =  Tk()
root.title('ARMgeddon')

btnOn = tk.Button(root, text="Turn on",command=turnOnLed)
btnOn.grid(row=0, column=0)

btnOff = tk.Button(root, text="Turn off",command=turnOffLed)
btnOff.grid(row=0, column=1)

root.geometry("350x350")
root.mainloop()