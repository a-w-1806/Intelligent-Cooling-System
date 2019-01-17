import serial
import binascii
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

ser = serial.Serial('COM3',9600)
print(ser.name)
print(ser.is_open) #Check if serial port is opened
plt.ion()
plt.figure("Temperature Graph")

temp = [] 

def plot_durations(y):
    plt.figure(1)
    plt.clf()
    plt.plot(y, color = 'r')
    plt.ylim(25, 35)
    plt.grid(True)
    plt.xticks([])
    plt.pause(0.001)  # pause a bit so that plots are updated

HLflag = 0
while(1):
    if ser.is_open :
        s = str(binascii.b2a_hex(ser.read(1)))[2:-1]
        if len(s) > 0 : 
            if HLflag == 0:
                a = int(s , 16)
                HLflag = 1
            else:
                b = int(s , 16)
                HLflag = 0
                
                temp.append(a + b / 100)
                plot_durations(np.array(temp))
