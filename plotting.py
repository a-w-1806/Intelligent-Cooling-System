import serial
import binascii
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
is_ipython = 'inline' in matplotlib.get_backend()
if is_ipython:
    from IPython import display


ser = serial.Serial('COM3',9600)
print(ser.name)
print(ser.is_open)#检验串口是否打开
plt.ion()
plt.figure("Temperature Graph")

temp=[] 

def plot_durations(y):
    plt.figure(1)
    plt.clf()
    plt.plot(y, color = 'r')
    plt.ylim(25,35)
    plt.grid(True)
    plt.xticks([])
    plt.pause(0.001)  # pause a bit so that plots are updated
    if is_ipython:
        display.clear_output(wait=True)
        display.display(plt.gcf())



HLflag=0;
while(1):
     if ser.is_open :
       s = str(binascii.b2a_hex(ser.read(1)))[2:-1]
       if len(s)>0 : 
          if HLflag==0:
             a=int(s , 16)
             HLflag=1
          else:
             b=int(s , 16)
             HLflag=0
             temp.append(a+b/100)
             plot_durations(np.array(temp))
