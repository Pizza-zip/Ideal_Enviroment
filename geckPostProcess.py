# -*- coding: utf-8 -*-
"""
Created on Sat Jun 15 21:58:06 2024

@author: Thomas Wolski

First Itteration of G.E.C.K Data collection & Post processing
    with intention of long term data collection and live* results.
"""

import numpy as np   
import pandas as pd
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial

import serial
import time


def readserial(comport, baudrate, timestamp=False):

    ser = serial.Serial(comport, baudrate, timeout=0.1)         # 1/timeout is the frequency at which the port is read
    
    data_x = np.array([])
    data_y = np.array([])
    
    
    while True:
        time_start = time.time()
        
        file_name = "Data_saved_%s.txt" % (time.strftime('%y%m%d_%H_%M_%S'))
        
        f = open(file_name,"w+")
        
        save_state = True
        plt.ion()
        plt.ylabel('Soil Moisture (%)')
        plt.xlabel('Time (seconds)')
        plt.grid(True)
        while save_state:
            data = ser.readline().decode().strip()
            if data and timestamp:
                data_temp = np.array(data.split())
                
                data_x = np.append(data_x, float(data_temp[0]));
                data_y = np.append(data_y, float(data_temp[1]));
                
                timestamp = time.strftime('%D:%H:%M:%S')
                print(f'{timestamp} > {data}')
                f.write(f'{timestamp} > {data}\r\n')
                
                plt.plot(data_x, data_y)
                plt.draw()
                plt.pause(0.1)
                            
            time_end = time.time()
            
            if ((time_end - time_start)/60.0 >= 5.0):
                save_state = False
        
        f.close()
        plt.show()
if __name__ == '__main__':

    readserial('COM3', 115200, True)
    