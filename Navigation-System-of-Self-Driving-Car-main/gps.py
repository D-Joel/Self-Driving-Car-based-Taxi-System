import serial              
from time import sleep
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
#matplotlib.use('Agg')

#import webbrowser          
import sys
import time
import pyrebase
print("started...")
config = {
  "apiKey": "AIzaSyD8RLktLMGo0VCqcsJJphpVmRYZCvtW_KU",
  "authDomain": "self-driving-car-imu-fb6aa-default-rtdb.firebaseapp.com",
  "databaseURL": "https://self-driving-car-imu-fb6aa-default-rtdb.firebaseio.com/",
  "storageBucket": "self-driving-car-imu-fb6aa-default-rtdb.appspot.com"
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()


#print(a.val())
#for i in a.each():
#    print(i.val())
BBox = (78.03448,78.04245,15.76279,15.75654)
ruh_m = plt.imread("map.png")
def Delete_Old_DB():
    db.child("GPS").remove()
def GPS_Info():
    global NMEA_buff
    global lat_in_degrees
    global long_in_degrees
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_time = NMEA_buff[0]              
    nmea_latitude = NMEA_buff[1]                
    nmea_longitude = NMEA_buff[3]              
   
    #print("NMEA Time: ", nmea_time,'\n')
    #print ("NMEA Latitude:", nmea_latitude,"NMEA Longitude:", nmea_longitude,'\n')
   
    lat = float(nmea_latitude)                
    longi = float(nmea_longitude)              
   
    lat_in_degrees = convert_to_degrees(lat)    
    long_in_degrees = convert_to_degrees(longi)
   
   
#l1 =[]
#l2 =[]
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.4f" %(position)
    return position
   

def main():
    l1,l2=[],[]
   
    try:
        a = db.child("GPS").child("Longitude").get()
        b = db.child("GPS").child("Latitute").get()
        print(a)
        print(a.val())
        for i in a.val():
            l1.append(float(i))
        for j in b.val():
            l2.append(float(j))
           
    except:
        pass
    print(l1,'\n',l2)
    fig, ax = plt.subplots(figsize = (8,7))
    ax.scatter(l2, l1, zorder=1, alpha= 0.2, c='b', s=10)
    ax.set_title('Plotting Spatial Data on Riyadh Map')
    ax.set_xlim(BBox[0],BBox[1])
    ax.set_ylim(BBox[2],BBox[3])
   
    ax.imshow(ruh_m, zorder=0, extent = BBox, aspect= 'equal')
    fig2 = plt.figure()
    fig2.axes.append(ax)
    #plt.show()
    #ax.show()
    #plt.show()
    #plt.imsave('test.png',ax)
    print(ax)
    global NMEA_buff
    global GPGGA_buffer
    global lat_in_degrees
    global long_in_degrees
    ser = serial.Serial ("/dev/serial0")              #baud rate
    GPGGA_buffer = 0
    NMEA_buff = 0
    lat_in_degrees = 0
    long_in_degrees = 0

    while True:
        #try:
       
            received_data = (str)(ser.readline())
            #print(received_data)      #read NMEA string received
            GPGGA_data_available = received_data.find("$GPGGA,")          
            if (GPGGA_data_available>0):
                GPGGA_buffer = received_data.split("$GPGGA,",1)[1] 
                NMEA_buff = (GPGGA_buffer.split(','))               
                GPS_Info()
                #get time, latitude, longitude
                l1.append(lat_in_degrees)
                l2.append(long_in_degrees)
     
                print(" >lat in degrees:", lat_in_degrees," long in degree: ", long_in_degrees, '\n')
                a = db.child("GPS").update({"Latitute":l1,"Longitude":l2})
                #print("> ",a)
                #return [lat_in_degrees,long_in_deA
