import smbus #import SMBus module of I2C
from time import sleep          #import
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


import pyrebase

config = {
  "apiKey": "AIzaSyD8RLktLMGo0VCqcsJJphpVmRYZCvtW_KU",
  "authDomain": "self-driving-car-imu-fb6aa-default-rtdb.firebaseapp.com",
  "databaseURL": "https://self-driving-car-imu-fb6aa-default-rtdb.firebaseio.com/",
  "storageBucket": "self-driving-car-imu-fb6aa-default-rtdb.appspot.com"
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()

def MPU_Init():
#write to sample rate register
bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

#Write to power management register
bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

#Write to Configuration register
bus.write_byte_data(Device_Address, CONFIG, 0)

#Write to Gyro configuration register
bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

#Write to interrupt enable register
bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
   
        #concatenate higher and lower value
        value = ((high << 8) | low)
       
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value
def animate(i, xs, ys,lxa,lya,lza,lgx,lgy,lgz):

    print("i : ",i)
    # Add x and y to lists
##    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    xs.append(dt.datetime.now().strftime('%H%M%S'))
    ## #Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

#Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)


#Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0
    lxa.append(Ax)
    lya.append(Ay)
    lza.append(Az)

    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0
    lgx.append(Gx)
    lgy.append(Gy)
    lgz.append(Gz)
    print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)

    db.child("IMU").set({"GX":lgx,"GY":lgy,"GZ":lgz,"AX":lxa,"AY":lya,"AZ":lza})
    ys.append(Ax)  # we are plotting Ax timebeing
    sleep(1)
   

    # Limit x and y lists to 40 items
   
    xs = xs[-40:]
    ys = ys[-40:]

    print("ys: ",ys)
    print("xs : ",xs)
    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    print("Format plot")
    plt.xticks(rotation= 45, ha='right')
    plt.subplots_adjust(bottom=0.20)
   
    plt.title('acceleration data over Time')
    plt.ylabel('m/s')
   
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []
bus = smbus.SMBus(1) # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")
lxa=[]
lya=[]
lza=[]
lgx,lgy,lgz =[],[],[]
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys,lxa,lya,lza,lgx,lgy,lgz), interval=5000)
plt.show()
##while True:
##
## #Read Accelerometer raw value
## acc_x = read_raw_data(ACCEL_XOUT_H)
## acc_y = read_raw_data(ACCEL_YOUT_H)
## acc_z = read_raw_data(ACCEL_ZOUT_H)
##
## #Read Gyroscope raw value
## gyro_x = read_raw_data(GYRO_XOUT_H)
## gyro_y = read_raw_data(GYRO_YOUT_H)
## gyro_z = read_raw_data(GYRO_ZOUT_H)
##
## #Full scale range +/- 250 degree/C as per sensitivity scale factor
## Ax = acc_x/16384.0
## Ay = acc_y/16384.0
## Az = acc_z/16384.0
##
## Gx = gyro_x/131.0
## Gy = gyro_y/131.0
## Gz = gyro_z/131.0
## ys.append(Ax)
#### plt.plot(ys)
#### plt.show()
## print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
## sleep(1)
